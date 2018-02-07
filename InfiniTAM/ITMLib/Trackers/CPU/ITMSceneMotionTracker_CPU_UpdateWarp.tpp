//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
//  Copyright (c) 2017-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#pragma once



//stdlib
#include <chrono>

//local
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"
#include "../../Utils/ITMVoxelFlags.h"


using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::UpdateWarpField(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) {

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int noTotalEntries = canonicalScene->index.noTotalEntries;
	float maxWarpUpdateLength = 0.0f;

	//_DEBUG
#ifdef _DEBUG
#ifdef PRINT_ADDITIONAL_STATS
	double aveCanonicaSdf = 0.0;
	double aveLiveSdf = 0.0;
	double aveSdfDiff = 0.0;
	int consideredVoxelCount = 0;
	double aveWarpDist = 0.0;
	double aveWarpDistBoundary = 0.0;
	int boundaryVoxelCount = 0;

	int voxelOscillationCount = 0;
	int ignoredVoxelOscillationCount = 0;
#ifndef WITH_OPENMP
//_DEBUG
	std::vector<std::tuple<int, int>> voxelOscillations;
#endif // WITH_OPENMP
#endif // PRINT_ADDITIONAL_STATS

#ifdef PRINT_ENERGY_STATS
	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalEnergy = 0.0;
#endif

	float maxWarpLength = 0.0;
	const std::string red("\033[0;31m");
	const std::string green("\033[0;32m");
	const std::string yellow("\033[0;33m");
	const std::string cyan("\033[0;36m");
	const std::string reset("\033[0m");
#endif

	const float epsilon = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon;
	// fraction of narrow band (non-truncated region) half-width that a voxel spans
	const float unity = liveScene->sceneParams->voxelSize / liveScene->sceneParams->mu;

	TIC(timeWarpUpdateCompute);

	//compute the update, don't apply yet (computation depends on previous warp for neighbors,
	//no practical way to keep those buffered with the hash & multithreading in mind)
#ifdef WITH_OPENMP
#ifndef OPENMP_WARP_UPDATE_COMPUTE_DISABLE

#if defined(PRINT_ADDITIONAL_STATS) && defined(PRINT_ENERGY_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, aveLiveSdf, aveWarpDist, aveSdfDiff, boundaryVoxelCount, aveWarpDistBoundary, totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy, voxelOscillationCount, ignoredVoxelOscillationCount)
#else
#if defined(PRINT_ADDITIONAL_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, aveLiveSdf, aveWarpDist, aveSdfDiff, boundaryVoxelCount, aveWarpDistBoundary, voxelOscillationCount, ignoredVoxelOscillationCount)
#elif defined(PRINT_ENERGY_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy)
#else
#pragma omp parallel for firstprivate(canonicalCache, liveCache)
#endif
#endif //both sets of debug vars

#endif// ndef OPENMP_WARP_UPDATE_COMPUTE_DISABLE
#else

#endif// WITH_OPENMP


	for (int hash = 0; hash < noTotalEntries; hash++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];
		if (currentCanonicalHashEntry.ptr < 0) continue;
		//position of the current entry in 3D space
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];

					//=================================== TRUNCATION REGION CHECKS =====================================
					float liveSdf;
					bool foundInLive;
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;
					liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache,
					                                 foundInLive);

					//almost no restriction -- Mira's case with addition of VOXEL_UNKNOWN flag checking
					bool emptyInCanonical = canonicalVoxel.flags == ITMLib::VOXEL_UNKNOWN;
					// the latter condition needs to be included since sometimes, even if some live voxels in the lookup
					// neighborhood are non-truncated, they ally may be a whole voxel away from the warped position,
					// which would then result in a live SDF lookup equivalent to that in a truncated region.
					bool emptyInLive = !foundInLive || (1.0 - std::abs(liveSdf) < FLT_EPSILON);
					bool ignoreVoxelDueToOscillation = canonicalVoxel.flags & ITMLib::VOXEL_OSCILLATION_DETECTED_TWICE;
#ifdef PRINT_ADDITIONAL_STATS
					if (ignoreVoxelDueToOscillation) ignoredVoxelOscillationCount++;
#endif
					if ((emptyInCanonical && emptyInLive) ||
					    ignoreVoxelDueToOscillation)
						continue;

					float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);
					bool useColor;
					Vector3f liveColor, liveSdfJacobian, liveColorJacobian, liveSdf_Center_WarpForward;
					Matrix3f warpedSdfHessian;
					//_DEBUG
					bool boundary = false, printResult = false;
#ifdef PRINT_SINGLE_VOXEL_RESULT
					if (originalPosition == (ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos1)) {
						printResult = true;
						std::cout << std::endl << "Priniting voxel at " << originalPosition << ". ";
						std::cout <<  "Source SDF vs. target SDF: " << canonicalSdf
								  << "-->" << liveSdf << std::endl << "Warp: " << canonicalVoxel.warp_t;
						std::cout << " Live SDF found: " << liveSdfFound;
						std::cout << std::endl;
					}
#endif
#ifdef PRINT_TIME_STATS
					TIC(timeDataJandHCompute);
#endif

					//================================ RETRIEVE NEIGHBOR'S WARPS =======================================
					const int neighborhoodSize = 9;
					Vector3f warp_tNeighbors[neighborhoodSize];
					bool neighborFound[neighborhoodSize];
					//    0        1        2          3         4         5           6         7         8
					//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
					findPoint2ndDerivativeNeighborhoodWarp(warp_tNeighbors/*x9*/, neighborFound, originalPosition,
					                                       canonicalVoxels, canonicalHashTable, canonicalCache);
					for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
						if (!neighborFound[iNeighbor]) {
							warp_tNeighbors[iNeighbor] = canonicalVoxel.warp_t;
							boundary = true;
						}
					}
#ifdef PRINT_ADDITIONAL_STATS
					if (boundary) boundaryVoxelCount++;
#endif
					// =============================== DATA & LEVEL SET TERMS ==========================================
					Vector3f deltaEData = Vector3f(0.0f);
					Vector3f deltaELevelSet = Vector3f(0.0f);
					float diffSdf = 0.0f, sdfJacobianNormMinusUnity = 0.0f;

					//if we are in the truncated region of canonical or live, we completely disregard the data and level set terms:
					//there is no sufficient information to compute those terms. There we rely solely on the killing regularizer
					if (!(emptyInCanonical || emptyInLive)) {
						//=================================== DATA TERM ================================================
						if (std::abs(canonicalSdf) >
						    ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::colorSdfThreshold) {
							useColor = false;
							//This is jacobian of the live frame at the lookup (warped) position
							ComputePerPointWarpedLiveJacobian
									(originalPosition,
									 canonicalVoxel.warp_t,

									 liveVoxels,
									 liveHashTable,
									 liveCache,

									 liveSdf, liveSdfJacobian,
									 liveSdf_Center_WarpForward, printResult);
						} else {
							useColor = true;
							//This is jacobian of the live frame at the lookup (warped) position
							ComputePerPointWarpedLiveJacobian
									(originalPosition, canonicalVoxel.warp_t,
									 liveVoxels, liveHashTable, liveCache,
									 liveSdf, liveColor, liveSdfJacobian,
									 liveSdf_Center_WarpForward, liveColorJacobian);
						}
						//Compute data term error / energy
						diffSdf = liveSdf - canonicalSdf;

						deltaEData = liveSdfJacobian * diffSdf;
						if (useColor) {
							float diffColor =
									ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightColorDataTerm *
									squareDistance(liveColor, TO_FLOAT3(canonicalVoxel.clr) / 255.f);
							deltaEData += liveColorJacobian * diffColor;
						}
						//=================================== LEVEL SET TERM ===============================================
						ComputeWarpHessian(warp_tNeighbors, originalPosition, liveSdfJacobian,
						                   liveSdf_Center_WarpForward, liveSdf, liveVoxels,
						                   liveHashTable, liveCache, warpedSdfHessian,
						                   printResult);
						float sdfJacobianNorm = length(liveSdfJacobian);

						sdfJacobianNormMinusUnity = sdfJacobianNorm - unity;
						deltaELevelSet =
								sdfJacobianNormMinusUnity * (liveSdfJacobian * liveSdfJacobian) /
								(sdfJacobianNorm + ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon);
					}
					//=================================== KILLING TERM =================================================
#ifdef PRINT_TIME_STATS
					TOC(timeDataJandHCompute);
					TIC(timeWarpJandHCompute);
#endif
					Matrix3f warpJacobian;
					Matrix3f warpHessian[3];

					ComputePerPointWarpJacobianAndHessian<TVoxelCanonical, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, warp_tNeighbors, neighborFound,
							warpJacobian, warpHessian, printResult);

#ifdef PRINT_TIME_STATS
					TOC(timeWarpJandHCompute);
					TIC(timeUpdateTermCompute);
#endif

					const float gamma = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::rigidityEnforcementFactor;
					float onePlusGamma = 1.0f + gamma;
					// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
					// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
					// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
					Matrix3f& H_u = warpHessian[0];
					Matrix3f& H_v = warpHessian[1];
					Matrix3f& H_w = warpHessian[2];
					float KillingDeltaEu, KillingDeltaEv, KillingDeltaEw;


					KillingDeltaEu =
							-2.0f * ((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy + gamma * H_w.xz);
					KillingDeltaEv =
							-2.0f * ((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy + gamma * H_w.yz);
					KillingDeltaEw =
							-2.0f * ((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz + gamma * H_u.xz);


					Vector3f deltaEKilling = Vector3f(KillingDeltaEu,
					                                  KillingDeltaEv,
					                                  KillingDeltaEw);
#ifdef PRINT_TIME_STATS
					TOC(timeUpdateTermCompute);
#endif
					//_DEBUG
#ifdef PRINT_ENERGY_STATS
					//=================================== ENERGY =======================================================
					// KillingTerm Energy
					Matrix3f warpJacobianTranspose = warpJacobian.t();

					float localSmoothnessEnergy = dot(warpJacobian.getColumn(0), warpJacobian.getColumn(0)) +
					                              dot(warpJacobian.getColumn(1), warpJacobian.getColumn(1)) +
					                              dot(warpJacobian.getColumn(2), warpJacobian.getColumn(2));

					float localKillingEnergy = localSmoothnessEnergy +
					                           gamma *
					                           (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
					                            dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
					                            dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));
#endif

					//=================================== FINAL UPDATE =================================================
					const float weightKilling = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightKillingTerm;
					const float weightLevelSet = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightLevelSetTerm;
					const float learningRate = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::gradientDescentLearningRate;
					Vector3f deltaE = deltaEData + weightLevelSet * deltaELevelSet + weightKilling * deltaEKilling;

					Vector3f warpUpdate = learningRate * deltaE;
					float warpUpdateLength = length(warpUpdate);//meters
					//BEGIN _DEBUG
					float warpLength = ORUtils::length(canonicalVoxel.warp_t - warpUpdate);
					float distanceFromTwoIterationsAgo = ORUtils::length(canonicalVoxel.warp_t_update + warpUpdate);
					float distanceTraveledInTwoIterations = ORUtils::length(canonicalVoxel.warp_t_update - warpUpdate);
					//TODO: this is a bad way to do convergence. Use something like Adam instead, maybe? --Greg
					//TODO: figure out the exact conditions causing these oscillations, maybe nothing fancy is necessary here --Greg(GitHub: Algomorph)
					const float maxVectorUpdateThresholdVoxels = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::maxVectorUpdateThresholdVoxels;
					if (distanceFromTwoIterationsAgo < maxVectorUpdateThresholdVoxels &&
					    distanceTraveledInTwoIterations >= maxVectorUpdateThresholdVoxels * 2) {
						//We think that an oscillation has been detected
#ifdef PRINT_ADDITIONAL_STATS
						voxelOscillationCount++;
#ifndef WITH_OPENMP
						voxelOscillations.push_back(std::tuple<int, int>(hash, locId));
#endif
#endif
#ifdef LOG_HIGHLIGHTS
						int& currentFrame = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
						const int& frameOfInterest = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::frameOfInterest;
						int& currentIteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
						if(currentFrame == frameOfInterest){
							ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::sceneLogger
									.LogHighlight(hash,locId,currentFrame,currentIteration);
						}
#endif
#ifdef OLD_UGLY_WAY
						warpUpdate *= 0.5;//magic! -UNDO THE MAGIC FOR DEBUGGING OSCILLATIONS FURTHER
#else //new super-awesome way (that still doesn't address the cause, but, oh well)

						//TODO: do we need the iteration check? Verify -Greg (GitHub: Algomorph)
						//TODO: magic value should be fine-tuned and set as constant or parameter -Greg (GitHub: Algomorph)
						if( ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration > 15) {
							if (!(canonicalVoxel.flags & VOXEL_OSCILLATION_DETECTED_ONCE)) {
								canonicalVoxel.flags |= VOXEL_OSCILLATION_DETECTED_ONCE;
							} else {
								canonicalVoxel.flags |= VOXEL_OSCILLATION_DETECTED_TWICE;
								continue;
							}
						}
#endif
					}
					//END _DEBUG

					//need thread lock here to ensure atomic updates to maxWarpUpdateLength
#ifdef WITH_OPENMP
#pragma omp critical(maxVectorUpdate)
#endif
					{
						if (maxWarpUpdateLength < warpUpdateLength) {
							maxWarpUpdateLength = warpUpdateLength;
						}
						if (maxWarpLength < warpLength) {
							maxWarpLength = warpLength;
						}
					};

					canonicalVoxel.warp_t_update = warpUpdate;
#ifdef PRINT_SINGLE_VOXEL_RESULT
					if (printResult) {
						std::cout << "Data update: " << deltaEData;
						std::cout << " Level set update: " << deltaELevelSet;
						std::cout << " Killing update: " << deltaEKilling;
						std::cout << std::endl;
						std::cout << "Warp update: " << warpUpdate;
						std::cout << " Warp update length: " << warpUpdateLength << std::endl << std::endl;
					}
#endif

					//debug stats
#ifdef PRINT_ADDITIONAL_STATS
					aveCanonicaSdf += canonicalSdf;
					aveLiveSdf += liveSdf;
					aveSdfDiff += diffSdf;
					aveWarpDist += ORUtils::length(canonicalVoxel.warp_t);
					if (boundary) {
						aveWarpDistBoundary += ORUtils::length(canonicalVoxel.warp_t);
					}
					consideredVoxelCount += 1;

#endif
#ifdef PRINT_ENERGY_STATS
					totalDataEnergy += 0.5 * (diffSdf * diffSdf);
					totalLevelSetEnergy +=
							weightLevelSet * 0.5 * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
					totalKillingEnergy += weightKilling * localKillingEnergy;
					totalSmoothnessEnergy += weightKilling * localSmoothnessEnergy;
#endif

				}
			}
		}
	}
#ifdef PRINT_TIME_STATS
	TOC(timeWarpUpdateCompute);
#endif

	//Apply the update
#ifdef PRINT_DEBUG_HISTOGRAM
	//_DEBUG
	//Warp Update Length Histogram
	// <20%, 40%, 60%, 80%, 100%
	const int histBinCount = 10;
	int warpBins[histBinCount] = {0};
	int updateBins[histBinCount] = {0};
#endif
	TIC(timeWarpUpdateApply);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);


		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];

					canonicalVoxel.warp_t -= canonicalVoxel.warp_t_update;
#if defined(PRINT_DEBUG_HISTOGRAM) || defined(PRINT_MAX_WARP_AND_UPDATE)
					float warpLength = ORUtils::length(canonicalVoxel.warp_t);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.warp_t_update);
#endif
#ifdef PRINT_MAX_WARP_AND_UPDATE
					if (maxWarpLength > 0.0f && warpLength == maxWarpLength) {
						std::cout << " Max warp pos: "
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}

					if (maxWarpUpdateLength > 0.0f && warpUpdateLength == maxWarpUpdateLength) {
						std::cout << " Max update pos: "
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
#endif //PRINT_MAX_WARP_AND_UPDATE
#ifdef PRINT_DEBUG_HISTOGRAM
					int binIdx = 0;
					if (maxWarpLength > 0) {
						binIdx = std::min(histBinCount - 1, (int) (warpLength * histBinCount / maxWarpLength));
					}
					warpBins[binIdx]++;
					if (maxWarpUpdateLength > 0) {
						binIdx = std::min(histBinCount - 1,
						                  (int) (warpUpdateLength * histBinCount / maxWarpUpdateLength));
					}
					updateBins[binIdx]++;

#endif //PRINT_DEBUG_HISTOGRAM

				}
			}
		}
	}
#ifdef PRINT_TIME_STATS
	TOC(timeWarpUpdateApply);
#endif

#ifdef PRINT_ENERGY_STATS
	totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalKillingEnergy;

	std::cout << " [ENERGY] Data term: " << totalDataEnergy
	          << " Level set term: " << totalLevelSetEnergy << cyan
	          << " Smoothness term: " << totalSmoothnessEnergy << yellow
	          << " Killing term: " << totalKillingEnergy << green
	          << " Total: " << totalEnergy << reset;
//	          << " No Killing: " << totalDataEnergy + totalLevelSetEnergy << reset
//	          << " No Level Set: " << totalDataEnergy + totalKillingEnergy;
	std::cout << std::endl;
#ifdef WRITE_ENERGY_STATS_TO_FILE
	std::ofstream& energy_stat_file = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::energy_stat_file;
	energy_stat_file << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalSmoothnessEnergy << ", "
	                 << totalKillingEnergy << ", " << totalEnergy << std::endl;
#endif
#endif
#ifdef PRINT_ADDITIONAL_STATS
	//_DEBUG
	aveCanonicaSdf /= consideredVoxelCount;
	aveLiveSdf /= consideredVoxelCount;
	aveWarpDist /= consideredVoxelCount;
	aveSdfDiff /= consideredVoxelCount;
	if (boundaryVoxelCount > 0) {
		aveWarpDistBoundary /= boundaryVoxelCount;
	}
	std::cout //<< " Ave canonical SDF: " << aveCanonicaSdf
			//<< " Ave live SDF: " << aveLiveSdf
			//<< " Ave SDF diff: " << aveSdfDiff
			<< " Used voxel count: " << consideredVoxelCount
			//<< " Ave warp distance: " << aveWarpDist
			<< " Oscillation ct: " << voxelOscillationCount
			<< " I-oscillation ct: " << ignoredVoxelOscillationCount;
#ifndef WITH_OPENMP
	std::cout << " Oscillation locations: {";

	for (auto entry : voxelOscillations) {
		std::cout << std::get<0>(entry) << "-" << std::get<1>(entry) << ", ";
	}
	std::cout << "}" << std::endl;
#endif

//	if (boundaryVoxelCount > 0) {
//		std::cout << " Boundary voxel count: " << boundaryVoxelCount
//		          << " Boundary ave w. dist.: " << aveWarpDistBoundary;
//	}

	std::cout << std::endl;
#endif
	//start _DEBUG

#ifdef PRINT_TIME_STATS
	std::cout << " Update compute time: " << timeWarpUpdateCompute
			  << " Update apply time: " << timeWarpUpdateApply
			  << " Data J&H time: " << timeDataJandHCompute
			  << " Warp J&H time: " << timeWarpJandHCompute
			  << " Update term time: " << timeUpdateTermCompute;
#endif
#ifdef PRINT_DEBUG_HISTOGRAM
	std::cout << "Warp length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << warpBins[iBin] << "  ";
	}
	std::cout << std::endl;
	std::cout << "Warp length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << updateBins[iBin] << "  ";
	}
	std::cout << std::endl;
#endif
#ifdef PRINT_MAX_WARP_AND_UPDATE
	std::cout << "Max warp: " << maxWarpLength << " Max update: " << maxWarpUpdateLength << std::endl;
#endif
	//end _DEBUG
	return maxWarpUpdateLength;

}

