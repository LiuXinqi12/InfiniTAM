//  ================================================================
//  Created by Gregory Kramida on 2/20/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include "../../Utils/Analytics/ITMNeighborVoxelIterationInfo.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Objects/Scene/ITMTrilinearInterpolation.h"

using namespace ITMLib;

//======================================================================================================================
//=========================================== DEBUG ROUTINES FOR SAVING INFORMATION DURING OPTIMIZATION ================
//======================================================================================================================
//DEBUG
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel
ReadVoxelAndLinearIndex(const CONSTPTR(TVoxel)* voxelData,
                        const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex,
                        const THREADPTR(Vector3i)& point, THREADPTR(int)& vmIndex,
                        THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache)& cache, THREADPTR(int)& linearIdx) {
	Vector3i blockPos;
	linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos) {
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = hashIndex(blockPos);

	while (true) {
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
			cache.blockPos = blockPos;
			cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound
			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	vmIndex = false;
	linearIdx = 0;
	return TVoxel();
}

inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(const ITMLib::ITMPlainVoxelArray::IndexData* data, Vector3i position) {
	return position.z * (data->size.z * data->size.y)
	       + position.y * (data->size.x + position.x);
};

//DEBUG
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel
ReadVoxelAndLinearIndex(const CONSTPTR(TVoxel)* voxelData,
                        const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData)* voxelIndex,
                        const THREADPTR(Vector3i)& point, THREADPTR(int)& vmIndex,
                        THREADPTR(ITMLib::ITMPlainVoxelArray::IndexCache)& cache, THREADPTR(int)& linearIdx) {

	linearIdx = ComputeLinearIndexFromPosition_PlainVoxelArray(voxelIndex, point);
	return voxelData[linearIdx];
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndexData, typename TCache>
inline void FindHighlightNeighborInfo(std::array<ITMLib::ITMNeighborVoxelIterationInfo, 9>& neighbors,
                                      const CONSTPTR(Vector3i)& highlightPosition,
                                      const CONSTPTR(int)& highlightHash,
                                      const CONSTPTR(TVoxelCanonical)* canonicalVoxelData,
                                      const CONSTPTR(TIndexData)* canonicalIndexData,
                                      THREADPTR(TCache)& canonicalCache,
                                      const CONSTPTR(TVoxelLive)* liveVoxelData,
                                      const CONSTPTR(TIndexData)* liveIndexData,
                                      THREADPTR(TCache)& liveCache
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i locations[9] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1),
	                         Vector3i(1, 0, 0), Vector3i(0, 1, 0), Vector3i(0, 0, 1),
	                         Vector3i(1, 1, 0), Vector3i(0, 1, 1), Vector3i(1, 0, 1)};
	int vmIndex, localId = 0;
	vmIndex = highlightHash + 1;
	int iNeighbor = 0;
	for (auto location : locations) {
		ITMLib::ITMNeighborVoxelIterationInfo& info = neighbors[iNeighbor];
		Vector3i neighborPosition = highlightPosition + (location);
		TVoxelCanonical voxelCanonical = ReadVoxelAndLinearIndex(canonicalVoxelData, canonicalIndexData,
		                                                         neighborPosition,
		                                                         vmIndex, canonicalCache, localId);
		const TVoxelLive& voxelLive = readVoxel(liveVoxelData, liveIndexData, neighborPosition, vmIndex, liveCache);
		if (vmIndex != 0) {
			info.unknown = voxelCanonical.flags == ITMLib::VOXEL_TRUNCATED;
			info.hash = vmIndex - 1;
		} else {
			info.notAllocated = true;
			info.hash = 0;
			vmIndex = highlightHash + 1;//reset
		}
		info.localId = localId;
		info.warp = voxelCanonical.flow_warp;
		info.warpGradient = voxelCanonical.gradient0;
		info.sdf = TVoxelCanonical::valueToFloat(voxelCanonical.sdf);
		info.liveSdf = TVoxelLive::valueToFloat(voxelLive.sdf);
		iNeighbor++;
	}
}

//======================================================================================================================
//====================================== DEBUG PRINTING ROUTINES =======================================================
//======================================================================================================================

//_DEBUG printing routine
inline void _DEBUG_PrintDataTermStuff(const CONSTPTR(Vector3f)& liveSdfJacobian) {
	const std::string cyan("\033[0;36m");
	const std::string reset("\033[0m");
	std::cout << std::endl;
	std::cout << "Jacobian of live SDF at current warp: " << cyan << liveSdfJacobian << reset << std::endl;
}

//_DEBUG printing routine
inline void _DEBUG_PrintLevelSetTermStuff(const CONSTPTR(Vector3f)& liveSdfJacobian,
                                          const CONSTPTR(Vector3f)& liveSdf_Center_WarpForward,
                                          const CONSTPTR(Vector3f)& warpedSdfJacobian,
                                          const CONSTPTR(Matrix3f)& warpedSdfHessian) {
	std::cout << std::endl;
	std::cout << "Warped SDF Jacobian [Difference from neighbor's lookup values from live SDF]: " << green
	          << warpedSdfJacobian << reset << std::endl;
	std::cout << "Change in warped SDF Jacobian when warp changes (by one): " << std::endl
	          << green << warpedSdfHessian << reset << std::endl;
}


//_DEBUG printing routine
inline void _DEBUG_PrintKillingTermStuff(const CONSTPTR(Vector3f*) neighborWarps,
                                         const CONSTPTR(bool*) neighborAllocated,
                                         const CONSTPTR(bool*) neighborTruncated,
                                         THREADPTR(Matrix3f)& jacobian, //in
                                         THREADPTR(Matrix3f)* hessian //in, x3
) {

	const int neighborhoodSize = 9;
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i neighborPositions[] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1), Vector3i(1, 0, 0),
	                                Vector3i(0, 1, 0), Vector3i(0, 0, 1), Vector3i(1, 1, 0), Vector3i(0, 1, 1),
	                                Vector3i(1, 0, 1),};

	std::cout << green;
	std::cout << "Neighbors' warps: " << std::endl;
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		std::cout << reset << neighborPositions[iNeightbor] << " (Neighbor " << iNeightbor << ")" << ": " << green
		          << neighborWarps[iNeightbor] << ", " << std::endl;
	}

	std::cout << std::endl << reset << "Unallocated neighbors: ";
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		if (!neighborAllocated[iNeightbor]) {
			std::cout << iNeightbor << ", ";
		}
	}
	std::cout << std::endl;
	std::cout << "Truncated neighbors: ";
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		if (neighborTruncated[iNeightbor]) {
			std::cout << iNeightbor << ", ";
		}
	}
	std::cout << std::endl;
	std::cout << std::endl << yellow;
	std::cout << "Jacobian: " << std::endl << jacobian << std::endl << cyan;
	std::cout << "Hessian: " << std::endl << hessian[0] << hessian[1] << hessian[2] << reset << std::endl;
};


//_DEBUG printing routine
inline void _DEBUG_PrintTikhonovTermStuff(const CONSTPTR(Vector3f*) neighborWarps,
                                          const CONSTPTR(Vector3f)& laplacian) {

	const int neighborhoodSize = 6;
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i neighborPositions[] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1), Vector3i(1, 0, 0),
	                                Vector3i(0, 1, 0), Vector3i(0, 0, 1)};

	std::cout << green;
	std::cout << "Neighbors' warps: " << std::endl;
	for (int iNeightbor = 0; iNeightbor < neighborhoodSize; iNeightbor++) {
		std::cout << reset << neighborPositions[iNeightbor] << " (Neighbor " << iNeightbor << ")" << ": " << green
		          << neighborWarps[iNeightbor] << ", " << std::endl;
	}
	std::cout << std::endl << yellow;
	std::cout << "Laplacian: " << std::endl << laplacian << reset << std::endl;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void find6ConnectedNeighborInfo(
		THREADPTR(bool)* neighborKnown, //x6, out
		THREADPTR(bool)* neighborTruncated, //x6, out
		THREADPTR(bool)* neighborAllocated, //x6, out
		THREADPTR(float)* neighborSdf, //x6, out
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(ITMHashEntry)* hashEntries,
		THREADPTR(TCache)& cache) {
	int vmIndex;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, hashEntries, voxelPosition + (location), vmIndex, cache);\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;\
    neighborSdf[index] = TVoxel::valueToFloat(voxel.sdf);

	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);
#undef PROCESS_VOXEL
}

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void find6ConnectedNeighborInfoIndexedFields(
		THREADPTR(bool)* neighborKnown, //x6, out
		THREADPTR(bool)* neighborTruncated, //x6, out
		THREADPTR(bool)* neighborAllocated, //x6, out
		THREADPTR(float)* neighborSdf, //x6, out
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* hashEntries,
		THREADPTR(TCache)& cache,
		int fieldIndex) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, hashEntries, voxelPosition + (location), vmIndex, cache);\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flag_values[fieldIndex] != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flag_values[fieldIndex] != ITMLib::VOXEL_NONTRUNCATED;\
    neighborSdf[index] = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);
#undef PROCESS_VOXEL
}


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void print6ConnectedNeighborInfo(
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache) {
	const int neighborhoodSize = 6;
	bool neighborKnown[neighborhoodSize];
	bool neighborTruncated[neighborhoodSize];
	bool neighborAllocated[neighborhoodSize];
	float neighborSdf[neighborhoodSize];

	find6ConnectedNeighborInfo(neighborKnown, neighborTruncated, neighborAllocated, neighborSdf, voxelPosition, voxels,
	                           indexData, cache);
	const Vector3i positions[6] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1),
	                               Vector3i(1, 0, 0), Vector3i(0, 1, 0), Vector3i(0, 0, 1)};

	for (int iNeighbor = 0; iNeighbor < 6; iNeighbor++) {
		std::cout << ITMLib::yellow << "[" << positions[iNeighbor] << "]" << ITMLib::reset
		          << " allocated: " << printBool(neighborAllocated[iNeighbor])
		          << " truncated: " << printBool(neighborTruncated[iNeighbor]) << " known: "
		          << printBool(neighborKnown[iNeighbor]) << " sdf: " << neighborSdf[iNeighbor] << std::endl;
	}
}


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void print6ConnectedNeighborInfoIndexedFields(
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache,
		const CONSTPTR(int)& fieldIndex) {

#define USE_SPATIAL_PRINTING

	const int neighborhoodSize = 6;
	bool neighborKnown[neighborhoodSize];
	bool neighborTruncated[neighborhoodSize];
	bool neighborAllocated[neighborhoodSize];
	float neighborSdf[neighborhoodSize];
	int vmIndex;
	TVoxel voxel = readVoxel(voxels, indexData, voxelPosition, vmIndex, cache);
	float currentSdf = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	bool currentKnown = voxel.flag_values[fieldIndex] != ITMLib::VOXEL_UNKNOWN;
	find6ConnectedNeighborInfoIndexedFields(neighborKnown, neighborTruncated, neighborAllocated, neighborSdf,
	                                        voxelPosition, voxels, indexData, cache, fieldIndex);

	const Vector3i positions[6] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1),
	                               Vector3i(1, 0, 0), Vector3i(0, 1, 0), Vector3i(0, 0, 1)};

	auto printKnown = [](bool arg) { return arg ? "known  " : "unknown"; };

#ifdef USE_SPATIAL_PRINTING
	//for now, use x-z for in-plane printing, y +/- for additional pair
	std::printf("                     %sz+1:%s%+.3f, %s\n", ITMLib::yellow.c_str(), ITMLib::reset.c_str(),
	            neighborSdf[2], printKnown(neighborKnown[2]));
	std::printf(" %1$sx-1%2$s:%3$+.3f,%4$s %1$scur%2$s: %5$+.3f, %6$s %1$sx+1%2$s: %7$+.3f,%8$s         "
	            "%1$sy-1%2$s: %9$+.3f,%10$s / %1$sy+1%2$s: %11$+.3f,%12$s\n",
	            ITMLib::yellow.c_str(), ITMLib::reset.c_str(),
	            neighborSdf[0], printKnown(neighborKnown[0]),
	            currentSdf, printKnown(currentKnown),
	            neighborSdf[3], printKnown(neighborKnown[3]),
	            neighborSdf[1], printKnown(neighborKnown[1]),
	            neighborSdf[4], printKnown(neighborKnown[4])

	);
	std::printf("                     %sz+1%s:%+.3f, %s\n", ITMLib::yellow.c_str(), ITMLib::reset.c_str(),
	            neighborSdf[5], printKnown(neighborKnown[5]));
#else
	for (int iNeighbor = 0; iNeighbor < 6; iNeighbor++) {
		std::cout << ITMLib::yellow << "[" << positions[iNeighbor] << "]" << ITMLib::reset
				  << " allocated: " << printBool(neighborAllocated[iNeighbor])
				  << " truncated: " << printBool(neighborTruncated[iNeighbor])
				  << " known: " << printBool(neighborKnown[iNeighbor])
				  << " sdf: " << neighborSdf[iNeighbor] << std::endl;
	}
#endif
}


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_NontruncatedOnly(Vector3f& jacobian,
                                                             const Vector3i& voxelPosition,
                                                             const TVoxel* voxels,
                                                             const ITMHashEntry* hashEntries,
                                                             TCache cache) {
	int vmIndex;
	TVoxel voxel;
	bool xValid = true, yValid = true, zValid = true;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtXplusOne = TVoxel::valueToFloat(voxel.sdf);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	yValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtYplusOne = TVoxel::valueToFloat(voxel.sdf);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	zValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtZplusOne = TVoxel::valueToFloat(voxel.sdf);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtXminusOne = TVoxel::valueToFloat(voxel.sdf);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	yValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtYminusOne = TVoxel::valueToFloat(voxel.sdf);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	zValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtZminusOne = TVoxel::valueToFloat(voxel.sdf);


	jacobian[0] = xValid ? 0.5f * (sdfAtXplusOne - sdfAtXminusOne) : 0.0f;
	jacobian[1] = yValid ? 0.5f * (sdfAtYplusOne - sdfAtYminusOne) : 0.0f;
	jacobian[2] = zValid ? 0.5f * (sdfAtZplusOne - sdfAtZminusOne) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_ForwardDifferences_NontruncatedOnly_IndexedFields(Vector3f& jacobian,
                                                                           const Vector3i& voxelPosition,
                                                                           const TVoxel* voxels,
                                                                           const ITMHashEntry* hashEntries,
                                                                           THREADPTR(TCache) cache,
                                                                           int fieldIndex) {
	int vmIndex;
	bool xValid = true, yValid = true, zValid = true;
	TVoxel voxel;
	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtXplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	yValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtYplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	zValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtZplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition, vmIndex, cache);
	float sdfAtPosition = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);


	jacobian[0] = xValid ? (sdfAtXplusOne - sdfAtPosition) : 0.0f;
	jacobian[1] = yValid ? (sdfAtYplusOne - sdfAtPosition) : 0.0f;
	jacobian[2] = zValid ? (sdfAtZplusOne - sdfAtPosition) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_NontruncatedOnly_IndexedFields(Vector3f& jacobian,
                                                                           const Vector3i& voxelPosition,
                                                                           const TVoxel* voxels,
                                                                           const ITMHashEntry* hashEntries,
                                                                           THREADPTR(TCache) cache,
                                                                           int fieldIndex) {
	int vmIndex;
	TVoxel voxel;
	bool xValid = true, yValid = true, zValid = true;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtXplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	yValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtYplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	zValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtZplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtXminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	yValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtYminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	zValid &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	float sdfAtZminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);


	jacobian[0] = xValid ? 0.5f * (sdfAtXplusOne - sdfAtXminusOne) : 0.0f;
	jacobian[1] = yValid ? 0.5f * (sdfAtYplusOne - sdfAtYminusOne) : 0.0f;
	jacobian[2] = zValid ? 0.5f * (sdfAtZplusOne - sdfAtZminusOne) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_SmallDifferences_IndexedFields(Vector3f& jacobian,
                                                                           const Vector3i& voxelPosition,
                                                                           const TVoxel* voxels,
                                                                           const ITMHashEntry* hashEntries,
                                                                           THREADPTR(TCache) cache,
                                                                           int fieldIndex) {
	int vmIndex;
	TVoxel voxel;


	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	float sdfAtXplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	float sdfAtYplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	float sdfAtZplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	float sdfAtXminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	float sdfAtYminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	float sdfAtZminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	float xDiff = sdfAtXplusOne - sdfAtXminusOne;
	float yDiff = sdfAtYplusOne - sdfAtYminusOne;
	float zDiff = sdfAtZplusOne - sdfAtZminusOne;
	const float threshold = 1.0;

	jacobian[0] = std::abs(xDiff) < threshold ? 0.5f * xDiff : 0.0f;
	jacobian[1] = std::abs(yDiff) < threshold ? 0.5f * yDiff : 0.0f;
	jacobian[2] = std::abs(zDiff) < threshold ? 0.5f * zDiff : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_IgnoreUnknown_IndexedFields(Vector3f& jacobian,
                                                                        const Vector3i& voxelPosition,
                                                                        const TVoxel* voxels,
                                                                        const ITMHashEntry* hashEntries,
                                                                        THREADPTR(TCache) cache,
                                                                        int fieldIndex) {
	int vmIndex;
	TVoxel voxel;
	bool xValid = true, yValid = true, zValid = true;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtXplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	yValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtYplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	zValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtZplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtXminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	yValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtYminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	zValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtZminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);


	jacobian[0] = xValid ? 0.5f * (sdfAtXplusOne - sdfAtXminusOne) : 0.0f;
	jacobian[1] = yValid ? 0.5f * (sdfAtYplusOne - sdfAtYminusOne) : 0.0f;
	jacobian[2] = zValid ? 0.5f * (sdfAtZplusOne - sdfAtZminusOne) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_IgnoreUnknown_IndexedFields_BorderTreatment(Vector3f& jacobian,
                                                                                        const Vector3i& voxelPosition,
                                                                                        const TVoxel* voxels,
                                                                                        const ITMHashEntry* hashEntries,
                                                                                        THREADPTR(TCache) cache,
                                                                                        float liveSdf,
                                                                                        int fieldIndex) {
	int vmIndex;
	TVoxel voxel;
	bool xValid = true, yValid = true, zValid = true;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	//TODO
	DIEWITHEXCEPTION_REPORTLOCATION("NOT YET IMPLEMENTED");
//	float sdfAtXplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
//
//	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
//	yValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
//	float sdfAtYplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
//
//	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
//	zValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
//	float sdfAtZplusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
//
//	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
//	xValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
//	float sdfAtXminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
//
//	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
//	yValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
//	float sdfAtYminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
//
//	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
//	zValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
//	float sdfAtZminusOne = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
//
//
//	jacobian[0] = xValid ? 0.5f * (sdfAtXplusOne - sdfAtXminusOne) : 0.0f;
//	jacobian[1] = yValid ? 0.5f * (sdfAtYplusOne - sdfAtYminusOne) : 0.0f;
//	jacobian[2] = zValid ? 0.5f * (sdfAtZplusOne - sdfAtZminusOne) : 0.0f;
};

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_IndexedFields_TruncationSignInference(Vector3f& jacobian,
                                                                                  const Vector3i& voxelPosition,
                                                                                  const TVoxel* voxels,
                                                                                  const ITMHashEntry* hashEntries,
                                                                                  THREADPTR(TCache) cache,
                                                                                  float value,
                                                                                  int fieldIndex) {
	int vmIndex;
	TVoxel voxel;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	float sdfAtXplusOne =
			voxel.flags == ITMLib::VOXEL_NONTRUNCATED ? TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]) : value;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	float sdfAtYplusOne =
			voxel.flags == ITMLib::VOXEL_NONTRUNCATED ? TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]) : value;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	float sdfAtZplusOne =
			voxel.flags == ITMLib::VOXEL_NONTRUNCATED ? TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]) : value;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	float sdfAtXminusOne =
			voxel.flags == ITMLib::VOXEL_NONTRUNCATED ? TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]) : value;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	float sdfAtYminusOne =
			voxel.flags == ITMLib::VOXEL_NONTRUNCATED ? TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]) : value;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	float sdfAtZminusOne =
			voxel.flags == ITMLib::VOXEL_NONTRUNCATED ? TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]) : value;


	jacobian[0] = sdfAtXplusOne - sdfAtXminusOne;
	jacobian[1] = sdfAtYplusOne - sdfAtYminusOne;
	jacobian[2] = sdfAtZplusOne - sdfAtZminusOne;
};

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_IgnoreUnknown_IndexedFields_TruncationSignInference(Vector3f& jacobian,
                                                                                                const Vector3i& voxelPosition,
                                                                                                const TVoxel* voxels,
                                                                                                const ITMHashEntry* hashEntries,
                                                                                                THREADPTR(TCache) cache,
                                                                                                float value,
                                                                                                int fieldIndex) {
	int vmIndex;
	TVoxel voxel;
	bool xValid = true, yValid = true, zValid = true;

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtXplusOne =
			voxel.flags == ITMLib::VOXEL_TRUNCATED ? value : TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	yValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtYplusOne =
			voxel.flags == ITMLib::VOXEL_TRUNCATED ? value : TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	zValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtZplusOne =
			voxel.flags == ITMLib::VOXEL_TRUNCATED ? value : TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	xValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtXminusOne =
			voxel.flags == ITMLib::VOXEL_TRUNCATED ? value : TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	yValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtYminusOne =
			voxel.flags == ITMLib::VOXEL_TRUNCATED ? value : TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	zValid &= voxel.flags != ITMLib::VOXEL_UNKNOWN;
	float sdfAtZminusOne =
			voxel.flags == ITMLib::VOXEL_TRUNCATED ? value : TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);


	jacobian[0] = xValid ? 0.5f * (sdfAtXplusOne - sdfAtXminusOne) : 0.0f;
	jacobian[1] = yValid ? 0.5f * (sdfAtYplusOne - sdfAtYminusOne) : 0.0f;
	jacobian[2] = zValid ? 0.5f * (sdfAtZplusOne - sdfAtZminusOne) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_AllocatedOnly(Vector3f& jacobian,
                                                          const Vector3i& voxelPosition,
                                                          const TVoxel* voxels,
                                                          const ITMHashEntry* hashEntries,
                                                          TCache cache) {
	int vmIndex;
	TVoxel voxel;
	bool xValid = true, yValid = true, zValid = true;
#define sdf_at(offset) (TVoxel::valueToFloat(readVoxel(voxels, hashEntries, voxelPosition + (offset), vmIndex, cache).sdf))
	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(1, 0, 0), vmIndex, cache);
	xValid &= vmIndex;
	float sdfAtXplusOne = sdf_at(Vector3i(1, 0, 0));

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 1, 0), vmIndex, cache);
	yValid &= vmIndex;
	float sdfAtYplusOne = sdf_at(Vector3i(0, 1, 0));

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, 1), vmIndex, cache);
	zValid &= vmIndex;
	float sdfAtZplusOne = sdf_at(Vector3i(0, 0, 1));

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(-1, 0, 0), vmIndex, cache);
	xValid &= vmIndex;
	float sdfAtXminusOne = sdf_at(Vector3i(-1, 0, 0));

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, -1, 0), vmIndex, cache);
	yValid &= vmIndex;
	float sdfAtYminusOne = sdf_at(Vector3i(0, -1, 0));

	voxel = readVoxel(voxels, hashEntries, voxelPosition + Vector3i(0, 0, -1), vmIndex, cache);
	zValid &= vmIndex;
	float sdfAtZminusOne = sdf_at(Vector3i(0, 0, -1));

#undef sdf_at
	jacobian[0] = xValid ? 0.5f * (sdfAtXplusOne - sdfAtXminusOne) : 0.0f;
	jacobian[1] = yValid ? 0.5f * (sdfAtYplusOne - sdfAtYminusOne) : 0.0f;
	jacobian[2] = zValid ? 0.5f * (sdfAtZplusOne - sdfAtZminusOne) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_LiveSdf(Vector3f& jacobian,
                                                                      const Vector3i& voxelPosition,
                                                                      const TVoxel* voxels,
                                                                      const ITMHashEntry* hashEntries,
                                                                      THREADPTR(TCache) cache,
                                                                      int fieldIndex) {


	const Vector3i offsets[] = {Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	                            Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	                            Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

	int vmIndex;
	TVoxel voxel;
	voxel = readVoxel(voxels, hashEntries, voxelPosition, vmIndex, cache);
	float sdfAtPosition = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);

	float sdfs[6];
	bool valid[3] = {true};

	for (int iNeighbor = 0; iNeighbor < sizeof(offsets) / sizeof(Vector3i); iNeighbor++) {
		voxel = readVoxel(voxels, hashEntries, voxelPosition + offsets[iNeighbor], vmIndex, cache);
		switch ((VoxelFlags) voxel.flags) {
			case VOXEL_UNKNOWN:
				valid[iNeighbor / 2] = false;
				break;
			case VOXEL_TRUNCATED:
				sdfs[iNeighbor] = std::copysign(1.0f, sdfAtPosition);
				break;
			default:
				sdfs[iNeighbor] = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
				break;
		}
	}

	jacobian[0] = valid[0] ? 0.5f * (sdfs[0] - sdfs[1]) : 0.0f;
	jacobian[1] = valid[1] ? 0.5f * (sdfs[2] - sdfs[3]) : 0.0f;
	jacobian[2] = valid[2] ? 0.5f * (sdfs[4] - sdfs[5]) : 0.0f;
};


template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_CanonicalSdf(Vector3f& jacobian,
                                                                           const Vector3i& voxelPosition,
                                                                           const TVoxel* voxels,
                                                                           const ITMHashEntry* hashEntries,
                                                                           THREADPTR(TCache) cache,
                                                                           int fieldIndex,
                                                                           const float canonicalSdf) {


	const Vector3i offsets[] = {Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	                            Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	                            Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

	int vmIndex;
	TVoxel voxel;

	float sdfs[6];
	bool valid[3] = {true};

	for (int iNeighbor = 0; iNeighbor < sizeof(offsets) / sizeof(Vector3i); iNeighbor++) {
		voxel = readVoxel(voxels, hashEntries, voxelPosition + offsets[iNeighbor], vmIndex, cache);
		switch ((VoxelFlags) voxel.flags) {
			case VOXEL_UNKNOWN:
				valid[iNeighbor / 2] = false;
				break;
			case VOXEL_TRUNCATED:
				sdfs[iNeighbor] = std::copysign(1.0f, canonicalSdf);
				break;
			default:
				sdfs[iNeighbor] = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
				break;
		}
	}

	jacobian[0] = valid[0] ? 0.5f * (sdfs[0] - sdfs[1]) : 0.0f;
	jacobian[1] = valid[1] ? 0.5f * (sdfs[2] - sdfs[3]) : 0.0f;
	jacobian[2] = valid[2] ? 0.5f * (sdfs[4] - sdfs[5]) : 0.0f;
};

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_ChangeTruncatedsSignToCanonicals(Vector3f& jacobian,
                                                                             const Vector3i& voxelPosition,
                                                                             const TVoxel* voxels,
                                                                             const ITMHashEntry* hashEntries,
                                                                             THREADPTR(TCache) cache,
                                                                             int fieldIndex,
                                                                             const float canonicalSdf) {


	const Vector3i offsets[] = {Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	                            Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	                            Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

	int vmIndex;
	TVoxel voxel;

	float sdfs[6];
	bool valid[3] = {true};

	for (int iNeighbor = 0; iNeighbor < sizeof(offsets) / sizeof(Vector3i); iNeighbor++) {
		voxel = readVoxel(voxels, hashEntries, voxelPosition + offsets[iNeighbor], vmIndex, cache);
		switch ((VoxelFlags) voxel.flags) {
			case VOXEL_UNKNOWN:
			case VOXEL_TRUNCATED:
				sdfs[iNeighbor] = std::copysign(1.0f, canonicalSdf);
				break;
			default:
				sdfs[iNeighbor] = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
				break;
		}
	}

	jacobian[0] = valid[0] ? 0.5f * (sdfs[0] - sdfs[1]) : 0.0f;
	jacobian[1] = valid[1] ? 0.5f * (sdfs[2] - sdfs[3]) : 0.0f;
	jacobian[2] = valid[2] ? 0.5f * (sdfs[4] - sdfs[5]) : 0.0f;
};

template<typename TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
void ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_CanonicalSdf2(Vector3f& jacobian,
                                                                            const Vector3i& voxelPosition,
                                                                            const TVoxel* voxels,
                                                                            const ITMHashEntry* hashEntries,
                                                                            THREADPTR(TCache) cache,
                                                                            int fieldIndex,
                                                                            const float canonicalSdf) {


	const Vector3i offsets[] = {Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	                            Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	                            Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

	int vmIndex;
	TVoxel voxel;

	float sdfs[6];

	voxel = readVoxel(voxels, hashEntries, voxelPosition, vmIndex, cache);
	float liveSdf = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	float diff = canonicalSdf - liveSdf;
	float absDiff = std::abs(diff);

	for (int iNeighbor = 0; iNeighbor < sizeof(offsets) / sizeof(Vector3i); iNeighbor++) {
		voxel = readVoxel(voxels, hashEntries, voxelPosition + offsets[iNeighbor], vmIndex, cache);
		sdfs[iNeighbor] = TVoxel::valueToFloat(voxel.sdf_values[fieldIndex]);
	}

	jacobian[0] = 0.5f * (sdfs[0] - sdfs[1]);
	jacobian[1] = 0.5f * (sdfs[2] - sdfs[3]);
	jacobian[2] = 0.5f * (sdfs[4] - sdfs[5]);


	jacobian[0] = std::abs(canonicalSdf - sdfs[std::signbit(jacobian[0]) != std::signbit(diff)]) < absDiff ?
	              jacobian[0] : 0.0f;
	jacobian[1] = std::abs(canonicalSdf - sdfs[2 + (std::signbit(jacobian[1]) != std::signbit(diff))]) < absDiff ?
	              jacobian[1] : 0.0f;
	jacobian[2] = std::abs(canonicalSdf - sdfs[4 + (std::signbit(jacobian[2]) != std::signbit(diff))]) < absDiff ?
	              jacobian[2] : 0.0f;
};