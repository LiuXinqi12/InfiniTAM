//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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
//stdlib
#include <limits>
#include <iomanip>
#include <memory>


//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>

//local
#include "ITMSceneMotionTracker.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Utils/ITMSceneSliceRasterizer.h"

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace ITMLib;

//region =========================================== CONSTRUCTORS / DESTRUCTORS ========================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker(const ITMSceneParams& params,
                                                                                  std::string scenePath)
		:
		maxVectorUpdateThresholdVoxels(maxVectorUpdateThresholdMeters / params.voxelSize),
		sceneLogger(nullptr),
		baseOutputDirectory(scenePath),
		hasFocusCoordinates(false) {}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker(const ITMSceneParams& params,
                                                                                  std::string scenePath,
                                                                                  Vector3i focusCoordinates)
		:
		maxVectorUpdateThresholdVoxels(maxVectorUpdateThresholdMeters / params.voxelSize),
		sceneLogger(nullptr),
		baseOutputDirectory(scenePath),
		hasFocusCoordinates(true), focusCoordinates(focusCoordinates) {}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker() {
	delete sceneLogger;
	delete targetLiveScene;
}

//endregion

// region =========================================== FUNCTIONS ========================================================
//_DEBUG (?, verbose param?)
inline static void PrintOperationStatus(const char* status) {
	std::cout << bright_cyan << status << reset << std::endl;
}

/**
 * \brief Tracks motion of voxels from canonical frame to live frame.
 * \details The warp field representing motion of voxels in the canonical frame is updated such that the live frame maps
 * as closely as possible back to the canonical using the warp.
 * \tparam TVoxelCanonical type of canonical voxels
 * \tparam TVoxelLive type of live voxels
 * \tparam TIndex type of voxel index used
 * \param canonicalScene the canonical voxel grid
 * \param sourceLiveScene the live voxel grid (typcially obtained by integrating a single depth image into an empty TSDF grid)
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::TrackMotion(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& sourceLiveScene,
		bool recordWarpUpdates, ITMSceneReconstructionEngine<TVoxelLive, TIndex>* liveSceneReconstructor) {


	//_DEBUG
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> canonicalCalculator;
	//_DEBUG
	ITMSceneStatisticsCalculator<TVoxelLive, TIndex> liveCalculator;

	auto PrintLiveSceneStats = [&liveCalculator](ITMScene<TVoxelLive, TIndex>* scene, const char* desc) {
		std::cout << std::setprecision(10) << "   Count of allocated blocks in " << desc << ": "
		          << liveCalculator.ComputeAllocatedHashBlockCount(scene) << std::endl;
		std::cout << "   Sum of non-truncated SDF magnitudes in " << desc << ": "
		          << liveCalculator.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
		std::cout << "   Count of non-truncated voxels in " << desc << ": "
		          << liveCalculator.ComputeNonTruncatedVoxelCount(scene) << std::endl;
	};
	PrintLiveSceneStats(sourceLiveScene, "raw live scene");

	PrintOperationStatus("Allocating canonical blocks based on live frame...");
	AllocateNewCanonicalHashBlocks(canonicalScene, sourceLiveScene);
	std::cout << "   Number of allocated blocks in canonical scene after allocation: "
	          << canonicalCalculator.ComputeAllocatedHashBlockCount(canonicalScene) << std::endl;

	liveSceneReconstructor->ResetScene(targetLiveScene);
	PrintOperationStatus(
			"Initializing live frame by mapping the raw live scene to a blank scene using canonical voxel warp field...");
	ApplyWarpFieldToLive(canonicalScene, sourceLiveScene, targetLiveScene);

	PrintLiveSceneStats(targetLiveScene, "working/target live scene after frame-initialization");

	SwapSourceAndTargetLiveScenes(sourceLiveScene);
	// region ================================== DEBUG 2D VISUALIZATION ================================================

	if (rasterizeCanonical) {
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderCanonicalSceneSlices(
				canonicalScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_X, "_X");
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderCanonicalSceneSlices(
				canonicalScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Y, "_Y");
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderCanonicalSceneSlices(
				canonicalScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Z, "_Z");
	}
	if (rasterizeLive) {
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderLiveSceneSlices(
				sourceLiveScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_X, "_X");
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderLiveSceneSlices(
				sourceLiveScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Y, "_Y");
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::RenderLiveSceneSlices(
				sourceLiveScene, ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::AXIS_Z, "_Z");
	}
	cv::Mat blank;
	cv::Mat liveImgTemplate;
	if (rasterizeUpdates) {
		std::cout << "Desired warp update (voxels) below " << maxVectorUpdateThresholdVoxels << std::endl;
		cv::Mat canonicalImg =
				ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawCanonicalSceneImageAroundPoint(
						canonicalScene) * 255.0f;
		cv::Mat canonicalImgOut;
		canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
		cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
		cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder
		            + "canonical.png", canonicalImgOut);
		cv::Mat liveImg =
				ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawLiveSceneImageAroundPoint(
						sourceLiveScene) * 255.0f;
		cv::Mat liveImgOut;
		liveImg.convertTo(liveImgTemplate, CV_8UC1);
		cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
		cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "live.png",
		            liveImgOut);
		blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
	}
	// endregion

	std::string currentFrameOutputPath = GenerateCurrentFrameOutputPath();
	const std::string energyStatFilePath = currentFrameOutputPath + "/energy.txt";
	energy_stat_file = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energy_stat_file << "data" << "," << "level_set" << "," << "smoothness" << ","
	                 << "killing" << "," << "total" << std::endl;
	// region ================================== WARP UPDATE RECORDING (BEFORE OPTIMIZATION) ===========================

	if (recordWarpUpdates) {
		sceneLogger = new ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>(canonicalScene, sourceLiveScene,
		                                                                      currentFrameOutputPath);
		sceneLogger->SaveScenesCompact();
		sceneLogger->StartSavingWarpState(currentFrameIx);
		if (hasFocusCoordinates) {
			sceneLogger->ClearHighlights();
			//TODO: setup highlight saving w/o interest region saving
		}
	}
	// endregion

	PrintOperationStatus("*** Optimizing warp based on difference between canonical and live SDF. ***");
	float maxVectorUpdate = std::numeric_limits<float>::infinity();
	for (iteration = 0; maxVectorUpdate > maxVectorUpdateThresholdVoxels && iteration < maxIterationCount;
	     iteration++) {
		// region ================================== DEBUG 2D VISUALIZATION FOR UPDATES ================================

		//TODO: move draw image routines into separpate memeber function
		if (rasterizeUpdates) {
			cv::Mat warpImg =
					ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpedSceneImageAroundPoint(
							canonicalScene) * 255.0f;
			cv::Mat warpImgChannel, warpImgOut, mask, liveImgChannel, markChannel;
			blank.copyTo(markChannel);
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
					canonicalScene, markChannel,
					ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos1);
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
					canonicalScene, markChannel,
					ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos2);
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
					canonicalScene, markChannel,
					ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos3);
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::MarkWarpedSceneImageAroundPoint(
					canonicalScene, markChannel,
					ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos4);
			liveImgChannel = cv::Mat::zeros(warpImg.rows, warpImg.cols, CV_8UC1);
			warpImg.convertTo(warpImgChannel, CV_8UC1);
			cv::threshold(warpImgChannel, mask, 1.0, 1.0, cv::THRESH_BINARY_INV);
			liveImgTemplate.copyTo(liveImgChannel, mask);

			cv::Mat channels[3] = {liveImgTemplate, warpImgChannel, markChannel};

			cv::merge(channels, 3, warpImgOut);
			std::stringstream numStringStream;
			numStringStream << std::setw(3) << std::setfill('0') << iteration;
			std::string image_name =
					ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "warp" +
					numStringStream.str() + ".png";
			cv::imwrite(image_name, warpImgOut);
		}
		// endregion
		std::cout << red << "Iteration: " << iteration << reset << std::endl;
		PrintOperationStatus("Calculating warp energy gradient...");
		CalculateWarpUpdate(canonicalScene, sourceLiveScene);
		PrintOperationStatus("Applying Sobolev smoothing to energy gradient...");
		ApplySmoothingToGradient(canonicalScene, sourceLiveScene);
		PrintOperationStatus("Applying warp update (based on energy gradient) to the cumulative warp...");
		maxVectorUpdate = ApplyWarpUpdateToWarp(canonicalScene, sourceLiveScene);
		PrintOperationStatus(
				"Updating live frame SDF by mapping from old live SDF to new live SDF based on latest warp update...");
		liveSceneReconstructor->ResetScene(targetLiveScene);
		ApplyWarpUpdateToLive(canonicalScene, sourceLiveScene, targetLiveScene);
		PrintLiveSceneStats(targetLiveScene,"new live scene after iteration-end interpolation");
		SwapSourceAndTargetLiveScenes(sourceLiveScene);

		if (recordWarpUpdates) {
			sceneLogger->SaveCurrentWarpState();
		}

	}
	PrintOperationStatus("*** Warp optimization finished for current frame. ***");

	// region ================================== WARP UPDATE RECORDING (AFTER OPTIMIZATION) ============================
	if (recordWarpUpdates) {
		sceneLogger->StopSavingWarpState();
		if (hasFocusCoordinates) {
			Vector3i sliceMinPoint(focusCoordinates[0] - FOCUS_SLICE_RADIUS,
			                       focusCoordinates[1] - FOCUS_SLICE_RADIUS,
			                       focusCoordinates[2] - FOCUS_SLICE_RADIUS);
			Vector3i sliceMaxPoint(focusCoordinates[0] + FOCUS_SLICE_RADIUS,
			                       focusCoordinates[1] + FOCUS_SLICE_RADIUS,
			                       focusCoordinates[2] + FOCUS_SLICE_RADIUS);
			std::cout << "Making slice around voxel " << green << focusCoordinates << reset << " with l_0 radius of "
			          << FOCUS_SLICE_RADIUS << "...";
			std::string sliceId;
			sceneLogger->MakeSlice(sliceMinPoint, sliceMaxPoint, currentFrameIx, sliceId);
			std::cout << "Slice finished." << std::endl;
			sceneLogger->SwitchActiveScene(sliceId);
		}
		delete sceneLogger;
	}
	// endregion =======================================================================================================
	energy_stat_file.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::GenerateCurrentFrameOutputPath() const {
	fs::path path(baseOutputDirectory + "/Frame_" + std::to_string(currentFrameIx));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::SwapSourceAndTargetLiveScenes(
		ITMScene<TVoxelLive, TIndex>*& sourceLiveScene) {
	ITMScene<TVoxelLive, TIndex>* tmp = sourceLiveScene;
	sourceLiveScene = targetLiveScene;
	targetLiveScene = tmp;
}

// endregion
