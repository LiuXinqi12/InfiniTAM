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
#pragma once


#include "../../Objects/Scene/ITMScene.h"

namespace ITMLib {
	template<class TVoxel, class TIndex>
	class ITMSceneMotionTracker {
	protected:

		virtual float UpdateWarpField(ITMScene <TVoxel, TIndex>* canonicalScene,
		                              ITMScene <ITMVoxelAux, TIndex>* liveScene) = 0;

		virtual void FuseFrame(ITMScene <TVoxel, TIndex>* canonicalScene, ITMScene <ITMVoxelAux, TIndex>* liveScene) = 0;
		//TODO -- make all of these parameters
		const int maxIterationCount = 300;
		const float maxVectorUpdateThresholdMeters = 0.0001f;//m
		//_DEBUG
		//const float gradientDescentLearningRate = 1.0f;
		const float gradientDescentLearningRate = 0.1f;
		//_DEBUG
		const float rigidityEnforcementFactor = 0.0f;
		//const float rigidityEnforcementFactor = 0.1f;
		const float weightKillingTerm = 0.5f;
		const float weightLevelSetTerm = 0.2f;
		const float weightColorDataTerm = 0.0f;
		//_DEBUG
		const float colorSdfThreshold = -1.00f;
		//const float colorSdfThreshold = 0.25f;
		const float epsilon = 1.0e-10f;

		float maxVectorUpdateThresholdVoxels;
	public:
		explicit ITMSceneMotionTracker(const ITMSceneParams& params);
		void ProcessFrame(ITMScene <TVoxel, TIndex>* canonicalScene,
		                  ITMScene <ITMVoxelAux, TIndex>* liveScene);
	};


}//namespace ITMLib



