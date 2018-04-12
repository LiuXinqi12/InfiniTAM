//  ================================================================
//  Created by Gregory Kramida on 1/22/18.
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

//========================= TEMPORARY FLAGS FOR DEBUGGING DYNAMIC FUSION FEATURE =======================================
#define _DEBUG

#ifdef _DEBUG

//#define USE_COLOR

#include "Utils/FileIO/ITMSceneLogger.h"


#include <opencv2/core/mat.hpp>

//*** 2D RASTERIZATION FOR VISUAL DEBUGGING ***

//#define RASTERIZE_CANONICAL_SCENE
//#define RASTERIZE_LIVE_SCENE
//#define DRAW_IMAGE
#if defined(DRAW_IMAGE) || defined(RASTERIZE_CANONICAL_SCENE) || defined(RASTERIZE_LIVE_SCENE)
#include "../../Utils/ITMSceneSliceRasterizer.h"
#endif

//*** DEBUG OUTPUT MESSAGES FOR UPDATE WARP ON CPU ***

//#define PRINT_TIME_STATS //-- needs rearranging of TICs and TOCs
#define PRINT_SINGLE_VOXEL_RESULT //Caution: very verbose!
#define PRINT_MAX_WARP_AND_UPDATE
#define PRINT_ENERGY_STATS
#define PRINT_ADDITIONAL_STATS
#define PRINT_DEBUG_HISTOGRAM
//#define OPENMP_WARP_UPDATE_COMPUTE_DISABLE
//***

#ifdef PRINT_TIME_STATS
#define TIC(var)\

#define TOC(var)\
    auto end_##var = std::chrono::steady_clock::now();\
    auto diff_##var = end_##var - start_##var;\
    var += std::chrono::duration <double, std::milli> (diff_##var).count();
#else
#define TIC(var)
#define TOC(var)
#endif

//different logging parameters
#define FOCUS_SLICE_RADIUS 2

#ifdef PRINT_ENERGY_STATS
#define WRITE_ENERGY_STATS_TO_FILE
#endif

#endif //ifdef _DEBUG