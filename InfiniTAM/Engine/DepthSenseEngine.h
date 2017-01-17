//  ================================================================
//  Created by Gregory Kramida on 1/10/17.
//  Copyright (c) 2017 Gregory Kramida
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
#include "ImageSourceEngine.h"

namespace InfiniTAM {
	namespace Engine {
		class DepthSenseEngine : public ImageSourceEngine {
		private:
			class Grabber;
			class Manager;
			Manager* manager;
			Vector2i colorResolution, depthResolution;
		public:
			DepthSenseEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = true,
			             Vector2i requested_imageSize_rgb = Vector2i(1280, 720), Vector2i requested_imageSize_d = Vector2i(320, 240));

			~DepthSenseEngine();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};

	}
}



