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
#include <Config.h>
#include "DepthSenseEngine.h"

#ifdef COMPILE_WITH_DSSDK

#include <DepthSense.hxx>
#include <queue>
#include <mutex>
#include <thread>
#include <sstream>

using namespace InfiniTAM::Engine;
using namespace DepthSense;

class DepthSenseEngine::Grabber {
public:
	DepthSense::Device device;
	DepthSense::ColorNode colorNode;
	DepthSense::DepthNode depthNode;
	DepthSense::Context context;
	std::shared_ptr<DepthSense::ProjectionHelper> projection;
	int colorSize, depthSize;

	static const Vector2i enumeratedResolutions[];

	Grabber(DepthSense::Device device, DepthSense::Context context):
			device(device),
			context(context){
		std::vector<Node> nodes = device.getNodes();
		for (Node node: nodes){
			if (node.is<DepthNode>()){
				depthNode = node.as<DepthNode>();
				projection.reset(new DepthSense::ProjectionHelper(device.getStereoCameraParameters()));
			}
			if (node.is<ColorNode>()){
				colorNode = node.as<ColorNode>();
			}
		}
	}
	void startDevice(){
		try{
			context.registerNode(colorNode);
			context.registerNode(depthNode);
			context.startNodes();
			context.run();
		}catch(DepthSense::ArgumentException e){
			printf("Unable to start DepthSense grabber, possibly disconnected.");
		}

	}
	void stopDevice(){
		try{
			context.unregisterNode(colorNode);
			context.unregisterNode(depthNode);
			if (context.getRegisteredNodes ().size () == 0)
				context.stopNodes ();
		}catch(DepthSense::ArgumentException e){
			printf("Unable to stop DepthSense grabber, possibly disconnected");
		}
	}
/*--------------------------------------------------------------------------------------------------------------------*/
	template<class NodeType, class ConfigType>
	void configureNodeHelper(NodeType node, ConfigType config){
		try{
			context.requestControl(node,0);
			node.setConfiguration(config);
			context.releaseControl(node);
		}catch (ArgumentException& e)
		{
			printf("Argument Exception: %s\n",e.what());
		}
		catch (UnauthorizedAccessException& e)
		{
			printf("Unauthorized Access Exception: %s\n",e.what());
		}
		catch (ConfigurationException& e)
		{
			printf("Configuration Exception: %s\n",e.what());
		}
		catch (StreamingException& e)
		{
			printf("Streaming Exception: %s\n",e.what());
		}
		catch (TimeoutException&)
		{
			printf("TimeoutException\n");
		}
	}


	void reconfigure(){
		configureDepthNode();
		configureColorNode();
	}

/*--------------------------------------------------------------------------------------------------------------------*/
	void configureDepthNode()
	{
		//TODO pick format and framerate based on requested

		DepthNode::Configuration config = depthNode.getConfiguration();
		config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
		config.framerate = 30;
		config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
		config.saturation = false;
		depthNode.setEnableDepthMapFloatingPoint (false);
		depthNode.setEnableUvMap (true);
		depthNode.setEnableConfidenceMap (true);
		depthNode.setEnableDepthMap(true);
		configureNodeHelper(depthNode, config);

		Vector2i depthResolution = enumeratedResolutions[config.frameFormat];
		depthSize = depthResolution.width * depthResolution.height * sizeof(uint16_t);
	}

/*--------------------------------------------------------------------------------------------------------------------*/
	void configureColorNode()
	{
		//TODO pick format and framerate based on requested
		ColorNode::Configuration config = colorNode.getConfiguration();
		config.frameFormat = FRAME_FORMAT_VGA;
		config.compression = COMPRESSION_TYPE_MJPEG;
		config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
		config.framerate = 30;
		colorNode.setEnableColorMap(true);
		configureNodeHelper(colorNode, config);

		Vector2i colorResolution = enumeratedResolutions[config.frameFormat];
		colorSize = colorResolution.width * colorResolution.height * 3;//3 channels

	}
/*--------------------------------------------------------------------------------------------------------------------*/
};

class DepthSenseEngine::Manager {
public:



	Manager(const char *deviceURI, Vector2i requestedColorResolution, Vector2i requestedDepthResolution, int maxQueueSize = 10) :
			context(DepthSense::Context::create("localhost")),
	        deviceConnected(false),
			maxQueueSize(maxQueueSize),
			requestedColorResolution(requestedColorResolution),
	        requestedDepthResolution(requestedDepthResolution),
	        confidenceThreshold(50)

	{
		std::vector<DepthSense::Device> devices = context.getDevices();


		if(devices.size() == 0){
			printf("Warning: failed to connect to DepthSesne grabber...");
			return;
		}

		if(deviceURI == nullptr){
			grabber = new Grabber(devices[0], context);
		}else{
			grabber = new Grabber(devices[0], context);
			printf("Warning: DS DeviceURI not yet supported...");
		}
		captureDevice(grabber);
		grabber->reconfigure();

		Vector2i depthResolution = DepthSenseEngine::Grabber::enumeratedResolutions[grabber->depthNode.getConfiguration().frameFormat];
		colorData.resize(grabber->colorSize);//
		depthData.resize(depthResolution.width * depthResolution.height);
		depthSize = grabber->depthSize;
		if(grabber->depthNode.isSet() && grabber->colorNode.isSet()){
			deviceConnected = true;
		}

		thread = new std::thread(&Grabber::startDevice, grabber);
	}



	/*============================ MEMBER VARIABLES ===============================*/
	Context context;
	Grabber* grabber;

	bool deviceConnected;

	std::queue<std::vector<uint16_t>> depthQueue;
	std::vector<uint16_t> depthData;
	std::vector<uint8_t> colorData;

	int depthSize;
	unsigned int maxQueueSize;
	std::mutex colorMutex;
	std::mutex depthMutex;
	std::thread* thread;

	Vector2i requestedColorResolution, requestedDepthResolution;

	int confidenceThreshold;
/*============================ MEMBER METHODS ================================*/
/*----------------------------------------------------------------------------*/
	~Manager(){
		grabber->stopDevice();
		releaseDevice(grabber);
		if(thread != nullptr){
			if(thread->joinable()){
				thread->join();
			}
		}
		delete thread;
		delete grabber;
	}
/*----------------------------------------------------------------------------*/

// New color sample event handler
	void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
	{
		std::lock_guard<std::mutex> lock(this->colorMutex);
		memcpy (&colorData[0], data.colorMap, colorData.size ());
	}
	/*----------------------------------------------------------------------------*/
	void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
	{
		std::lock_guard<std::mutex> lock(this->depthMutex);
		memcpy(depthData.data(), data.depthMap, depthSize);
		for(unsigned i = 0; i < depthData.size(); i++){
			if(data.confidenceMap[i] < confidenceThreshold){
				depthData[i] = 0;
			}
		}

	}
// New depth sample event handler
	void onNewDepthSampleBuffered(DepthNode node, DepthNode::NewSampleReceivedData data)
	{
		std::lock_guard<std::mutex> lock(this->depthMutex);
		std::vector<uint16_t> depthData (depthSize);
		memcpy(depthData.data(), data.depthMap, depthSize);
		for(int i =0; i < depthSize; i++){
			if(data.confidenceMap[i] < confidenceThreshold){
				depthData[i] = 0;
			}
		}
		while(depthQueue.size() >= maxQueueSize){
			depthQueue.pop();
		}
		depthQueue.push(depthData);
	}
//readers for access to color & depth data from other threads
	void readColor(ITMUChar4Image* rgbImage){
		std::lock_guard<std::mutex> lock(this->colorMutex);
		Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
		for (int i = 0, iOrig = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++)
		{
			Vector4u newPix;
			newPix.x = colorData[iOrig]; iOrig++;
			newPix.y = colorData[iOrig]; iOrig++;
			newPix.z = colorData[iOrig]; iOrig++;

			newPix.w = 255;
			rgb[i] = newPix;
		}
	}

	void readDepth(ITMShortImage* rawDepthImage){
		std::lock_guard<std::mutex> lock(this->depthMutex);
		short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
		memcpy(depth, &depthData[0], grabber->depthSize);
	}

	void readDepthBuffered(ITMShortImage* rawDepthImage){


		std::lock_guard<std::mutex> lock(this->depthMutex);

		if(depthQueue.size() > 0 ){
			std::vector<uint16_t> depthData = depthQueue.front();
			short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
			memcpy(depth, &depthData[0], grabber->depthSize);
		}else{
			short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
			memset(depth, 0, grabber->depthSize);
		}
	}

	void captureDevice(Grabber* device){
		device->colorNode.newSampleReceivedEvent().connect(this, &DepthSenseEngine::Manager::onNewColorSample);
		device->depthNode.newSampleReceivedEvent().connect(this, &DepthSenseEngine::Manager::onNewDepthSample);
	}
	void releaseDevice(Grabber* device){
		device->colorNode.newSampleReceivedEvent().disconnect(this, &DepthSenseEngine::Manager::onNewColorSample);
		device->depthNode.newSampleReceivedEvent().disconnect(this, &DepthSenseEngine::Manager::onNewDepthSample);
	}


};

const Vector2i DepthSenseEngine::Grabber::enumeratedResolutions[]  = {
	Vector2i(0,0),
	Vector2i(160,120),
	Vector2i(176,144),
	Vector2i(240,160),
	Vector2i(320,240),
	Vector2i(352,288),
	Vector2i(480,320),
	Vector2i(640,480),
	Vector2i(1280,720),
	Vector2i(320,120),
	Vector2i(1024,768),
	Vector2i(800,600),
	Vector2i(636,438),
	Vector2i(640,240),
	Vector2i(640,360)
};


DepthSenseEngine::DepthSenseEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration,
                   Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
		:
		ImageSourceEngine(calibFilename),
		manager(new Manager(deviceURI, requested_imageSize_rgb, requested_imageSize_d)),
        colorResolution(0,0),
        depthResolution(0,0)
{
	if(manager->deviceConnected){
		colorResolution = DepthSenseEngine::Grabber::enumeratedResolutions[manager->grabber->colorNode.getConfiguration().frameFormat];
		depthResolution = DepthSenseEngine::Grabber::enumeratedResolutions[manager->grabber->depthNode.getConfiguration().frameFormat];
	}
}

DepthSenseEngine::~DepthSenseEngine() {
	if(manager != nullptr){
		delete manager;
	}
}

bool DepthSenseEngine::hasMoreImages(void) {
	return (manager != nullptr && manager->deviceConnected);
}

Vector2i DepthSenseEngine::getDepthImageSize(void) {
	return depthResolution;
}

Vector2i DepthSenseEngine::getRGBImageSize(void) {
	return colorResolution;
}

void DepthSenseEngine::getImages(ITMUChar4Image* rgbImage, ITMShortImage* rawDepthImage) {
	if(manager->deviceConnected){
		manager->readColor(rgbImage);
		manager->readDepth(rawDepthImage);
	}else{
		Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
		memset(rgb,0,rgbImage->dataSize * sizeof(Vector4u));
		short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
		memset(depth, 0, rawDepthImage->dataSize * sizeof(short));
	}
}


#else
using namespace InfiniTAM::Engine;

DepthSenseEngine::DepthSenseEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{
	printf("compiled without DepthSense support\n");
}
DepthSenseEngine::~DepthSenseEngine()
{}
void DepthSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool DepthSenseEngine::hasMoreImages(void)
{ return false; }
Vector2i DepthSenseEngine::getDepthImageSize(void)
{ return Vector2i(0,0); }
Vector2i DepthSenseEngine::getRGBImageSize(void)
{ return Vector2i(0,0); }

#endif

