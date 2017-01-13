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

//TODO: use for debugging
//static int getThreadId(){
//	std::stringstream ss;
//	ss << std::this_thread::get_id();
//	uint64_t id = std::stoull(ss.str());
//	return id;
//}

struct CapturedDevice{
	DepthSense::Device device;
	DepthSense::ColorNode colorNode;
	DepthSense::DepthNode depthNode;
	CapturedDevice(DepthSense::Device device){

	}
};

class DepthSenseEngine::PrivateData {
public:



	PrivateData(const char *deviceURI, Vector2i requestedColorResolution, Vector2i requestedDepthResolution, int maxQueueSize = 10) :
			context(DepthSense::Context::create("localhost")),
			devices(context.getDevices()),
	        deviceConnected(false),
			maxQueueSize(maxQueueSize),
			requestedColorResolution(requestedColorResolution),
	        requestedDepthResolution(requestedDepthResolution),
	        confidenceThreshold(50)

	{


		if(devices.size() == 0){
			printf("Warning: failed to connect to DepthSesne device...");
			return;
		}
		deviceConnected = true;
		if(deviceURI == nullptr){
			capturedDevice = devices[0];
		}else{
			capturedDevice = devices[0];
			printf("Warning: DS DeviceURI not yet supported...");
		}

#ifdef WITH_ONLINE_DEVICE_HANDLERS
		context.deviceAddedEvent().connect(this,&DepthSenseEngine::PrivateData::onDeviceConnected);
		context.deviceRemovedEvent().connect(this,&DepthSenseEngine::PrivateData::onDeviceDisconnected);
		capturedDevice.nodeAddedEvent().connect(this, &PrivateData::onNodeConnected);
		capturedDevice.nodeRemovedEvent().connect(this, &PrivateData::onNodeDisconnected);
#endif
		std::vector<Node> nodes = capturedDevice.getNodes();

		printf("Found %u nodes\n", static_cast<unsigned >(nodes.size()));
		for (Node node: nodes){
			configureNode(node);
		}
		projection.reset(new DepthSense::ProjectionHelper(capturedDevice.getStereoCameraParameters()));
		thread = new std::thread(&PrivateData::start_helper, this);
	}

	void start_helper(){
		context.startNodes();
		//context.run();
	}

	static const Vector2i enumeratedResolutions[];
	static  std::array<Vector2i,14> getEnumeratedResolutions(){
		return {{ Vector2i(160,120),
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
				Vector2i(640,360)}};
	}


	/*============================ MEMBER VARIABLES ===============================*/
	Context context;
	std::vector<DepthSense::Device> devices;
	DepthSense::Device capturedDevice;

	bool deviceConnected;
	DepthNode depthNode;
	ColorNode colorNode;
	AudioNode audioNode;
	std::shared_ptr<DepthSense::ProjectionHelper> projection;

	std::queue<std::vector<uint16_t>> depthQueue;
	std::vector<uint8_t> colorData;
	unsigned int maxQueueSize;
	std::mutex colorMutex;
	std::mutex depthMutex;
	std::thread* thread;

	Vector2i requestedColorResolution, requestedDepthResolution;
	Vector2i colorResolution, depthResolution;
	int colorSize, depthSize;
	int confidenceThreshold;
/*============================ MEMBER METHODS ================================*/
/*----------------------------------------------------------------------------*/
	~PrivateData(){
		context.stopNodes();
		if (colorNode.isSet()) context.unregisterNode(colorNode);
		if (depthNode.isSet()) context.unregisterNode(depthNode);
		if (audioNode.isSet()) context.unregisterNode(audioNode);
		if(thread != nullptr){
			if(thread->joinable()){
				thread->join();
			}
		}
		delete thread;
	}
/*----------------------------------------------------------------------------*/

// New color sample event handler
	void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
	{
		std::lock_guard<std::mutex> lock(this->colorMutex);
		memcpy (&colorData[0], data.colorMap, colorData.size ());
	}
	/*----------------------------------------------------------------------------*/
// New depth sample event handler
	void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
	{
		std::lock_guard<std::mutex> lock(this->depthMutex);
		std::vector<uint16_t> depth_data (depthSize);
		memcpy(depth_data.data(), &data.depthMap, depthSize * sizeof(uint16_t));
		for(int i =0; i < depthSize; i++){
			if(data.confidenceMap[i] < confidenceThreshold){
				depth_data[i] = 0;
			}
		}
		while(depthQueue.size() >= maxQueueSize){
			depthQueue.pop();
		}
		depthQueue.push(depth_data);
	}

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

	void readDepth(ITMShortImage *rawDepthImage){
		//printf("readDepth entered");
		std::lock_guard<std::mutex> lock(this->depthMutex);
		//printf("readDepth past lock");
		std::vector<uint16_t> depthData = depthQueue.front();
		short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
		memcpy(depth, &depthData[0], rawDepthImage->dataSize * sizeof(short));
	}

	template<class NodeType, class ConfigType>
	void configureNodeHelper(NodeType node, ConfigType config){
		try{
			context.requestControl(node,0);
			node.setConfiguration(config);
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

	/*----------------------------------------------------------------------------*/
	void configureAudioNode()
	{
		//we don't need to gather audio samples here, so don't bind new data event
		AudioNode::Configuration config = audioNode.getConfiguration();
		config.sampleRate = 44100;
		configureNodeHelper(audioNode, config);
		audioNode.setInputMixerLevel(0.5f);
	}

/*----------------------------------------------------------------------------*/
	void configureDepthNode()
	{
		depthNode.newSampleReceivedEvent().connect(this, &DepthSenseEngine::PrivateData::onNewDepthSample);

		DepthNode::Configuration config = depthNode.getConfiguration();

		//TODO pick format based on requested
		config.frameFormat = FRAME_FORMAT_QVGA;
		depthResolution = enumeratedResolutions[config.frameFormat];
		depthSize = depthResolution.width * depthResolution.height;
		config.framerate = 25;
		config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
		config.saturation = true;

		depthNode.setEnableVertices(true);
		configureNodeHelper(depthNode, config);
	}


/*----------------------------------------------------------------------------*/
	void configureColorNode()
	{
		// connect new color sample handler
		colorNode.newSampleReceivedEvent().connect(this, &DepthSenseEngine::PrivateData::onNewColorSample);
		ColorNode::Configuration config = colorNode.getConfiguration();
		//TODO pick format based on requested
		config.frameFormat = FRAME_FORMAT_VGA;
		colorResolution = enumeratedResolutions[config.frameFormat];
		colorSize = colorResolution.width * colorResolution.height;
		colorData.resize(colorSize);
		config.compression = COMPRESSION_TYPE_MJPEG;
		config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
		config.framerate = 25;
		configureNodeHelper(colorNode, config);
	}
/*----------------------------------------------------------------------------*/
	void configureDevice(DepthSense::Device& device){

	}

/*----------------------------------------------------------------------------*/
	void configureNode(Node node)
	{

		if ((node.is<DepthNode>())&&(!depthNode.isSet()))
		{
			depthNode = node.as<DepthNode>();
			configureDepthNode();

			context.registerNode(node);
		}
		if ((node.is<ColorNode>())&&(!colorNode.isSet()))
		{
			colorNode = node.as<ColorNode>();
			configureColorNode();
			context.registerNode(node);
		}
#ifdef WITH_AUDIO
		if ((node.is<AudioNode>())&&(!audioNode.isSet()))
		{
			audioNode = node.as<AudioNode>();
			configureAudioNode();
			context.registerNode(node);
		}
#endif
	}

#ifdef WITH_ONLINE_DEVICE_HANDLERS
	void onDeviceConnected(Context context, Context::DeviceAddedData data)
	{
		if (!deviceConnected)
		{
			data.device.nodeAddedEvent().connect(this, &DepthSenseEngine::PrivateData::onNodeConnected);
			data.device.nodeRemovedEvent().connect(this, &DepthSenseEngine::PrivateData::onNodeDisconnected);
			deviceConnected = true;
		}
	}

	void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
	{
		deviceConnected = false;
		printf("Device disconnected\n");
	}

		void onNodeConnected(Device device, Device::NodeAddedData data)
	{
		configureNode(data.node);
	}

	/*----------------------------------------------------------------------------*/
	void onNodeDisconnected(Device device, Device::NodeRemovedData data)
	{
		if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == audioNode))
			audioNode.unset();
		if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == colorNode))
			colorNode.unset();
		if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == depthNode))
			depthNode.unset();
		printf("Node disconnected\n");
	}

/*----------------------------------------------------------------------------*/
#endif

};


// // Borrowed for reference from DepthSense API:
//	enum FrameFormat
//	{
//		FRAME_FORMAT_UNKNOWN = 0,/*!< unknown */
//		FRAME_FORMAT_QQVGA = 1,/*!< QQVGA (160x120) */
//		FRAME_FORMAT_QCIF = 2,/*!< QCIF (176x144) */
//		FRAME_FORMAT_HQVGA = 3,/*!< HQVGA (240x160) */
//		FRAME_FORMAT_QVGA = 4,/*!< QVGA (320x240) */
//		FRAME_FORMAT_CIF = 5,/*!< CIF (352x288) */
//		FRAME_FORMAT_HVGA = 6,/*!< HVGA (480x320) */
//		FRAME_FORMAT_VGA = 7,/*!< VGA (640x480) */
//		FRAME_FORMAT_WXGA_H = 8,/*!< WXGA_H (1280x720) */
//		FRAME_FORMAT_DS311 = 9,/*!< DS311 (320x120) */
//		FRAME_FORMAT_XGA = 10,/*!< XGA (1024x768) */
//		FRAME_FORMAT_SVGA = 11,/*!< SVGA (800x600) */
//		FRAME_FORMAT_OVVGA = 12,/*!< OVVGA (636x480) */
//		FRAME_FORMAT_WHVGA = 13,/*!< WHVGA (640x240) */
//		FRAME_FORMAT_NHD = 14,/*!< nHD (640x360) */
//		FRAME_FORMAT_STEREOLR = 15,/*!< StereoLR (320x480) */
//	};
//const Vector2i DepthSenseEngine::PrivateData::enumeratedResolutions[];
const Vector2i DepthSenseEngine::PrivateData::enumeratedResolutions[]  = {
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
		: ImageSourceEngine(calibFilename), data(new PrivateData(deviceURI, requested_imageSize_rgb, requested_imageSize_d))
{

}

DepthSenseEngine::~DepthSenseEngine() {
	if(data != nullptr){
		delete data;
	}
}

bool DepthSenseEngine::hasMoreImages(void) {
	return (data != nullptr && data->depthQueue.size() > 0);
}

Vector2i DepthSenseEngine::getDepthImageSize(void) {
	if(data->depthNode.isSet()){
		return DepthSenseEngine::PrivateData::enumeratedResolutions[data->depthNode.getConfiguration().frameFormat];
		//return DepthSenseEngine::PrivateData::getEnumeratedResolutions()[data->depthNode.getConfiguration().frameFormat];
	}else{
		return Vector2i(0,0);
	}
}

Vector2i DepthSenseEngine::getRGBImageSize(void) {
	if(data->colorNode.isSet()){
		return DepthSenseEngine::PrivateData::enumeratedResolutions[data->colorNode.getConfiguration().frameFormat];
		//return DepthSenseEngine::PrivateData::getEnumeratedResolutions()[data->colorNode.getConfiguration().frameFormat];
	} else{

		return Vector2i(0,0);
	}

}

void DepthSenseEngine::getImages(ITMUChar4Image* rgbImage, ITMShortImage* rawDepthImage) {
	bool colorAvailable = data->colorNode.isSet();
	bool depthAvailable = data->colorNode.isSet();

	if(colorAvailable){
		data->readColor(rgbImage);
	}else{
		Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
		memset(rgb,0,rgbImage->dataSize * sizeof(Vector4u));
	}
	if(depthAvailable){
		data->readDepth(rawDepthImage);
	}else{
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

