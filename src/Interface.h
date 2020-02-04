#pragma once
#include "ofThread.h"
#include "ofThreadChannel.h"
#include "ofParameter.h"
#include "ofPixels.h"
#include "ofTexture.h"

#include <k4a/k4a.hpp>
#include <k4abt.h>

namespace ofxAzureKinectUtil {
	class Interface : protected ofThread {
	public:
		// All public members are intended to be accessed by main thread
		void close();

		virtual bool start();
		virtual bool stop();

		/// Shoud be called from main thread such as ofApp::update()
		void update();
		ofParameterGroup& getParameters() { return group; }
	protected:
		// Accessed in a main thread
		bool isOpen;
		
		ofParameterGroup group;

		struct FrameData {
			ofShortPixels depthPix;
			ofPixels colorPix;
			ofShortPixels irPix;

			ofShortPixels depthInColorPix;
			ofFloatPixels depthToWorldPix;
			ofPixels colorInDepthPix;
			ofFloatPixels colorToWorldPix;
			
			ofPixels bodyIndexPix;
			std::vector<k4abt_skeleton_t> bodySkeletons;
			std::vector<uint32_t> bodyIDs;
		} fd;
	private:
		ofTexture irTex, depthTex, colorTex; // Raw textures
		ofTexture depthRemapedTex; // Depth texture in color camera space (ScreenSpace)
		
		ofTexture rayTex; // Ray of color camera (ScreenSpace -> CameraSpace)
		k4a::image rayImg;
		bool createRayTex();
		

	protected:
		// Accessed in a second thread
		bool isUseColor;
		bool isUseIR;
		bool isUseDepth;
		bool isUseBodies;
		bool isUsePointCloud;

		k4a::capture capture;
		k4a::calibration calibration;
		k4a::transformation transformation;
		k4abt_tracker_configuration_t trackerConfig;
		k4abt_tracker_t bodyTracker;
		
		virtual void updateCapture() = 0;
	private:
		void threadedFunction() override;

		ofThreadChannel<bool> request;
		ofThreadChannel<FrameData> response;

		

	};
}