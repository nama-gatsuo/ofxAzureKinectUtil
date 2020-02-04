#pragma once
#include "ofThread.h"
#include "ofThreadChannel.h"
#include "ofParameter.h"
#include "ofPixels.h"

#include <k4a/k4a.hpp>

namespace ofxAzureKinectUtil {
	class Interface : protected ofThread {
	public:

		void update();

	protected:
		// Accessed in a main thread
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
		} fd;

	protected:
		// Accessed in a second thread only
		k4a::capture capture;
		virtual void updateCapture() = 0;
	private:
		void threadedFunction() override;

		ofThreadChannel<bool> request;
		ofThreadChannel<FrameData> response;

		bool isUseColor;
		bool isUseIR;
		bool isUseDepth;
		bool isUseBodies;

	};
}