#pragma once
#include "ofThread.h"
#include "ofThreadChannel.h"
#include "ofParameter.h"
#include "ofPixels.h"
#include "ofTexture.h"
#include "ofVboMesh.h"

#include <k4a/k4a.hpp>
#include <k4abt.h>

namespace ofxAzureKinectUtil {
	class Interface : protected ofThread {
	public:
		// All public members are intended to be accessed by main thread
		Interface();
		virtual ~Interface();

		virtual bool start();
		virtual bool stop();

		/// Shoud be called from main thread such as ofApp::update()
		void update();

		ofParameterGroup& getParameters() { return group; }
		const ofTexture& getIrTex() const { return irTex; }
		const ofTexture& getDepthTex() const { return depthRemappedTex; }
		const ofTexture& getColorTex() const { return colorTex; }
		const ofVboMesh& getPointCloud() const { return pointCloud; }

		const ofTexture& getBodyIndexTex() const { return bodyIndexTex; }
		const std::vector<k4abt_skeleton_t>& getBodySkeletons() const { return bodySkeletons; }
		const std::vector<uint32_t>& getBodyIDs() const { return bodyIDs; }

	protected:
		// Accessed in a main thread
		bool isOpen;
		
		ofParameterGroup group;

		struct FrameData {
			ofPixels colorPix;
			ofShortPixels irPix;
			ofShortPixels depthRemappedPix;
			ofMesh pointCloud;

			ofPixels bodyIndexPix;
			std::vector<k4abt_skeleton_t> bodySkeletons;
			std::vector<uint32_t> bodyIDs;
		} fd;
	private:
		ofTexture irTex, colorTex; // Raw textures
		ofTexture depthRemappedTex; // Depth texture in Color Camera Space
		ofVboMesh pointCloud;

		ofTexture rayTex; // Ray of color camera (Color Camera Space -> Wolrd Space)
		k4a::image rayImg;

		ofTexture bodyIndexTex;
		std::vector<k4abt_skeleton_t> bodySkeletons;
		std::vector<uint32_t> bodyIDs;

		bool createRayTex();
		ofShortPixels createDepthRemapped(const k4a::image& depth, const k4a::image& color);

	protected:
		// Accessed in a second thread
		bool isUseDepth;
		bool isUseColor;
		bool isUseIR;
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