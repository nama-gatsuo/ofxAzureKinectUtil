#pragma once
#include "ofThread.h"
#include "ofThreadChannel.h"
#include "ofParameter.h"
#include "ofPixels.h"
#include "ofTexture.h"
#include "ofVboMesh.h"
#include "k4aTypes.h"

#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include "turbojpeg.h"
#include "attitude_estimator.h"

namespace ofxAzureKinectUtil {
	class Interface : protected ofThread {
	public:
		// All public members are intended to be accessed by main thread
		Interface();
		virtual ~Interface();

		bool open();
		virtual void start();
		virtual void stop();
		virtual void close();

		/// Shoud be called from main thread such as ofApp::update()
		void update();
		bool isFrameNew() { return bFrameNew; }

		ofParameterGroup& getParameters() { return group; }
		const ofTexture& getIrTex() const { return irTex; }
		const ofTexture& getDepthTex() const { return depthRemappedTex; }
		const ofTexture& getColorTex() const { return colorTex; }
		const ofVboMesh& getPointCloud() const { return pointCloud; }
		const ofVboMesh& getPolygonMesh() const { return polygon; }

		const ofTexture& getBodyIndexTex() const { return bodyIndexTex; }
		const std::vector<k4abt_skeleton_t>& getBodySkeletons() const { return bodySkeletons; }
		const std::vector<uint32_t>& getBodyIDs() const { return bodyIDs; }

		const ofTexture& getRayTex() const { return rayTex; }
		const IMU& getIMU() const { return imu; }
		const glm::quat& getOrientation() const { return estimatedOrientation; }
		const std::chrono::milliseconds& getCurrentTime() const { return currentTime; }
		size_t getCurrentFrameCount() const { return frameCount; }

		void setPixelSize(int s) { pixelSize = s; }
		void setRad(float r) { rad = r; }
	protected:
		// Accessed in a main thread
		bool bOpen;
		bool bPlaying;
		bool bFrameNew;

		ofParameterGroup group;

		struct FrameData {
			ofPixels colorPix;
			ofShortPixels irPix;
			ofShortPixels depthRemappedPix;
			ofMesh pointCloud;
			ofMesh polygon;

			ofPixels bodyIndexPix;
			std::vector<k4abt_skeleton_t> bodySkeletons;
			std::vector<uint32_t> bodyIDs;

			IMU imu;
		} fd;
	private:
		ofTexture irTex, colorTex; // Raw textures
		ofTexture depthRemappedTex; // Depth texture in Color Camera Space
		ofVboMesh pointCloud;
		ofVboMesh polygon;

		ofTexture rayTex; // Ray of color camera (Color Camera Space -> Wolrd Space)
		k4a::image rayImg;

		ofTexture bodyIndexTex;
		std::vector<k4abt_skeleton_t> bodySkeletons;
		std::vector<uint32_t> bodyIDs;

		IMU imu;
		glm::quat estimatedOrientation;

		ofParameter<int> pixelSize;
		ofParameter<float> rad;
		ofParameter<bool> frameSync;

		bool createRayTex();

	protected:
		// Accessed in a second thread
		bool isUseDepth;
		bool isUseColor;
		bool isUseIR;
		bool isUseBodies;
		bool isUsePointCloud;
		bool isUsePolygonMesh;
		float frameTime; // millisec

		k4a::capture capture;
		k4a::calibration calibration;
		k4a::transformation transformation;
		k4abt_tracker_configuration_t trackerConfig;
		k4abt::tracker bodyTracker;
		k4a_imu_sample_t imuSample;

		std::chrono::milliseconds currentTime;
		size_t frameCount;

		virtual bool updateCapture() = 0;
		virtual bool updateIMU() = 0;
		void resetOrientationEstimation();
	private:
		void threadedFunction() override;
		ofShortPixels createDepthRemapped(const k4a::image& depth, const k4a::image& color);
		ofMesh createPointCloud(const k4a::image& frameImg, const k4a::image& tableImg);
		ofMesh createPolygonMesh(const k4a::image& frameImg, const k4a::image& tableImg);

		k4a::image depthRemappedImg;

		ofThreadChannel<bool> request;
		ofThreadChannel<FrameData> response;

		tjhandle jpegDecompressor;
		stateestimation::AttitudeEstimator ae;

		class ScopedFrameSync {
		public:
			ScopedFrameSync(float frameTimeInMill, bool enabled=true) :
				startTime(ofGetElapsedTimeMicros()),
				frameTime(frameTimeInMill),
				enabled(enabled)
			{}
			~ScopedFrameSync() {
				// Sync with certain frame rate the kinect has
				uint64_t endTime = ofGetElapsedTimeMicros();
				int dt = (endTime - startTime) / 1000.f;
				int waitTime = frameTime - dt;
				if (enabled && waitTime > 0) ofSleepMillis(waitTime);
			}

		private:
			const bool enabled;
			const uint64_t startTime;
			const float frameTime;
		};

	};
}