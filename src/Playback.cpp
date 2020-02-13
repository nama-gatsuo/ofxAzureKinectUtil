#include "Playback.h"

namespace ofxAzureKinectUtil {

	Playback::Settings::Settings()
		: fileName("")
		, updateDepth(true)
		, updateColor(true)
		, updateIr(false)
		, updateBodies(false)
		, updatePointCloud(true)
		, updatePolygonMesh(false)
	{}

	Playback::Playback() : isPlaying(false) {}
	Playback::~Playback() {}

	bool Playback::open(const Settings& s) {

		isUseDepth = s.updateDepth;
		isUseColor = s.updateColor;
		isUseIR = s.updateIr;
		isUseBodies = s.updateBodies;
		isUsePointCloud = s.updatePointCloud;
		isUsePolygonMesh = s.updatePolygonMesh;

		try {
			std::string fileName = "data/" + s.fileName;
			playback = k4a::playback::open(fileName.data());
			config = playback.get_record_configuration();

		} catch (const k4a::error& e) {
			ofLogError(__FUNCTION__) << e.what();
			playback.close();
			return false;
		}

		auto durationMS = std::chrono::duration_cast<std::chrono::milliseconds>(playback.get_recording_length());
		int min = durationMS.count() / 1000 / 60;
		int sec = (durationMS.count() - min * 1000 * 60) / 1000;
		int msec = durationMS.count() % 1000;

		isOpen = true;
		ofLogNotice(__FUNCTION__) << "Successfully opened playback file " << s.fileName << ".";
		ofLogNotice(__FUNCTION__) << "duration: " << min << "\"" << sec << "\'" << msec << ".";
		
		int fps = 30;
		switch (config.camera_fps)
		{
			case K4A_FRAMES_PER_SECOND_5: fps = 5; break;
			case K4A_FRAMES_PER_SECOND_15: fps = 15; break;
			case K4A_FRAMES_PER_SECOND_30: fps = 30; break;
			default: break;
		}
		timePerFrame = std::chrono::milliseconds(1000 / 60);
		ofLogNotice(__FUNCTION__) << "fps: " << fps << ".";
		return isOpen;
	}

	bool Playback::close() {
		if (!isOpen) return false;
		if (isPlaying) stop();

		playback.close();
		return true;
	}

	bool Playback::start() {
		if (isPlaying) return false;

		try {
			calibration = playback.get_calibration();
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		Interface::start();

		isPlaying = true;

		startThread();

		return true;
	}

	bool Playback::stop() {

		isPlaying = false;

		Interface::stop();

		return true;
	}

	void Playback::updateCapture() {

		try {
			bool isEnd = !playback.get_next_capture(&capture);
			if (isEnd) playback.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return;
		}
	}

	void Playback::updateIMU() {
		try {
			playback.get_next_imu_sample(&imuSample);
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return;
		}
	}
}

