#include "Playback.h"

namespace ofxAzureKinectUtil {

	Playback::Settings::Settings()
		: fileName("")
		, updateDepth(true)
		, updateColor(true)
		, updateIr(false)
		, updateBodies(false)
		, updatePointCloud(true)
		, updatePolygonMesh(false) {
	}

	Playback::Playback() : bLoop(true), bEnd(false) {}
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

		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			playback.close();
			return false;
		}

		switch (config.camera_fps) {
		case K4A_FRAMES_PER_SECOND_5: frameTime = 1000.f / 5.f; break;
		case K4A_FRAMES_PER_SECOND_15: frameTime = 1000.f / 15.f; break;
		case K4A_FRAMES_PER_SECOND_30: frameTime = 1000.f / 30.f; break;
		default: frameTime = 1000.f / 30.f; break;
		}

		duration = std::chrono::duration_cast<std::chrono::milliseconds>(playback.get_recording_length());

		try {
			calibration = playback.get_calibration();
			// playback.seek_timestamp(std::chrono::duration_cast<std::chrono::microseconds>(currentTime), K4A_PLAYBACK_SEEK_BEGIN);
		} catch (const k4a::error& e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		Interface::open();

		ofLogNotice(__FUNCTION__) << "Successfully opened playback file " << s.fileName << ".";
		ofLogNotice(__FUNCTION__) << "duration: " << toString(duration) << ".";

		return bOpen;
	}

	void Playback::close() {
		if (!bOpen) return;
		if (bPlaying) stop();

		Interface::close();
		playback.close();
	}

	void Playback::start() {

		playback.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);

		frameCount = -1;
		resetOrientationEstimation();

		bEnd = false;

		Interface::start();
	}

	bool Playback::updateCapture() {

		if (!bPlaying) return false;

		try {
			bEnd = !playback.get_next_capture(&capture);

			if (bEnd) {
				if (bLoop) {
					playback.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
					resetOrientationEstimation();
					frameCount = -1;
					bEnd = false;
				} else {
					stop();
				}
			}
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		return true;
	}

	bool Playback::updateIMU() {

		if (!bPlaying) return false;

		try {
			playback.get_next_imu_sample(&imuSample);
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}
		return true;
	}
	std::string Playback::toString(std::chrono::milliseconds duration) {
		int min = duration.count() / 1000 / 60;
		int sec = (duration.count() - min * 1000 * 60) / 1000;
		int msec = duration.count() % 1000;
		
		std::stringstream str;
		if (sec < 10) str << min << "\'0" << sec << "." << msec;
		else str << min << "\'" << sec << "." << msec;
		
		return str.str();
	}
}

