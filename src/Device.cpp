#include "ofxAzureKinectUtil.h"


namespace ofxAzureKinectUtil {

	const int32_t TIMEOUT_IN_MS = 1000;

	Device::Settings::Settings(int idx)
		: deviceIndex(idx)
		, depthMode(K4A_DEPTH_MODE_WFOV_2X2BINNED)
		, colorResolution(K4A_COLOR_RESOLUTION_1080P)
		, colorFormat(K4A_IMAGE_FORMAT_COLOR_BGRA32)
		, cameraFps(K4A_FRAMES_PER_SECOND_30)
		, sensorOrientation(K4ABT_SENSOR_ORIENTATION_DEFAULT)
		, updateDepth(true)
		, updateColor(true)
		, updateIr(false)
		, updateBodies(false)
		, updatePointCloud(true)
		, synchronized(true) {
	}

	Device::Device() : index(-1) {}
	Device::~Device() {}

	bool Device::open(const Settings& s) {

		if (bOpen) {
			ofLogWarning(__FUNCTION__) << "Device " << index << " already open!";
			return false;
		}

		index = s.deviceIndex;
		
		config = {
			s.colorFormat,
			s.colorResolution,
			s.depthMode,
			s.cameraFps,
			s.synchronized
		};

		isUseDepth = s.updateDepth;
		isUseColor = s.updateColor;
		isUseIR = s.updateIr;
		isUseBodies = s.updateBodies;
		isUsePointCloud = s.updatePointCloud;
		isUsePolygonMesh = s.updatePolygonMesh;
		
		switch (s.cameraFps) {
			case K4A_FRAMES_PER_SECOND_5: frameTime = 1000.f / 5.f; break;
			case K4A_FRAMES_PER_SECOND_15: frameTime = 1000.f / 15.f; break;
			case K4A_FRAMES_PER_SECOND_30: frameTime = 1000.f / 30.f; break;
			default: frameTime = 1000.f / 30.f; break;
		}

		trackerConfig.sensor_orientation = s.sensorOrientation;

		try {
			// Open connection to the device.
			device = k4a::device::open(static_cast<uint32_t>(s.deviceIndex));

			// Get the device serial number.
			serialNumber = device.get_serialnum();

		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			device.close();

			return false;
		}

		bOpen = true;
		ofLogNotice(__FUNCTION__) << "Successfully opened device " << s.deviceIndex << " with serial number " << this->serialNumber << ".";

		return bOpen;
	}

	bool Device::close() {
		if (!bOpen) return false;
		if (bPlaying) stop();

		device.close();

		index = -1;
		bOpen = false;
		serialNumber = "";

		return true;
	}

	bool Device::start() {
		if (!bOpen) {
			ofLogError(__FUNCTION__) << "Open device before starting cameras!";
			return false;
		}

		// Get calibration.
		try {
			calibration = device.get_calibration(config.depth_mode, config.color_resolution);
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		Interface::start();

		// Start cameras.
		try {
			device.start_cameras(&config);
			device.start_imu();
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		startThread();

		bPlaying = true;

		return bPlaying;
	}

	bool Device::stop() {

		bPlaying = false;

		Interface::stop();

		device.stop_cameras();
		device.stop_imu();

		return true;
	}

	void Device::updateCapture() {
		try {
			if (!device.get_capture(&capture, std::chrono::milliseconds(TIMEOUT_IN_MS))) {
				ofLogWarning(__FUNCTION__) << "Timed out waiting for a capture for device " << index << ".";
				return;
			}
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return;
		}
	}

	void Device::updateIMU() {
		try {
			if (!device.get_imu_sample(&imuSample, std::chrono::milliseconds(TIMEOUT_IN_MS))) {
				ofLogWarning(__FUNCTION__) << "Timed out waiting for a IMU for device " << index << ".";
				return;
			}
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return;
		}
	}

}