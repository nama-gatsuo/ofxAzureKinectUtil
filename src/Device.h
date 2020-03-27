#pragma once
#include "k4aTypes.h"
#include "Interface.h"
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

namespace ofxAzureKinectUtil {

	class Device : public Interface {

	public:
		struct Settings {
			int deviceIndex;

			DepthMode depthMode;
			ColorResolution colorResolution;
			ImageFormat colorFormat;
			Fps cameraFps;
			SensorOrientation sensorOrientation;

			bool updateDepth;
			bool updateColor;
			bool updateIr;
			bool updateBodies;
			bool updatePointCloud;
			bool updatePolygonMesh;

			bool synchronized;

			Settings(int idx = 0);
		};

		Device();
		~Device();

		bool open(const Settings& s);
		void close() override;

		void start() override;
		void stop() override;

		bool isStreaming() const { return bPlaying; }

		static int getInstalledCount() { return k4a_device_get_installed_count(); }

	protected:
		bool updateCapture() override;
		bool updateIMU() override;
	private:
		int index;
		std::string serialNumber;
		k4a::device device;

		k4a_device_configuration_t config;
	};

	using DeviceSettings = Device::Settings;
};

