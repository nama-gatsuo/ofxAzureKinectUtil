#pragma once
#include "Types.h"
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

			bool synchronized;

			Settings(int idx = 0);
		};

		Device();
		~Device();

		bool open(const Settings& s);
		bool close();

		bool start() override;
		bool stop() override;

		static int getInstalledCount() { return k4a_device_get_installed_count(); }

	protected:
		void updateCapture() override;

	private:
		int index;
		bool isStreaming;
		std::string serialNumber;
		k4a::device device;

		k4a_device_configuration_t config;
	};

	using DeviceSettings = Device::Settings;
};

