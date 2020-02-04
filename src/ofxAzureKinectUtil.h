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

			bool updateColor;
			bool updateIr;
			bool updateBodies;
			bool updateWorld;
			bool updateVbo;

			bool synchronized;

			Settings(int idx = 0);
		};

		bool open(const Settings& s);
		bool close();

		bool start() override;
		bool stop() override;

	protected:
		void updateCapture() override;

	private:
		int index;
		std::string serialNumber;
		k4a::device device;
		
		k4a_device_configuration_t config;
		
		bool isStreaming;
	};

	using DeviceSettings = Device::Settings;

	/*class Playback : public Interface {
	public:

		struct Settings {
			std::string fileName;
		};

		bool open(const Settings& s);
		void close();

		bool start() override;
		bool stop() override;
	protected:
		void updateCapture() override;

	private:
		k4a::playback playback;
	};

	using PlaybackSettings = Playback::Settings;*/
	
};


