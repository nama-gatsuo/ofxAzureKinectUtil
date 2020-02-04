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

	protected:
		void updateCapture() override;

	private:
		int index;
		k4a::device device;
		
	};

	using DeviceSettings = Device::Settings;

	class Playback : public Interface {
	public:


	protected:
		void updateCapture() override;

	private:
		k4a::playback playback;
	};

	
};


