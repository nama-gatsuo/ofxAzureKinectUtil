#pragma once
#include "Types.h"
#include "Interface.h"
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

namespace ofxAzureKinectUtil {

	class Playback : public Interface {
	public:

		struct Settings {
			std::string fileName;

			bool updateDepth;
			bool updateColor;
			bool updateIr;
			bool updateBodies;
			bool updatePointCloud;
			bool updatePolygonMesh;

			Settings();
		};

		Playback();
		~Playback();

		bool open(const Settings& s);
		bool close();

		bool start() override;
		bool stop() override;
	protected:
		void updateCapture() override;

	private:
		bool isPlaying;
		k4a::playback playback;
		k4a_record_configuration_t config;
		std::chrono::milliseconds timePerFrame;
	};

	using PlaybackSettings = Playback::Settings;

};

