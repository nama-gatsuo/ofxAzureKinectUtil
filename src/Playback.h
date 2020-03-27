#pragma once
#include "k4aTypes.h"
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
		void close() override;

		void start() override;

		void resume() { bPlaying = true; }
		void pause() { bPlaying = false; }
		
		float getProgress() const { return (float)currentTime.count() / (float)duration.count(); }
		bool isPlaying() const { return bPlaying; }
		bool isEnd() const { return bEnd; }

		bool getLoop() const { return bLoop; }
		void setLoop(bool t) { bLoop = t; }

		const std::chrono::milliseconds& getDuration() const { return duration; }
		std::string getDurationString() const { return toString(duration); }

	protected:
		bool updateCapture() override;
		bool updateIMU() override;
	private:
		static std::string toString(std::chrono::milliseconds duration);
		bool bLoop;
		bool bEnd;
		k4a::playback playback;
		k4a_record_configuration_t config;

		std::chrono::milliseconds duration;
		size_t totalFrame;

	};

	using PlaybackSettings = Playback::Settings;

};

