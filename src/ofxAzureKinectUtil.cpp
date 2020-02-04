#include "ofxAzureKinectUtil.h"

namespace ofxAzureKinectUtil {

	const int32_t TIMEOUT_IN_MS = 1000;

	void Device::updateCapture() {
		try {
			if (!device.get_capture(&capture, std::chrono::milliseconds(TIMEOUT_IN_MS))) {
				ofLogWarning(__FUNCTION__) << "Timed out waiting for a capture for device " << this->index << ".";
				return;
			}
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return;
		}
	}

	void Playback::updateCapture() {
		try {
			playback.get_next_capture(&capture);
		} catch (const k4a::error & e) {
			ofLogError(__FUNCTION__) << e.what();
			return;
		}
	}

}