#include "Interface.h"

namespace ofxAzureKinectUtil {
	void Interface::update() {
		bool r = true;
		request.send(r);
		bool isFrameNew = false;

		while (response.tryReceive(fd)) {
			isFrameNew = true;
		}

		if (isFrameNew) {
			// update





		}

	}
	void Interface::threadedFunction() {
		bool r = true;
		while (request.receive(r)) {
			FrameData newFd;
			
			// Take time ...
			updateCapture();

			


		}


	}
}
