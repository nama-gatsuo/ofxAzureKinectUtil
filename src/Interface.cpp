#include "Interface.h"

namespace ofxAzureKinectUtil {

	bool Interface::start() {
		if (isUseColor) {
			// Create transformation.
			transformation = k4a::transformation(this->calibration);
		}

		if (isUseBodies) {
			// Create tracker.
			k4abt_tracker_create(&calibration, trackerConfig, &bodyTracker);
		}

		if (isUseDepth || isUsePointCloud) {
			// Create ray texture for mapping depth texture
			createRayTex();
		}

		return true;
	}

	bool Interface::stop() {

		transformation.destroy();

		if (this->isUseBodies) {
			k4abt_tracker_shutdown(bodyTracker);
			k4abt_tracker_destroy(bodyTracker);
			bodyTracker = nullptr;
		}
		
		return true;
	}

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

	bool Interface::createRayTex() {

		const k4a_calibration_camera_t& calib = calibration.color_camera_calibration;
		k4a_calibration_type_t type = K4A_CALIBRATION_TYPE_COLOR;

		const glm::ivec2 res(calib.resolution_width, calib.resolution_height);

		try {
			rayImg = k4a::image::create(
				K4A_IMAGE_FORMAT_CUSTOM,
				res.x, res.y,
				res.x * static_cast<int>(sizeof(k4a_float2_t)));
		} catch (const k4a::error& e) {
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		auto imgData = reinterpret_cast<k4a_float2_t*>(rayImg.get_buffer());

		k4a_float2_t p;
		k4a_float3_t ray;
		int idx = 0;
		for (int y = 0; y < res.y; ++y) {
			p.xy.y = static_cast<float>(y);

			for (int x = 0; x < res.x; ++x) {
				p.xy.x = static_cast<float>(x);

				if (calibration.convert_2d_to_3d(p, 1.f, type, type, &ray)) {
					imgData[idx].xy.x = ray.xyz.x;
					imgData[idx].xy.y = ray.xyz.y;
				} else {
					// The pixel is invalid.
					//ofLogNotice(__FUNCTION__) << "Pixel " << depthToWorldData[idx].xy.x << ", " << depthToWorldData[idx].xy.y << " is invalid";
					imgData[idx].xy.x = 0;
					imgData[idx].xy.y = 0;
				}

				++idx;
			}
		}

		ofFloatPixels pix;
		pix.setFromPixels(reinterpret_cast<float*>(imgData), res.x, res.y, 2);

		rayTex.allocate(res.x, res.y, GL_RG32F);
		rayTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
		rayTex.loadData(pix);

		return true;
	}

	void Interface::threadedFunction() {
		bool r = true;
		while (request.receive(r)) {
			FrameData newFd;
			
			updateCapture();

			
			if (isUseDepth) {
				k4a::image& img = this->capture.get_depth_image();
				if (img) {
					const glm::ivec2 res(img.get_width_pixels(), img.get_height_pixels());
					newFd.depthPix.allocate(res.x, res.y, 1);
					newFd.depthPix.setFromPixels(reinterpret_cast<uint16_t*>(img.get_buffer()), res.x, res.y, 1);
				}
				img.reset();
			}

			if (isUseColor) {
				k4a::image& img = this->capture.get_color_image();
				if (img) {
					const glm::ivec2 res(img.get_width_pixels(), img.get_height_pixels());
					newFd.colorPix.allocate(res.x, res.y, OF_PIXELS_BGRA);
					newFd.colorPix.setFromPixels(reinterpret_cast<uint8_t*>(img.get_buffer()), res.x, res.y, 4);
				}
				img.reset();
			}

			if (isUseIR) {
				k4a::image& img = this->capture.get_ir_image();
				if (img) {
					const glm::ivec2 res(img.get_width_pixels(), img.get_height_pixels());
					newFd.irPix.allocate(res.x, res.y, 1);
					newFd.irPix.setFromPixels(reinterpret_cast<uint16_t*>(img.get_buffer()), res.x, res.y, 1);
				}
				img.reset();
			}

			if (this->isUseBodies) {
				k4a_wait_result_t enqueueResult = k4abt_tracker_enqueue_capture(bodyTracker, capture.handle(), K4A_WAIT_INFINITE);
				if (enqueueResult == K4A_WAIT_RESULT_FAILED) {
					ofLogError(__FUNCTION__) << "Failed adding capture to tracker process queue!";
				} else {
					k4abt_frame_t bodyFrame = nullptr;
					k4a_wait_result_t popResult = k4abt_tracker_pop_result(bodyTracker, &bodyFrame, K4A_WAIT_INFINITE);
					
					if (popResult == K4A_WAIT_RESULT_SUCCEEDED) {
						// Probe for a body index map image.
						k4a::image img = k4abt_frame_get_body_index_map(bodyFrame);
						const glm::ivec2 res(img.get_width_pixels(), img.get_height_pixels());
						
						newFd.bodyIndexPix.allocate(res.x, res.y, 1);
						newFd.bodyIndexPix.setFromPixels(reinterpret_cast<uint8_t*>(img.get_buffer()), res.x, res.y, 1);
						
						img.reset();

						// Detect bodies
						size_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);

						newFd.bodySkeletons.resize(numBodies);
						newFd.bodyIDs.resize(numBodies);
						for (size_t i = 0; i < numBodies; i++) {
							k4abt_skeleton_t skeleton;
							k4abt_frame_get_body_skeleton(bodyFrame, i, &skeleton);
							newFd.bodySkeletons[i] = skeleton;
							uint32_t id = k4abt_frame_get_body_id(bodyFrame, i);
							newFd.bodyIDs[i] = id;
						}

						// Release body frame once we're finished.
						k4abt_frame_release(bodyFrame);
					}
				}
			}


		}


	}
}
