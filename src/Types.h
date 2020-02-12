#pragma once
#include <k4a/k4atypes.h>
#include <k4abttypes.h>
#include "ofVectorMath.h"

namespace ofxAzureKinectUtil {
	using DepthMode = k4a_depth_mode_t;
	using ColorResolution = k4a_color_resolution_t ;
	using ImageFormat = k4a_image_format_t ;
	using Fps = k4a_fps_t;
	using SensorOrientation = k4abt_sensor_orientation_t;

	inline const glm::vec2& toGlm(const k4a_float2_t& v) {
		return *reinterpret_cast<const glm::vec2*>(&v);
	}

	inline const glm::vec3& toGlm(const k4a_float3_t& v) {
		return *reinterpret_cast<const glm::vec3*>(&v);
	}

	inline const glm::quat toGlm(const k4a_quaternion_t& q) {
		return glm::quat(q.v[0], q.v[1], q.v[2], q.v[3]);
	}

	struct IMU {
		float temperature; // Celsius
		glm::vec3 acc; // meter/s^2
		glm::vec3 gyro; // radians/s
	};
}

