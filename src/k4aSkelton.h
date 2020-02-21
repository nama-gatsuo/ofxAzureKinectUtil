#pragma once
#include "k4aTypes.h"
#include "ofGraphics.h"
#include "of3dGraphics.h"
#include "of3dUtils.h"
#include "ofVectorMath.h"

namespace ofxAzureKinectUtil {
	class Skelton {
	public:
		static void drawGizmos(const k4abt_skeleton_t& skeleton) {
			// Draw joints.
			for (int i = 0; i < K4ABT_JOINT_COUNT; ++i) {
				auto joint = skeleton.joints[i];
				ofPushMatrix();
				{
					glm::mat4 transform = glm::translate(toGlm(joint.position)) * glm::toMat4(toGlm(joint.orientation));
					ofMultMatrix(transform);
					
					ofDrawBox(10.f);
					ofDrawAxis(30.0f);
				}
				ofPopMatrix();
			}

		}

		static void drawLines(const k4abt_skeleton_t& skeleton) {
			// Draw connections.
			ofMesh skeletonMesh;
			skeletonMesh.setMode(OF_PRIMITIVE_LINES);
			auto& vertices = skeletonMesh.getVertices();
			vertices.resize(100);
			int vdx = 0;

			// Spine.
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_NAVEL].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_NAVEL].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HEAD].position);

			// Head.
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HEAD].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NOSE].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NOSE].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EAR_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NOSE].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EAR_RIGHT].position);

			// Left Leg.
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position);

			// Right leg.
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position);

			// Left arm.
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HAND_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HAND_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_THUMB_LEFT].position);

			// Right arm.
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].position);

			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position);
			vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_THUMB_RIGHT].position);

			skeletonMesh.draw(OF_MESH_WIREFRAME);
		}

	};


}