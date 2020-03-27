#include "ofApp.h"

void ofApp::setup() {

	ofxAzureKinectUtil::DeviceSettings s;
	s.updateColor = true;
	s.updateDepth = true;
	s.updateIr = false;
	s.updatePointCloud = true;

	kinect.open(s);

}

void ofApp::update() {
	if (kinect.isStreaming()) kinect.update();
}

void ofApp::draw() {

	if (kinect.getColorTex().bAllocated()) {
		cam.begin();
		ofPushMatrix();
		ofRotateX(180);
		ofMultMatrix(glm::toMat4(kinect.getOrientation()));

		ofEnableDepthTest();
		kinect.getColorTex().bind();
		kinect.getPointCloud().draw();
		kinect.getDepthTex().unbind();
		ofDisableDepthTest();

		ofPopMatrix();
		cam.end();

		ofDrawBitmapString("space key: toggle kinnect", 12, 16);
		ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), 12, 32);

	} else {
		ofDrawBitmapString("space key: toggle kinnect", 12, 16);
	}

}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		if (!kinect.isStreaming()) kinect.start();
		else kinect.stop();
	}
}

void ofApp::exit() {
	kinect.close();
}
