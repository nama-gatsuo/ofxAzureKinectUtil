#include "ofApp.h"

void ofApp::setup() {

	ofSetVerticalSync(false);
	ofSetFrameRate(60);

	ofxAzureKinectUtil::PlaybackSettings s;
	s.fileName = "mkv/output.mkv";
	s.updateColor = true;
	s.updateDepth = false;
	s.updateIr = false;
	s.updatePointCloud = true;

	//kinect = std::make_shared<ofxAzureKinectUtil::Device>();
	kinect.open(s);
	kinect.setLoop(false);
	kinect.start();

}

void ofApp::update() {
	kinect.update();
}

void ofApp::draw() {

	if (!kinect.getColorTex().isAllocated()) {
		return;
	}

	cam.begin();

	ofEnableDepthTest();
	ofScale(1, -1, -1);
	ofMultMatrix(glm::toMat4(kinect.getOrientation()));
	kinect.getColorTex().bind();
	kinect.getPointCloud().draw();
	kinect.getDepthTex().unbind();
	ofDisableDepthTest();

	cam.end();

	std::string s = "fps:\t" + ofToString(ofGetFrameRate());
	s += "\nduration:\t" + kinect.getDurationString();
	s += "\nprogress:\t" + ofToString(kinect.getProgress());
	s += "\nframeCount:\t" + ofToString(kinect.getCurrentFrameCount());
	s += "\n[space]\t" + std::string(kinect.isPlaying() ? "pause" : "resume");
	s += "\n[s]\t(re)start";
	ofDrawBitmapStringHighlight(s, 12, ofGetHeight() - 120);

	ofDrawRectangle(0, ofGetHeight() - 24, ofGetWidth() * kinect.getProgress(), 24);
	
}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		if (kinect.isPlaying()) kinect.pause();
		else kinect.resume();
	} else if (key == 's') {
		kinect.start();
	}

}

void ofApp::exit() {
	kinect.close();
}
