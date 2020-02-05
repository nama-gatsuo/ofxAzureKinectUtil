#include "ofApp.h"

void ofApp::setup() {
	
	ofSetVerticalSync(false);
	ofSetFrameRate(60);

	ofxAzureKinectUtil::DeviceSettings s;
	s.updateColor = true;
	s.updateDepth = true;
	s.updateIr = false;
	s.updatePointCloud = false;

	kinect = std::make_shared<ofxAzureKinectUtil::Device>();
	kinect->open(s);
	kinect->start();

	

}

void ofApp::update() {
	kinect->update();
}

void ofApp::draw() {
	kinect->getColorTex().draw(0, 0);
	ofDrawBitmapStringHighlight("fps: " + ofToString(ofGetFrameRate()), 12, 16);
}

void ofApp::keyPressed(int key) {
	
}

void ofApp::exit() {
	kinect->close();
}
