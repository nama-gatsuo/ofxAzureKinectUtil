#include "ofApp.h"

void ofApp::setup() {
	
	ofSetVerticalSync(false);
	ofSetFrameRate(60);

	ofxAzureKinectUtil::DeviceSettings s;
	s.updateColor = true;
	s.updateDepth = true;
	s.updateIr = false;
	s.updatePointCloud = true;

	//kinect = std::make_shared<ofxAzureKinectUtil::Device>();
	kinect.open(s);
	kinect.start();

}

void ofApp::update() {
	kinect.update();
}

void ofApp::draw() {

	if (!kinect.getColorTex().bAllocated()) return;

	cam.begin();

	ofEnableDepthTest();
	kinect.getColorTex().bind();
	kinect.getPointCloud().draw();
	kinect.getDepthTex().unbind();
	ofDisableDepthTest();
	
	cam.end();
	
	//kinect.getDepthTex().draw(0, 0);
	
	ofDrawBitmapStringHighlight("fps: " + ofToString(ofGetFrameRate()), 12, 16);
}

void ofApp::keyPressed(int key) {
	
}

void ofApp::exit() {
	kinect.close();
}
