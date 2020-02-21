#include "ofApp.h"

void ofApp::setup() {

	ofxAzureKinectUtil::DeviceSettings s;
	
	s.updateColor = true;
	s.updateDepth = true;
	s.updateIr = false;
	s.updatePointCloud = true;
	s.updatePolygonMesh = false;
	s.updateBodies = true;

	kinect.open(s);
	kinect.start();

	panel.setup(kinect.getParameters());

}

void ofApp::update() {
	kinect.update();
}

void ofApp::draw() {

	if (!kinect.getColorTex().bAllocated()) return;

	//kinect.getBodyIndexTex().draw(0, 0);

	cam.begin();
	ofPushMatrix();

	ofMultMatrix(glm::toMat4(kinect.getOrientation()));
	ofRotateX(180);
	ofEnableDepthTest();
	
	for (auto& s : kinect.getBodySkeletons()) {
		ofxAzureKinectUtil::Skelton::drawGizmos(s);
		ofxAzureKinectUtil::Skelton::drawLines(s);
	}

	kinect.getColorTex().bind();
	kinect.getPointCloud().draw(OF_MESH_POINTS);
	kinect.getColorTex().unbind();

	ofDisableDepthTest();
	
	ofPopMatrix();
	cam.end();
	
	panel.draw();

	ofDrawBitmapStringHighlight("fps: " + ofToString(ofGetFrameRate()), 12, 16);
}

void ofApp::keyPressed(int key) {
	
}

void ofApp::exit() {
	kinect.close();
}
