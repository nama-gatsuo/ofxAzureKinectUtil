#pragma once
#include "ofMain.h"
#include "ofxAzureKinectUtil.h"

class ofApp : public ofBaseApp{
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	void exit();
private:
	ofxAzureKinectUtil::Playback kinect;
	ofEasyCam cam;
};
