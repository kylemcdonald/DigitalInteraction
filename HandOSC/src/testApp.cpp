#include "testApp.h"

using namespace ofxCv;
using namespace cv;

void testApp::setup() {
	cam.initGrabber(640, 480);
	contourFinder.setMinAreaRadius(10);
	contourFinder.setMaxAreaRadius(400);
	osc.setup("localhost", 8000);
}

void testApp::update() {
	cam.update();
	if(cam.isFrameNew()) {
		int threshold = ofMap(mouseX, 0, ofGetWidth(), 0, 255);
		runningBackground.setThresholdValue(threshold);
		runningBackground.update(cam, thresholded);
		thresholded.update();
		contourFinder.findContours(thresholded);
		if(contourFinder.size() > 0) {
			float maxArea = 0;
			int maxAreaIndex = 0;
			for(int i = 0; i < contourFinder.size(); i++) {
				float curArea = contourFinder.getContourArea(i);
				if(curArea > maxArea) {
					maxArea = curArea;
					maxAreaIndex = i;
				}
			}
			biggest = contourFinder.getPolyline(maxAreaIndex);
			ofxOscMessage msg;
			msg.setAddress("/hand/size");
			msg.addFloatArg(sqrtf(maxArea));
			osc.sendMessage(msg);
		}
	}
}

void testApp::draw() {
	ofSetColor(255);
	cam.draw(0, 0);
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofSetColor(255, 64);
	thresholded.draw(0, 0);
	ofEnableBlendMode(OF_BLENDMODE_ALPHA);
	ofSetColor(255);
	biggest.draw();
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		runningBackground.reset();
	}
}