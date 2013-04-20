#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOsc.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
	void keyPressed(int key);
	
	ofVideoGrabber cam;	
	ofxCv::RunningBackground runningBackground;
	ofImage thresholded;
	ofxCv::ContourFinder contourFinder;
	ofPolyline biggest;
	ofxOscSender osc;
};
