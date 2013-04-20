#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOsc.h"
#include "ofxUI.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
	void keyPressed(int key);
	
	void sendOsc();
	
	ofVideoGrabber cam;	
	ofxCv::RunningBackground runningBackground;
	ofImage thresholded;
	ofxCv::ContourFinder contourFinder;
	ofPolyline biggest, resampled;
	vector<float> curvature;
	vector<int> peaks;
	vector<ofVec2f> fingers;
	float area;
	ofVec2f centroid;
	ofxOscSender osc;
	
	ofxUICanvas* gui;
	float threshold, smoothing, sampleOffset, peakAngleCutoff, peakNeighborDistance;
	bool clearBackground;
};
