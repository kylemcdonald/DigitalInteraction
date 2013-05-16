#include "testApp.h"

using namespace ofxCv;
using namespace cv;

vector<float> buildContourAnalysis(ofPolyline& polyline, int offset) {
	int n = polyline.size();
	if(offset > n) {
		offset = n;
	}
	vector<float> curvature(n);
	for(int i = 0; i < n; i++) {
		int left = i - offset;
		if(left < 0) {
			left += n;
		}
		int right = i + offset;
		if(right >= n) {
			right -= n;
		}
		ofVec2f a = polyline[left], b = polyline[i], c = polyline[right];
		a -= b;
		c -= b;
		float angle = a.angle(c);
		curvature[i] = -(angle > 0 ? angle - 180 : angle + 180);
	}
	return curvature;
}

vector<int> findPeaks(vector<float>& values, float cutoff, int peakArea) {
	vector< pair<float, int> > peaks;
	int n = values.size();
	for(int i = 1; i < n - 1; i++) {
		if(values[i] > cutoff) {
			peaks.push_back(pair<float, int>(-values[i], i));
		}
	}
	ofSort(peaks);
	vector<int> indices;
	for(int i = 0; i < peaks.size(); i++) {
		int curIndex = peaks[i].second;
		bool hasNeighbor = false;
		for(int j = 0; j < indices.size(); j++) {
			if(abs(curIndex - indices[j]) < peakArea || abs((curIndex + n) - indices[j]) < peakArea) {
				hasNeighbor = true;
				break;
			}
		}
		if(!hasNeighbor) {
			indices.push_back(curIndex);
		}
	}
	return indices;
}

void testApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(120);
	cam.initGrabber(640, 480);
	contourFinder.setMinAreaRadius(10);
	contourFinder.setMaxAreaRadius(400);
	
	ofxXmlSettings xml;
	xml.loadFile("settings.xml");
	string host = xml.getValue("host", "localhost");
	int port = xml.getValue("port", 8000);
	osc.setup(host, port);
	
	threshold = 64;
	smoothing = 10;
	sampleOffset = 60;
	peakAngleCutoff = 45;
	peakNeighborDistance = 60;
	clearBackground = false;
	
	gui = new ofxUICanvas();
	gui->addLabel("Hand OSC");
	gui->addSpacer();
	gui->addSlider("Threshold", 0.0, 255.0, &threshold);
	gui->addSlider("Smoothing", 0.0, 20, &smoothing);
	gui->addSlider("Sample offset", 0.0, 100, &sampleOffset);
	gui->addSlider("Peak angle cutoff", 0, 90, &peakAngleCutoff);
	gui->addSlider("Peak neighbor distance", 0, 100, &peakNeighborDistance);
	gui->addSpacer();
	gui->addLabelButton("Clear background", &clearBackground);
	gui->autoSizeToFitWidgets();
}

void testApp::update() {
	if(clearBackground) {
		runningBackground.reset();
	}
	cam.update();
	if(cam.isFrameNew()) {
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
			area = maxArea;
			centroid = toOf(contourFinder.getCentroid(maxAreaIndex));
			biggest = contourFinder.getPolyline(maxAreaIndex);
			resampled = biggest.getResampledBySpacing(1);
			resampled = resampled.getSmoothed(smoothing);
			
			int padding = 8;
			curvature = buildContourAnalysis(resampled, sampleOffset);
			peaks = findPeaks(curvature, peakAngleCutoff, peakNeighborDistance);
			fingers.clear();
			for(int i = 0; i < peaks.size(); i++) {
				ofVec2f finger = resampled[peaks[i]];
				if(finger.y < ofGetHeight() - padding) {
					fingers.push_back(finger);
				}
			}
			
			sendOsc();
		}
	}
}

void testApp::sendOsc() {
	ofxOscMessage handSize;
	handSize.setAddress("/hand/size");
	handSize.addFloatArg(sqrtf(area));
	osc.sendMessage(handSize);
	
	ofxOscMessage handPosition;
	handPosition.setAddress("/hand/position");
	handPosition.addFloatArg(centroid.x);
	handPosition.addFloatArg(centroid.y);
	osc.sendMessage(handPosition);
	
	for(int i = 0; i < fingers.size(); i++) {
		ofxOscMessage fingerPosition;
		fingerPosition.setAddress("/hand/finger/" + ofToString(i));
		fingerPosition.addFloatArg(fingers[i].x);
		fingerPosition.addFloatArg(fingers[i].y);
		osc.sendMessage(fingerPosition);
	}
}

void testApp::draw() {
	ofSetColor(255);
	cam.draw(0, 0);
	
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofSetColor(255, 64);
	thresholded.draw(0, 0);
	
	ofEnableBlendMode(OF_BLENDMODE_ALPHA);
	ofSetLineWidth(3);
	ofSetColor(255);
	resampled.draw();
	ofSetColor(magentaPrint);
	ofNoFill();
	for(int i = 0; i < fingers.size(); i++) {
		ofLine(centroid, fingers[i]);
	}
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		runningBackground.reset();
	}
}