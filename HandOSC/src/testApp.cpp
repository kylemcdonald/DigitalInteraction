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
			if(abs(curIndex - indices[j]) < peakArea) {
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
			centroid = toOf(contourFinder.getCentroid(maxAreaIndex));
			biggest = contourFinder.getPolyline(maxAreaIndex);
			resampled = biggest.getResampledBySpacing(1);
			resampled = resampled.getSmoothed(6);
			
			int offset = 60, peakCutoff = 30, peakArea = 60;
			curvature = buildContourAnalysis(resampled, offset);
			peaks = findPeaks(curvature, peakCutoff, peakArea);
			fingers.clear();
			for(int i = 0; i < peaks.size(); i++) {
				ofVec2f finger = resampled[peaks[i]];
				if(finger.y < ofGetHeight() - 8) {
					fingers.push_back(finger);
				}
			}
			
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
	
	ofSetLineWidth(3);
	ofSetColor(255);
	biggest.draw();
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