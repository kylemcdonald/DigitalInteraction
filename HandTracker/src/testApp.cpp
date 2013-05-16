#include "testApp.h"

using namespace ofxMiniGui;

aiMatrix4x4 toAi(ofMatrix4x4 ofMat) {
	aiMatrix4x4 aiMat;
	aiMat.a1 = ofMat(0, 0); aiMat.a2 = ofMat(0, 1); aiMat.a3 = ofMat(0, 2); aiMat.a4 = ofMat(0, 3);
	aiMat.b1 = ofMat(1, 0); aiMat.b2 = ofMat(1, 1); aiMat.b3 = ofMat(1, 2); aiMat.b4 = ofMat(1, 3);
	aiMat.c1 = ofMat(2, 0); aiMat.c2 = ofMat(2, 1); aiMat.c3 = ofMat(2, 2); aiMat.c4 = ofMat(2, 3);
	aiMat.d1 = ofMat(3, 0); aiMat.d2 = ofMat(3, 1); aiMat.d3 = ofMat(3, 2); aiMat.d4 = ofMat(3, 3);
	return aiMat;
}

ofMatrix4x4 toOf(aiMatrix4x4 aiMat) {
	ofMatrix4x4 ofMat;
	ofMat(0, 0) = aiMat.a1; ofMat(0, 1) = aiMat.a2; ofMat(0, 2) = aiMat.a3; ofMat(0, 3) = aiMat.a4;
	ofMat(1, 0) = aiMat.b1; ofMat(1, 1) = aiMat.b2; ofMat(1, 2) = aiMat.b3; ofMat(1, 3) = aiMat.b4;
	ofMat(2, 0) = aiMat.c1; ofMat(2, 1) = aiMat.c2; ofMat(2, 2) = aiMat.c3; ofMat(2, 3) = aiMat.c4;
	ofMat(3, 0) = aiMat.d1; ofMat(3, 1) = aiMat.d2; ofMat(3, 2) = aiMat.d3; ofMat(3, 3) = aiMat.d4;
	return ofMat;
}

void applyMatrix(const ofMatrix4x4& matrix) {
	glMultMatrixf((GLfloat*) matrix.getPtr());
}

ofMatrix4x4 randomRotation(float amt = 180) {
	ofQuaternion quat;
	ofMatrix4x4 mat;
	ofVec3f axis(ofRandomf(), ofRandomf(), ofRandomf());
	axis.normalize();
	quat.makeRotate(ofRandom(-amt, amt), axis);
	quat.get(mat);
	return mat;
}

void testApp::randomPose() {
	if(iterations > 100) {
		handPose = bestHandPose;
		iterations = 0;
	} else {
//		handPose.randomDeviation(rating);
		handPose.randomDeviation(labelRating);
		iterations++;
	}
	updateGuiFromPose();
}

bool find(string src, string target) {
	return src.find(target) != string::npos;
}

void testApp::setup(){
	//ofSetVerticalSync(true);
	//ofSetFrameRate(120);
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	reference.loadImage("three.png");
	reference.setImageType(OF_IMAGE_GRAYSCALE);
	reference.update();
	int side = 128;
	fbo.allocate(side, side);
	
	best.allocate(side, side, OF_IMAGE_GRAYSCALE);
	bestDifference = side * side;
	rating = 1;
	iterations = 0;
	
	gui.setup();	
	
	ofDisableArbTex();
	if(model.loadModel("rigged-human.dae")){
		bindPose = model.getPose();
		for(int i =0; i < handPose.size(); i++) {
			gui.add(Slider(handPose.getName(i), handPose.getMin(i), handPose.getMax(i), 0));
		}
	}
	
	handPose.load("three.txt");
	handPose.randomDeviation(.2);
	updateGuiFromPose();
	
	updateModel();
}

void testApp::update(){	
	randomPose();
	updateModel();
}

void testApp::updateGuiFromPose() {
	for(int i = 0; i < handPose.size(); i++) {
		gui.set(handPose.getName(i), handPose.getValue(i));
	}
}

void testApp::updatePoseFromGui() {
	for(int i = 0; i < handPose.size(); i++) {
		handPose.getValue(i) = gui.get(handPose.getName(i));
	}
}

void testApp::updateModel() {
	updatePoseFromGui();
	
	Pose pose = bindPose;
	for(Pose::iterator i = pose.begin(); i != pose.end(); i++) {
		string name = i->first;
		
		float x = gui.exists(name + ".x") ? gui.get(name + ".x").getValue() : 0;
		float y = gui.exists(name + ".y") ? gui.get(name + ".y").getValue() : 0;
		float z = gui.exists(name + ".z") ? gui.get(name + ".z").getValue() : 0;
		
		aiMatrix4x4& bone = i->second;
		
		aiMatrix4x4 cur;
		ofMatrix4x4 mat;
		ofQuaternion quat(x, ofVec3f(1, 0, 0),
											y, ofVec3f(0, 1, 0),
											z, ofVec3f(0, 0, 1));
		quat.get(mat);
		cur = toAi(mat);
		
		bone *= cur;
	}
	model.setPose(pose);
}

void testApp::updateDifference() {
	ofPixels current;
	fbo.readToPixels(current);
	current.setImageType(OF_IMAGE_GRAYSCALE);
	int width = current.getWidth(), height = current.getHeight();
	int n = width * height, difference = 0;
	int boneCount = model.getBoneCount();
	labelDifference = vector<int>(boneCount);
	labelTotal = vector<int>(boneCount);
	unsigned char* referencePixels = reference.getPixels();
	unsigned char* currentPixels = current.getPixels();
	for(int i = 0; i < n; i++) {
		if(currentPixels[i] > 0) {
			int label = 255 - currentPixels[i];
			if(referencePixels[i] == 0) {
				labelDifference[label]++;
				difference++;
			}
			labelTotal[label]++;
		}
	}
	rating = (float) difference / n;
	
	// pose iteration order is different than bone order
	for(int i = 0; i < boneCount; i++) {
		const aiBone* bone = model.getBone(i);
		if(labelTotal[i] > 0) {
			string name = bone->mName.data;
			float curRating = (float) labelDifference[i] / labelTotal[i];
			labelRating[name + ".x"] = curRating;
			labelRating[name + ".y"] = curRating;
			labelRating[name + ".z"] = curRating;
		}
	}
	if(difference < bestDifference) {
		best = current;
		best.update();
		bestDifference = difference;
		bestHandPose = handPose;
		cout << ofGetElapsedTimef() << "s " << (100. * rating) << "%" << endl;
	}
}

void testApp::draw(){
	ofBackground(128);
	
	fbo.begin();
	ofClear(0, 255);
	ofSetupScreenOrtho(fbo.getWidth(), fbo.getHeight(), OF_ORIENTATION_DEFAULT, false, -1000, 1000);
	ofTranslate(fbo.getWidth() / 2, fbo.getHeight() / 2);
	ofScale(fbo.getWidth() / 512., fbo.getHeight() / 512.);
	ofEnableBlendMode(OF_BLENDMODE_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_FLAT); // important for not smoothing color labels
	model.drawSkeleton();
	fbo.end();
	
	updateDifference();
	
	glDisable(GL_DEPTH_TEST);
	
	ofSetColor(255);
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofSetColor(255, 128, 0);
	fbo.setAnchorPercent(.5, 1);
	fbo.draw(ofGetWidth() / 2, ofGetHeight() / 2);
	ofSetColor(0, 128, 255);
	reference.setAnchorPercent(.5, 1);
	reference.draw(ofGetWidth() / 2, ofGetHeight() / 2);
	            
	ofSetColor(255, 128, 0);
	best.setAnchorPercent(.5, 0);
	best.draw(ofGetWidth() / 2, ofGetHeight() / 2);
	ofSetColor(0, 128, 255);
	reference.setAnchorPercent(.5, 0);
	reference.draw(ofGetWidth() / 2, ofGetHeight() / 2);

	ofSetColor(255);
	ofPushMatrix();
	for(map<string, float>::iterator itr = labelRating.begin(); itr != labelRating.end(); itr++) {
		ofRect(0, 0, 0, 4, itr->second * 100);
		ofTranslate(4, 0);
	}
	ofPopMatrix();
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		randomPose();
		updateModel();
	}
	if(key == 's') {
		ofPixels pixels;
		fbo.readToPixels(pixels);
		ofSaveImage(pixels, "out.png");
		handPose.save("out.txt");
	}
}
