#pragma once

#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "aiMesh.h"
#include "aiScene.h"
#include "ofxMiniGui.h"

inline float RandomGaussian(float mean, float stddev) {
  float r1=ofRandom(1), r2=ofRandom(1);
  float val = sqrtf(-2 * logf(r1)) * cos(TWO_PI * r2);
  val = stddev * val + mean;
  return val;
}

class HandPose {
private:
	vector<string> names;
	vector<float> minValues, maxValues, values;
	void addDof(string name, float min, float max) {
		names.push_back(name);
		minValues.push_back(min);
		maxValues.push_back(max);
		values.push_back(0);
	}
public:
	HandPose() {
		// thumb
		addDof("Finger-1-1_R.x", -40, 40);
		addDof("Finger-1-1_R.y", -28, 16);
		addDof("Finger-1-1_R.z", -2, 10);
		addDof("Finger-1-2_R.y", -8, 10);
		addDof("Finger-1-3_R.y", -20, 20);
		
		// fingers
		for(int i = 2; i <= 5; i++) {
			addDof("Finger-" + ofToString(i) + "-1_R.y", -15, 10);
			addDof("Finger-" + ofToString(i) + "-1_R.z", -60, 30);
			addDof("Finger-" + ofToString(i) + "-2_R.z", -90, 5);
			addDof("Finger-" + ofToString(i) + "-3_R.z", -90, 5);
		}
	}
	int size() {
		return names.size();
	}
	string getName(int i) {
		return names[i];
	}
	float& getMin(int i) {
		return minValues[i];
	}
	float& getMax(int i) {
		return maxValues[i];
	}
	float& getValue(int i) {
		return values[i];
	}
	void save(string filename) {
		ofFile file(filename, ofFile::WriteOnly);
		for(int i = 0; i < size(); i++) {
			file << getValue(i) << endl;
		}
	}
	void load(string filename) {
		ofFile file(filename, ofFile::ReadOnly);
		for(int i = 0; i < size(); i++) {
			file >> getValue(i);
		}
	}
	void randomDeviation(float stddev) {
		for(int i = 0; i < size(); i++) {
			float range = maxValues[i] - minValues[i];
			float curWidth = range * stddev;
			values[i] = ofClamp(RandomGaussian(values[i], curWidth), minValues[i], maxValues[i]);
		}
	}
	void randomDeviation(map<string, float>& stddev) {
		for(int i = 0; i < size(); i++) {
			string& name = names[i];
			float curstddev = stddev[name];
			float range = maxValues[i] - minValues[i];
			float curWidth = range * curstddev;
			values[i] = ofClamp(RandomGaussian(values[i], curWidth), minValues[i], maxValues[i]);
		}
	}
};

aiMatrix4x4 toAi(ofMatrix4x4 ofMat);
ofMatrix4x4 toOf(aiMatrix4x4 aiMat);

inline bool isHand(string name) {
	return (name.find("Finger") != string::npos ||
	 name.find("Wrist") != string::npos ||
	 name.find("Palm") != string::npos) &&
	(name.find("_R") != string::npos);
}

inline bool isControllable(string name) {
	return (name.find("Finger") != string::npos) &&
	(name.find("_R") != string::npos);
}

typedef map<string, aiMatrix4x4> Pose;
class RiggedModel : public ofxAssimpModelLoader {
protected:
	void updatePose(int which = 0) {
		const aiMesh* mesh = modelMeshes[which].mesh;
		int n = mesh->mNumBones;
		vector<aiMatrix4x4> boneMatrices(n);
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			boneMatrices[a] = bone->mOffsetMatrix;
			const aiNode* node = scene->mRootNode->FindNode(bone->mName);
			while(node) {
				boneMatrices[a] = node->mTransformation * boneMatrices[a];
				node = node->mParent;
			}
			modelMeshes[which].hasChanged = true;
			modelMeshes[which].validCache = false;
		}
		
		modelMeshes[which].animatedPos.assign(modelMeshes[which].animatedPos.size(),0);
		if(mesh->HasNormals()){
			modelMeshes[which].animatedNorm.assign(modelMeshes[which].animatedNorm.size(),0);
		}
		
		// loop through all vertex weights of all bones
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			const aiMatrix4x4& posTrafo = boneMatrices[a];
			for(int b = 0; b < bone->mNumWeights; b++) {
				const aiVertexWeight& weight = bone->mWeights[b];
				size_t vertexId = weight.mVertexId;
				const aiVector3D& srcPos = mesh->mVertices[vertexId];
				modelMeshes[which].animatedPos[vertexId] += weight.mWeight * (posTrafo * srcPos);
			}
			if(mesh->HasNormals()) {
				// 3x3 matrix, contains the bone matrix without the translation, only with rotation and possibly scaling
				aiMatrix3x3 normTrafo = aiMatrix3x3( posTrafo);
				for( size_t b = 0; b < bone->mNumWeights; b++) {
					const aiVertexWeight& weight = bone->mWeights[b];
					size_t vertexId = weight.mVertexId;
					const aiVector3D& srcNorm = mesh->mNormals[vertexId];
					modelMeshes[which].animatedNorm[vertexId] += weight.mWeight * (normTrafo * srcNorm);
				}
			}
		}
		
		skeleton.clear();
		skeleton.setMode(OF_PRIMITIVE_LINES);
		vector<aiVector3D> avg(n);
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			const aiMatrix4x4& posTrafo = boneMatrices[a];
			for(int b = 0; b < bone->mNumWeights; b++) {
				const aiVertexWeight& weight = bone->mWeights[b];
				size_t vertexId = weight.mVertexId;
				avg[a] += modelMeshes[which].animatedPos[vertexId];
			}
			avg[a] /= bone->mNumWeights;
		}
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			const aiNode* node = scene->mRootNode->FindNode(bone->mName);
			if(node->mParent) {
				for(int b = 0; b < n; b++) {
					const aiBone* parent = mesh->mBones[b];
					if(parent->mName == node->mParent->mName) {
						skeleton.addVertex(ofVec3f(avg[a].x, avg[a].y, avg[a].z));
						skeleton.addVertex(ofVec3f(avg[b].x, avg[b].y, avg[b].z));
					}
				}
			}
		}
		
		int m = modelMeshes[which].animatedPos.size();
		vector< vector<int> > vertexBones(m);
		
		influence.clear();
		influence.setMode(OF_PRIMITIVE_LINES);
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			const aiMatrix4x4& posTrafo = boneMatrices[a];
			for(int b = 0; b < bone->mNumWeights; b++) {
				const aiVertexWeight& weight = bone->mWeights[b];
				int vertexId = weight.mVertexId;
				const aiVector3D& cur = modelMeshes[which].animatedPos[vertexId];
				influence.addVertex(ofVec3f(avg[a].x, avg[a].y, avg[a].z));
				influence.addVertex(ofVec3f(cur.x, cur.y, cur.z));
				vertexBones[vertexId].push_back(a);
			}
		}
		
		vector<int> vertexLabel(m);
		for(int i = 0; i < m; i++) {
			float bestDistance = 0;
			for(int j = 0; j < vertexBones[i].size(); j++) {
				int cur = vertexBones[i][j];
				const aiVector3D& joint = avg[cur];
				const aiVector3D& vertex = modelMeshes[which].animatedPos[i];
				float dx = joint.x - vertex.x;
				float dy = joint.y - vertex.y;
				float dz = joint.z - vertex.z;
				float distance = dx * dx + dy * dy + dz * dz;
				if(j == 0 || distance < bestDistance) {
					vertexLabel[i] = 255 - cur;
					bestDistance = distance;
				}
			}
		}
		
		maskedModel.clear();
		maskedModel.setMode(OF_PRIMITIVE_TRIANGLES);
		maskedCenter.set(0);
		
		int maskedTotal = 0;
		vector<bool> validVertices(modelMeshes[which].animatedPos.size());
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			string name = bone->mName.data;
			if(isHand(name)) {
				for(int b = 0; b < bone->mNumWeights; b++) {
					const aiVertexWeight& weight = bone->mWeights[b];
					int vertexId = weight.mVertexId;
					validVertices[vertexId] = true;
					const aiVector3D& cur = modelMeshes[which].animatedPos[vertexId];
					ofVec3f pos(cur.x, cur.y, cur.z);
					maskedCenter += pos;
					maskedTotal++;
				}
			}
		}
		maskedCenter /= maskedTotal;
				
		for(int i = 0; i < m; i++) {
			const aiVector3D& cur = modelMeshes[which].animatedPos[i];
			maskedModel.addVertex(ofVec3f(cur.x, cur.y, cur.z));
			const aiVector3D& norm = modelMeshes[which].animatedNorm[i];
			maskedModel.addNormal(ofVec3f(norm.x, norm.y, norm.z));
			maskedModel.addColor(ofColor(vertexLabel[i]));
		}
		
		for(int i = 0; i < modelMeshes[which].indices.size(); i+= 3) {
			ofIndexType i0 = modelMeshes[which].indices[i + 0];
			ofIndexType i1 = modelMeshes[which].indices[i + 1];
			ofIndexType i2 = modelMeshes[which].indices[i + 2];
			if(validVertices[i0] || validVertices[i1] || validVertices[i2]) {
				maskedModel.addIndex(i0);
				maskedModel.addIndex(i1);
				maskedModel.addIndex(i2);
			}
		}
	}
public:
	int getBoneCount(int which = 0) {
		return modelMeshes[which].mesh->mNumBones;
	}
	const aiBone* getBone(int i, int which = 0) {
		return modelMeshes[which].mesh->mBones[i];
	}
	Pose getPose(int which = 0) {
		const aiMesh* mesh = modelMeshes[which].mesh;	
		int n = mesh->mNumBones;
		Pose pose;
		for(int a = 0; a < n; a++) {
			const aiBone* bone = mesh->mBones[a];
			aiNode* node = scene->mRootNode->FindNode(bone->mName);
			pose[node->mName.data] = node->mTransformation;
		}
		return pose;
	}
	void setPose(Pose& pose, int which = 0) {
		// load the pose
		for(Pose::iterator i = pose.begin(); i != pose.end(); i++) {
			const string& name = i->first;
			const aiMatrix4x4& mat = i->second;
			scene->mRootNode->FindNode(name)->mTransformation = mat;
		}
		updatePose(which);
		updateGLResources();
	}
	void drawSkeleton() {
		ofPushMatrix();
		//ofTranslate(pos);
		//ofRotate(180, 0, 0, 1);
		ofTranslate(-scene_center.x, -scene_center.y, scene_center.z);
		ofScale(250, 250, 250);
		for(int i = 0; i < (int) rotAngle.size(); i++){
			ofRotate(rotAngle[i], rotAxis[i].x, rotAxis[i].y, rotAxis[i].z);
		}
		ofScale(scale.x, scale.y, scale.z);
	
		ofRotateX(-90);
		ofRotateY(-90);
		ofTranslate(-maskedCenter);
		
		ofSetColor(255);
		//for(int i = 0; i < 64; i++) {
			maskedModel.draw();
		//}
		//maskedModel.drawWireframe();
		
		ofPopMatrix();
	}
	
	ofMesh skeleton;
	ofMesh influence;
	ofVboMesh maskedModel;
	ofVec3f maskedCenter;
};

class testApp : public ofBaseApp{
	
public:
	void randomPose();
	void updateModel();
	void updatePoseFromGui();
	void updateGuiFromPose();
	void updateDifference();
	
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
	RiggedModel model;
	Pose bindPose;
	ofEasyCam easyCam;
	ofxMiniGui::Gui gui;
	
	ofFbo fbo;
	HandPose handPose, bestHandPose;
	ofImage reference;
	
	int iterations;
	ofImage best;
	float rating;
	int bestDifference;
	vector<int> labelDifference, labelTotal;
	map<string, float> labelRating;
};
