#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup(){
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();
	kinect.initFaceSource();

}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();

}

//--------------------------------------------------------------
void ofApp::draw(){


	kinect.getColorSource()->draw(0, 0, ofGetWidth(), ofGetHeight());
	ofMesh mesh;
	auto &verts = kinect.getFaceSource()->imagePoints;
	for (auto &v : verts) {
		mesh.addVertex(ofVec3f(v * 0.5));
	}
	mesh.draw(OF_MESH_POINTS);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
