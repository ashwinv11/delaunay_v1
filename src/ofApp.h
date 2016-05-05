#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxKinect.h"
#include "ofxDelaunay.h"
#include "ofxPostProcessing.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"

#define PORT 12001
#define NUM_MSG_STRINGS 20


class ofApp : public ofBaseApp{
public:
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    void audioIn(float * input, int bufferSize, int nChannels);
  
  void exit();
  
  ofxKinect kinect;
  
  ofEasyCam cam;
  
  ofxPostProcessing postFx;
  
  int pot1;
  int pot2;
  int newpot1;
  int newpot2;
  
  int topright;
  int topleft;
  float newnoise;
  float newalpha;
  
  // gui
  bool showGui;
  ofxPanel gui;
  ofxSlider<int> colorAlpha;
  //ofxSlider<float> noiseAmount;
  float noiseAmount;
  ofxToggle useRealColors;
  ofxSlider<int> pointSkip;
  ofParameter<int> nearThreshold;
  ofParameter<int> farThreshold;
  
  // meshes
  
  ofMesh convertedMesh;
  ofMesh wireframeMesh;
  
  ofxDelaunay del;
  
  ofImage blob;
  
  ofxCvGrayscaleImage grayImage; // grayscale depth image
  ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
  ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
  ofxCvContourFinder contourFinder;
  bool bThreshWithOpenCV;
  
  ofParameter<bool> stepOffset;
  int angle;
  
  ofSoundPlayer soundPlayer;
  
  vector <float> left;
		vector <float> right;
		vector <float> volHistory;
		
		int 	bufferCounter;
		int 	drawCounter;
		
		float smoothedVol;
		float scaledVol;
		
		ofSoundStream soundStream;
  
  float threshold;
  float minimumThreshold;
  float decayRate;
  
private:
		ofxOscReceiver	 receiver;
  
		int	current_msg_string;
		string	 msg_strings[NUM_MSG_STRINGS];
		float	timers[NUM_MSG_STRINGS];
  
};
