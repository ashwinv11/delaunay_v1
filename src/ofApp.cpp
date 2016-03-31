#include "ofApp.h"



//--------------------------------------------------------------
void ofApp::setup(){
  ofSetFrameRate(30);
  kinect.init();
  
  kinect.open();
  kinect.setRegistration(true);
  
  blob.allocate(640,480,OF_IMAGE_GRAYSCALE);
  
  
  // Gui
  gui.setup();
  gui.setPosition(ofPoint(10,10));
  //gui.add(noiseAmount.setup("Noise Amount", 0.0, 0.0,20.0));
  gui.add(pointSkip.setup("Point Skip", 1, 1,20));
  gui.add(useRealColors.setup("Real Colors", false));
  gui.add(colorAlpha.setup("Color Alpha", 0,0,255));
  gui.add(stepOffset.set("use step offset", true));
  gui.add(nearThreshold.set( "near threshold", 255, 0, 255 ));
  gui.add(farThreshold.set( "far threshold", 200, 0, 255 ));
  gui.loadFromFile("settings.xml");
  showGui = true;
  
  bThreshWithOpenCV = true;
  nearThreshold = 255;
  farThreshold = 200;
  kinect.setLed(ofxKinect::LED_OFF);
  angle = -10;
  kinect.setCameraTiltAngle(angle);
  
  // fx
  postFx.init(ofGetWidth(), ofGetHeight());
  
  postFx.createPass<BloomPass>();
  postFx.createPass<FxaaPass>();
  
  ofSetFrameRate(60);
  
  
  soundPlayer.loadSound("song.wav");
  soundPlayer.play();
  
  // 0 output channels,
  // 2 input channels
  // 44100 samples per second
  // 256 samples per buffer
  // 4 num buffers (latency)
  
  soundStream.printDeviceList();
  
  //if you want to set a different device id
  //soundStream.setDeviceID(0); //bear in mind the device id corresponds to all audio devices, including  input-only and output-only devices.
  
  int bufferSize = 256;
  
  
  left.assign(bufferSize, 0.0);
  right.assign(bufferSize, 0.0);
  volHistory.assign(400, 0.0);
  
  bufferCounter	= 0;
  drawCounter		= 0;
  smoothedVol     = 0.0;
  scaledVol		= 0.0;
  
  soundStream.setup(this, 0, 2, 44100, bufferSize, 4);
  
  decayRate = 0.05;
  minimumThreshold = 0.2;
  noiseAmount = 2.0;
  
}



//--------------------------------------------------------------
void ofApp::update(){
  
  kinect.update();
  
  // OLD EXAMPLE --------------------------------------------------------------
  
  // there is a new frame and we are connected
  if(kinect.isFrameNew()) {
    del.reset(); // new delaunay
    // load grayscale depth image from the kinect source
    grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    
    // we do two thresholds - one for the far plane and one for the near plane
    // we then do a cvAnd to get the pixels which are a union of the two thresholds
    if(bThreshWithOpenCV) {
      grayThreshNear = grayImage;
      grayThreshFar = grayImage;
      grayThreshNear.threshold(nearThreshold, true);
      grayThreshFar.threshold(farThreshold);
      cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
    } else {
      
      // or we do it ourselves - show people how they can work with the pixels
      unsigned char * pix = grayImage.getPixels();
      
      int numPixels = grayImage.getWidth() * grayImage.getHeight();
      for(int i = 0; i < numPixels; i++) {
        if(pix[i] < nearThreshold && pix[i] > farThreshold) {
          pix[i] = 255;
        } else {
          pix[i] = 0;
        }
      }
    }
    
    // update the cv images
    grayImage.flagImageChanged();
    
    // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
    // also, find holes is set to true so we will get interior contours as well....
    contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, true);
    
    // END OLD EXAMPLE --------------------------------------------------------------
    
    unsigned char* pix = new unsigned char[640*480];
    
    unsigned char* gpix = new unsigned char[640*480];
    
    for(int x=0;x<640;x+=1) {
      for(int y=0;y<480;y+=1) {
        float distance = kinect.getDistanceAt(x, y);
        
        int pIndex = x + y * 640;
        pix[pIndex] = 0;
        
        if(distance > 100 && distance < 1100) {
          pix[pIndex] = 255;
        }
        
      }
    }
    
    blob.setFromPixels(pix, 640, 480, OF_IMAGE_GRAYSCALE);
    
    int numPoints = 0;
    
    for(int x=0;x<640;x+=pointSkip*2) {
      for(int y=0;y<480;y+=pointSkip*2) {
        int pIndex = x + 640 * y;
        
        if(blob.getPixels()[pIndex]> 0) {
          ofVec3f wc = kinect.getWorldCoordinateAt(x, y);
          
          wc.x = x - 320.0;
          wc.y = y - 240.0;
          
          if(abs(wc.z) > 100 && abs(wc.z ) < 2000) {
            
            wc.z = -wc.z;
            
            wc.x += ofSignedNoise(wc.x,wc.z)*noiseAmount;
            wc.y += ofSignedNoise(wc.y,wc.z)*noiseAmount;
            
            
            wc.x = ofClamp(wc.x, -320,320);
            wc.y = ofClamp(wc.y, -240,240);
            
            del.addPoint(wc);
          }
          numPoints++;
        }
        
      }
    }
    
    
    if(numPoints >0)
      del.triangulate();
    
    for(int i=0;i<del.triangleMesh.getNumVertices();i++) {
      del.triangleMesh.addColor(ofColor(0,0,0));
    }
    
    for(int i=0;i<del.triangleMesh.getNumIndices()/3;i+=1) {
      ofVec3f v = del.triangleMesh.getVertex(del.triangleMesh.getIndex(i*3));
      
      v.x = ofClamp(v.x, -319,319);
      v.y = ofClamp(v.y, -239, 239);
      
      ofColor c = kinect.getColorAt(v.x+320.0, v.y+240.0);
      
      if(!useRealColors)
        c = ofColor(255); //fill color
      
      c.a = colorAlpha;
      
      del.triangleMesh.setColor(del.triangleMesh.getIndex(i*3),c);
      del.triangleMesh.setColor(del.triangleMesh.getIndex(i*3+1),c);
      del.triangleMesh.setColor(del.triangleMesh.getIndex(i*3+2),c);
    }
    
    
    convertedMesh.clear();
    wireframeMesh.clear();
    wireframeMesh.setMode(OF_PRIMITIVE_TRIANGLES);
    for(int i=0;i<del.triangleMesh.getNumIndices()/3;i+=1) {
      
      int indx1 = del.triangleMesh.getIndex(i*3);
      ofVec3f p1 = del.triangleMesh.getVertex(indx1);
      int indx2 = del.triangleMesh.getIndex(i*3+1);
      ofVec3f p2 = del.triangleMesh.getVertex(indx2);
      int indx3 = del.triangleMesh.getIndex(i*3+2);
      ofVec3f p3 = del.triangleMesh.getVertex(indx3);
      
      ofVec3f triangleCenter = (p1+p2+p3)/3.0;
      triangleCenter.x += 320;
      triangleCenter.y += 240;
      
      triangleCenter.x = floor(ofClamp(triangleCenter.x, 0,640));
      triangleCenter.y = floor(ofClamp(triangleCenter.y, 0, 480));
      
      int pixIndex = triangleCenter.x + triangleCenter.y * 640;
      if(pix[pixIndex] > 0) {
        
        convertedMesh.addVertex(p1);
        convertedMesh.addColor(del.triangleMesh.getColor(indx1));
        
        convertedMesh.addVertex(p2);
        convertedMesh.addColor(del.triangleMesh.getColor(indx2));
        
        convertedMesh.addVertex(p3);
        convertedMesh.addColor(del.triangleMesh.getColor(indx3));
        
        wireframeMesh.addVertex(p1);
        wireframeMesh.addVertex(p2);
        wireframeMesh.addVertex(p3);
      }
    }
    
    delete pix;
    delete gpix;
    
  }
}


//--------------------------------------------------------------
void ofApp::draw(){
  ofBackground(0);
  //glEnable(GL_DEPTH_TEST);
  
  ofPushMatrix();
  
  cam.begin();
  cam.setScale(1,-1,1);
  
  ofSetColor(255,0,0);
  ofTranslate(0, -80,1100);
  ofFill();
  
  postFx.begin();
  
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glShadeModel(GL_FLAT);
  glProvokingVertex(GL_FIRST_VERTEX_CONVENTION);
  convertedMesh.drawFaces();
  glShadeModel(GL_SMOOTH);
  glPopAttrib();
  
  if(useRealColors) {
    ofSetColor(30,30,30, 255);
  } else
    ofSetColor(255,0,0,255); //line color
  
  ofPushMatrix();
  ofTranslate(0, 0,0.5);
  wireframeMesh.drawWireframe();
  ofPopMatrix();
  cam.end();
  ofPopMatrix();
  
  postFx.end();
  
  if(showGui) {
    
    ofPushStyle();
    ofSetColor(255,255,255,255);
    gui.draw();
    ofPopStyle();
  }
  ofSetColor(255, 255, 255);
  
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
  
  if(key == ' ') {
    showGui = !showGui;
  }
  
  if(key == 's') {
    gui.saveToFile("settings.xml");
  }
  if(key == 'l') {
    gui.loadFromFile("settings.xml");
  }
  
  switch (key) {
    case '>':
    case '.':
      farThreshold ++;
      if (farThreshold > 255) farThreshold = 255;
      break;
      
    case '<':
    case ',':
      farThreshold --;
      if (farThreshold < 0) farThreshold = 0;
      break;
      
    case '+':
    case '=':
      nearThreshold ++;
      if (nearThreshold > 255) nearThreshold = 255;
      break;
      
    case '-':
      nearThreshold --;
      if (nearThreshold < 0) nearThreshold = 0;
      break;
      
    case OF_KEY_UP:
      angle++;
      if(angle>30) angle=30;
      kinect.setCameraTiltAngle(angle);
      break;
      
    case OF_KEY_DOWN:
      angle--;
      if(angle<-30) angle=-30;
      kinect.setCameraTiltAngle(angle);
      break;
      
      case 's':
        soundStream.start();
      break;
      
      case 'e':
        soundStream.stop();
      break;
      
  }
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

void ofApp::audioIn(float * input, int bufferSize, int nChannels) {
  // modified from audioInputExample
  threshold = minimumThreshold;
  float rms = 0.0;
  int numCounted = 0;
  
  for (int i = 0; i < bufferSize; i++){
    float leftSample = input[i * 2] * 0.5;
    float rightSample = input[i * 2 + 1] * 0.5;
    
    rms += leftSample * leftSample;
    rms += rightSample * rightSample;
    numCounted += 2;
  }
  
  rms /= (float)numCounted;
  rms = sqrt(rms);
  // rms is now calculated
  threshold = ofLerp(threshold, minimumThreshold, decayRate);
  
  if(rms > threshold) {
    // onset detected!
    threshold = rms;
    noiseAmount = 20;
  }
  else{
    while (noiseAmount >= 0)
      noiseAmount -= 0.000001;
      noiseAmount *= decayRate;
  }
}

void ofApp::exit() {
  kinect.close();
  gui.saveToFile("settings.xml");
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
  
}