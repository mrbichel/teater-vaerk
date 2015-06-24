#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
    
    
    
    
    
    fboOut.allocate(1024, 768);
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.init();
    kinect2.open();
#endif
    
    colorImg.allocate(kinect.width, kinect.height);
    depthImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    grayBg.allocate(kinect.width, kinect.height);
    grayDiff.allocate(kinect.width, kinect.height);
    
    grayImage.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    
    bLearnBakground = true;
    threshold = 80;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    //angle = 0;
    //kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;
    
    syphonIn.setup();
    
    syphonIn.set("Screen 1", "Millumin");

    paramGroup.add(nearThreshold.set("Near Threshold", 0, 0, 255));
    paramGroup.add(farThreshold.set("Far Threshold", 0, 0, 255));
    paramGroup.add(minBlobSize.set("Min blob size", 0, 0, kinect.width*kinect.height/10));
    paramGroup.add(maxBlobSize.set("Max blob size", 0, 0, kinect.width*kinect.height/2));
    paramGroup.add(bLearnBakground.set("Learn bg", false));
    paramGroup.add(threshold.set("Threshold", 0, 0, 255));
    
    
    paramGroup.add(filterFc.set("filter fc", 0.05, 0, 1));
    //paramGroup.add(filterQ.set("filter q", 0, 0, 1));
    //paramGroup.add(filterPeakGain.set("filter peak gain", 0, 0, 1));
    
    paramGroup.add(scaleImage.set("scale img", 1, 0, 4));
    paramGroup.add(offsetImage.set("offset img", ofVec2f(0,0), ofVec2f(-400,-400), ofVec2f(400,400)));
    
    gui.setup(paramGroup);
    
    gui.loadFromFile("settings.xml");
    
    blobs.resize(1);
    
    for(int i=0; i<blobs.size(); i++) {
        blobs[i].setup();
    }
    
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofBackground(100, 100, 100);
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        
        //colorImg.setFromPixels(kinect.getPixelsRef());
        //grayImage = colorImg;
        
        grayThreshNear = depthImage;
        grayThreshFar = depthImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), depthImage.getCvImage(), NULL);
        
        if (bLearnBakground == true){
            grayBg = depthImage;		// the = sign copys the pixels from grayImage into grayBg (operator overloading)
            bLearnBakground = false;
        }
        // take the abs value of the difference between background and incoming and then threshold:
        
        grayDiff.absDiff(grayBg, depthImage);
        grayDiff.threshold(threshold);
            
            //cvAnd(depthImage.getCvImage(), grayImage.getCvImage(), grayImage.getCvImage());
        
            // mask colorimag with thresholded grayDepthImage
            // then do contourfind on masked image
            // find centroid of blob
            // map input image
            
            // also do background subtraction?
            
        
        
        // update the cv images
        grayImage.flagImageChanged();
        depthImage.flagImageChanged();

        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        
        contourFinder.findContours(grayDiff, minBlobSize, maxBlobSize, blobs.size(), false);
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
    
    for(int i=0; i<contourFinder.blobs.size();i++) {
        
        if(i < blobs.size()) {
            
            
            blobs[i].size.setFc(filterFc);
            //blobs[i].size.setQ(filterQ);
            //blobs[i].size.setPeakGain(filterPeakGain);
            
            blobs[i].pos.setFc(filterFc);
            //blobs[i].pos.setQ(filterQ);
            //blobs[i].pos.setPeakGain(filterPeakGain);
            
            blobs[i].size.update(ofVec2f(contourFinder.blobs[i].boundingRect.width, contourFinder.blobs[i].boundingRect.height));
        
            blobs[i].pos.update(contourFinder.blobs[i].centroid);
            
        }
        
    }
    
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    fboOut.begin();
    
    ofBackground(0,0,0);
    
    //grayDiff.draw(0, 0, fboOut.getWidth(), fboOut.getHeight());
    
    
    /// TODO: Draw media centered on blob / blobs
    //r
    
    ofScale(fboOut.getWidth()/kinect.width, fboOut.getHeight()/kinect.height);
    
    //contourFinder.draw(0, 0);
    
    for(int i=0; i<blobs.size();i++) {
        
        ofPushMatrix();
    
        ofTranslate(blobs[i].pos.value());
        ofSetRectMode(OF_RECTMODE_CENTER);
        
        ofTranslate(offsetImage.get());
        
        ofScale(scaleImage,scaleImage);
        syphonIn.draw(0, 0, blobs[i].size.value().y, blobs[i].size.value().y);
        
        ofSetRectMode(OF_RECTMODE_CORNER);
        ofPopMatrix();
        
    }
    
    
    fboOut.end();
    
    
    ofSetColor(255, 255, 255);
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    } else {
        // draw from the live kinect
        kinect.drawDepth(10, 10, 400, 300);
        kinect.draw(420, 10, 400, 300);
        
        depthImage.draw(10, 320, 400, 300);
        
        //grayImage.draw(10, 320, 400, 300);
        
        grayBg.draw(420,340, 400, 300);
        grayDiff.draw(860,340, 400, 300);
        
        contourFinder.draw(10, 320, 400, 300);
        
#ifdef USE_TWO_KINECTS
        kinect2.draw(420, 320, 400, 300);
#endif
    }
    
    // draw instructions
    ofSetColor(255, 255, 255);

    gui.draw();
    
    syphonOut.publishTexture(&fboOut.getTextureReference());
    
}

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
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
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            //kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            //kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case ' ':
                    bLearnBakground = true;
                    break;
        case 'u':
                    threshold ++;
                    if (threshold > 255) threshold = 255;
                    break;
        case 'j':
                    threshold --;
                    if (threshold < 0) threshold = 0;
                    break;
    }
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
