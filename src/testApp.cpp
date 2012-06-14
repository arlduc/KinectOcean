#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    ofEnableAlphaBlending();
    
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
    compositeImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR_ALPHA);
	
	nearThreshold = 200;
	farThreshold = 0;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    ofBackground(100, 200, 255);

}

//--------------------------------------------------------------
void testApp::update() {
    flock.update();
    for(int i=0; i<flock.boids.size(); i++) {            
        if (!overlapsRGBAComposite(&compositeImg, flock.boids[i], kinect.width, kinect.height))
            flock.boids[i].vel.set(0,0);
            flock.boids[i].acc.set(0,0);
    }
    
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
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
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        
        makeRGBAComposite(&colorImg, &grayImage, &compositeImg,  kinect.width, kinect.height,true);

	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
        ofSetColor(90, 175, 225); // darker color
		// draw from the live kinect
		//kinect.drawDepth(10, 10, 400, 300);
		//kinect.draw(10, 320, 400, 300);
        //kinect.draw(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
		//contourFinder.draw(-50, -80, ofGetWindowWidth()*1.12, ofGetWindowHeight()*1.12);
        
        //kinect.drawDepth(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
		//grayImage.draw(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
        compositeImg.draw(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
        
        //contourFinder.draw(10, 320, 400, 300);
		
        ofSetColor(255, 255, 255); // white
        flock.draw();
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
    
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
     
}

void testApp::drawPointCloud() {
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
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}


//GENERIC Function to create an RGBA Image from a color image + mask. Must be same number of pixels. (can be used elsewhere) 

void testApp::makeRGBAComposite(ofxCvColorImage* _colorImg, ofxCvGrayscaleImage* _maskImg, ofImage* _composite, int _w, int _h, bool _fillBlack) {
	//printf(" w: %d, h: %d \n", _w, _h);
	unsigned char * colorPx = _colorImg->getPixels(); // 3 channel image
	unsigned char * maskPx = _maskImg->getPixels(); // 1 channel image
	unsigned char alphaImg[_w*_h*4]; // 4 channel image
	
	for(int i=0; i<_w; i++) { //horizontal row
		for(int j=0; j<_h; j++) { 
			for(int k=0; k<4; k++) { //each of the 4 channels in the image
				//translate values
				int loc = i+(j*_w);
				int RGBAval = loc*4 + k;
				int colorVal = loc*3 + k;
				//filter channels
				if(k == 3) {
					//alpha channel
					alphaImg[RGBAval] = maskPx[loc];
				} else {	
                    alphaImg[RGBAval] = 255;  //arlduc wants white! 
                    /*
					if((maskPx[loc] < 1) && (_fillBlack)){
						alphaImg[RGBAval] = 255;  //make it black if the mask is black. 	
					} else {
						alphaImg[RGBAval] = colorPx[colorVal];
                        
					}*/
				}
			}
		}
	} 
	_composite->setFromPixels(alphaImg,_w,_h,OF_IMAGE_COLOR_ALPHA,true);
}

bool testApp::overlapsRGBAComposite(ofImage* _composite, Boid b, int _w, int _h) {
	unsigned char *alphaImg = _composite->getPixels();
    int loc = b.loc.x+(b.loc.y*_w);
    int RGBAval = loc*4 + 3;
    
    printf("b.loc.x: %f, b.loc.y: %f, RGBAval:%i, alphaImg[RGBAval]:%c \n", b.loc.x, b.loc.y, RGBAval, alphaImg[RGBAval]);

    if(b.loc.x <= _w && b.loc.y<=_h) {
        if (alphaImg[RGBAval]==255) {
            printf("OVERLAP \n");
            return true;
        }
        else {
            printf("DOESN'T OVERLAP\n");
            return false;
        }
    } 
    else {
        printf("OUT OF RANGE \n");
        return false; 
    }
}



//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
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
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
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
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    printf("mouse.x: %i, mouse.y: %i \n", x, y);
	flock.addBoid(x,y);
    //flock.addBoid();
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
