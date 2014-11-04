#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MultiObjectTLD.h"


using namespace std;
using namespace cv;

#define LOADCLASSIFIERATSTART 0
#define CLASSIFIERFILENAME "test.moctld"
const char* videoName = "output.mpg";//"light.avi";


//#define FORCE_RESIZING
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

#define MOUSE_MODE_MARKER 0
#define MOUSE_MODE_ADD_BOX 1
#define MOUSE_MODE_IDLE 2

#define gotBB_MODE_MARKER 0
#define gotBB_MODE_ADD_BOX 1
#define gotBB_MODE_IDLE 2
int gotBB = gotBB_MODE_MARKER;

IplImage* curImage = NULL;
bool ivQuit = false;
int ivWidth, ivHeight;
CvCapture* capture;
ObjectBox mouseBox = {0,0,0,0,0};
int mouseMode = MOUSE_MODE_IDLE;
int drawMode = 255;
bool learningEnabled = true, save = false, load = false, reset = false;



void Init(int argc, char *argv[]);
void* Run(void*);
void HandleInput(int interval = 1);
void MouseHandler(int event, int x, int y, int flags, void* param);
void FromRGB(Matrix& maRed, Matrix& maGreen, Matrix& maBlue);

int main(int argc, char *argv[])
{
  Init(argc, argv);
  Run(0);
  cvDestroyAllWindows();
  return 0;
}


void Init(int argc, char *argv[])
{  
	capture = cvCaptureFromFile(videoName);
  //VideoCapture capture("light2.avi");
  if(!capture){
    std::cout << "error starting video capture" << std::endl;
    exit(0);
  }
  //propose a resolution
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
  //get the actual (supported) resolution
  ivWidth = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
  ivHeight = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
#ifdef FORCE_RESIZING
  ivWidth = RESOLUTION_X;
  ivHeight = RESOLUTION_Y;
#endif
  
  cvNamedWindow("tracker", 0); //CV_WINDOW_AUTOSIZE );
  
  CvSize wsize = {ivWidth, ivHeight};
  curImage = cvCreateImage(wsize, IPL_DEPTH_8U, 3);
  
  cvResizeWindow("tracker", ivWidth, ivHeight);
  cvSetMouseCallback("tracker", MouseHandler);
}

void* Run(void*)
{
  int size = ivWidth*ivHeight;
  // PM_dem PM(modelName);
  // Initialize MultiObjectTLD
  #if LOADCLASSIFIERATSTART
  MultiObjectTLD p = MultiObjectTLD::loadClassifier((char*)CLASSIFIERFILENAME);
  #else
  MOTLDSettings settings(COLOR_MODE_RGB);
  settings.useColor = false;
  MultiObjectTLD p(ivWidth, ivHeight, settings);
  #endif
  
  Matrix maRed;
  Matrix maGreen;
  Matrix maBlue;
  unsigned char * img = new unsigned char[ivWidth * ivHeight * 3];
  #ifdef FORCE_RESIZING
  CvSize wsize = {ivWidth, ivHeight};
  IplImage* frame = cvCreateImage(wsize, IPL_DEPTH_8U, 3);
  #endif

  clock_t nTimeStart;
  clock_t nTimeStop;
  nTimeStart = clock();
  vector<Rect> bbox;

  while(!ivQuit)
  {    
    /*
    if(reset){
      p = *(new MultiObjectTLD(ivWidth, ivHeight, COLOR_MODE_RGB));
      reset = false;
    }
    if(load){
      p = MultiObjectTLD::loadClassifier(CLASSIFIERFILENAME);
      load = false;
    }
    */

	 nTimeStop = clock();
    // Grab an image
    if(!cvGrabFrame(capture)){
      std::cout << "error" << std::endl;
      break;
    }


    #ifdef FORCE_RESIZING
    IplImage* capframe = cvRetrieveFrame(capture);
    cvResize(capframe, frame);
    #else
    IplImage* frame = cvRetrieveFrame(capture);
    #endif

	
	if (gotBB == gotBB_MODE_MARKER){
  // if(mouseMode == MOUSE_MODE_MARKER){  
    mouseMode == MOUSE_MODE_ADD_BOX;
		while (1){	// MOUSE CONTROL
      
			cvShowImage("tracker", frame);
			if ((char)cvWaitKey(10) == 'q')
				return 0;
			if ((char)cvWaitKey(10) == 's')
				break;
		}
 
		// AUTO DETECT
		// Mat mframe = Mat(frame, false);
		// Mat	img_ = PM.prepareImg(mframe);
		// bbox = PM.detect(img_, -0.68f, false, true);		
		 gotBB = gotBB_MODE_ADD_BOX;
	 }
	


    for(int j = 0; j<size; j++){
      img[j] = frame->imageData[j*3+2];
      img[j+size] = frame->imageData[j*3+1];
      img[j+2*size] = frame->imageData[j*3];
    }
    
    // Processing
    p.processFrame(img);
    // if(mouseMode == MOUSE_MODE_ADD_BOX){
    //   p.addObject(mouseBox);
    //   mouseMode = MOUSE_MODE_IDLE;
    // }
	// if (gotBB == gotBB_MODE_ADD_BOX){
	// 	if (bbox.size() == 0)
	// 		gotBB = gotBB_MODE_IDLE;
		
	// 	else{
	// 		Rect box = bbox.back();
	// 		mouseBox.x = box.x;
	// 		mouseBox.y = box.y;
	// 		mouseBox.width = box.width;
	// 		mouseBox.height = box.height;
	// 		p.addObject(mouseBox);
	// 		bbox.pop_back();
	// 	}
	// }

	// if (nTimeStop - nTimeStart > 5000){
	// 	nTimeStart = clock();
	// 	gotBB = gotBB_MODE_MARKER;
	// 	}

    // Add new box
    if(mouseMode == MOUSE_MODE_ADD_BOX){
      p.addObject(mouseBox);
      mouseMode = MOUSE_MODE_IDLE;
    }
    
    // Display result
    HandleInput();
    p.getDebugImage(img, maRed, maGreen, maBlue, drawMode);    
    FromRGB(maRed, maGreen, maBlue);
    cvShowImage("tracker", curImage);
    p.enableLearning(learningEnabled);
	waitKey(1);
    if(save){
      p.saveClassifier((char*)CLASSIFIERFILENAME);
      save = false;
    }
  }
  delete[] img;
  cvReleaseCapture(&capture);
  return 0;
}

void HandleInput(int interval)
{
  int key = cvWaitKey(interval);
  if(key >= 0)
  {
    switch (key)
    {
      case 'd': drawMode ^= DEBUG_DRAW_DETECTIONS;  break;
      case 't': drawMode ^= DEBUG_DRAW_CROSSES;  break;
      case 'p': drawMode ^= DEBUG_DRAW_PATCHES;  break;
      case 'l':
        learningEnabled = !learningEnabled;
        std::cout << "learning " << (learningEnabled? "en" : "dis") << "abled" << std::endl;
        break;
      case 'r': reset = true; break;
      case 's': save = true;  break;
      case 'o': load = true;  break;
      case 27:  ivQuit = true; break; //ESC
      default: 
        //std::cout << "unhandled key-code: " << key << std::endl;
        break;
    }
  }
}

void MouseHandler(int event, int x, int y, int flags, void* param)
{
  switch(event){
    case CV_EVENT_LBUTTONDOWN:
      mouseBox.x = x;
      mouseBox.y = y;
      mouseBox.width = mouseBox.height = 0;
      mouseMode = MOUSE_MODE_MARKER;
      break;
    case CV_EVENT_MOUSEMOVE:
      if(mouseMode == MOUSE_MODE_MARKER){
        mouseBox.width = x - mouseBox.x;
        mouseBox.height = y - mouseBox.y;
      }
      break;
    case CV_EVENT_LBUTTONUP:
      if(mouseMode != MOUSE_MODE_MARKER)
        break;
      if(mouseBox.width < 0){
        mouseBox.x += mouseBox.width;
        mouseBox.width *= -1;
      }
      if(mouseBox.height < 0){
        mouseBox.y += mouseBox.height;
        mouseBox.height *= -1;
      }
      if(mouseBox.width < 4 || mouseBox.height < 4){
        std::cout << "bounding box too small!" << std::endl;
        mouseMode = MOUSE_MODE_IDLE;
      }else
        mouseMode = MOUSE_MODE_ADD_BOX;
      break;
    case CV_EVENT_RBUTTONDOWN:
      mouseMode = MOUSE_MODE_IDLE;
      break;
  }
}

void FromRGB(Matrix& maRed, Matrix& maGreen, Matrix& maBlue)
{
  for(int i = 0; i < ivWidth*ivHeight; ++i){
    curImage->imageData[3*i+2] = maRed.data()[i];
    curImage->imageData[3*i+1] = maGreen.data()[i];
    curImage->imageData[3*i+0] = maBlue.data()[i];
  }
  //at this place you could save the images using
  //cvSaveImage(filename, curImage);
  if(mouseMode == MOUSE_MODE_MARKER)
  {
    CvPoint pt1; pt1.x = mouseBox.x; pt1.y = mouseBox.y;
    CvPoint pt2; pt2.x = mouseBox.x + mouseBox.width; pt2.y = mouseBox.y + mouseBox.height;  
    cvRectangle(curImage, pt1, pt2, CV_RGB(0,0,255));
  }
}
