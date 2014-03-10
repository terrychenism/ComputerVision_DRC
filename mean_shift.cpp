
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;


Mat src,dst;
int spatialRad=10,colorRad=10,maxPryLevel=1;
//const Scalar& colorDiff=Scalar::all(1);

void meanshift_seg(int,void *)
 {

    pyrMeanShiftFiltering(src,dst,spatialRad,colorRad,maxPryLevel);
    imshow("dst",dst);
 }
  
  
 int main(int argc, uchar* argv[])
 {
      
     namedWindow("src",WINDOW_AUTOSIZE);
     namedWindow("dst",WINDOW_AUTOSIZE);
  
     src=imread("debris.jpg");
     imshow("src",src); 
     CV_Assert(!src.empty());
  
     spatialRad=28;
     colorRad=36;
     maxPryLevel=2;
 
     //createTrackbar("spatialRad","dst",&spatialRad,80,meanshift_seg);
     //createTrackbar("colorRad","dst",&colorRad,60,meanshift_seg);
    // createTrackbar("maxPryLevel","dst",&maxPryLevel,5,meanshift_seg);
  
     meanshift_seg(0,0);
     cv::imwrite("c:\\result.jpg",dst );
      
     waitKey();
     return 0;
 }
