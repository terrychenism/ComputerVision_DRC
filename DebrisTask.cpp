
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
     //cv::imwrite("result.jpg",dst );
	 Mat  cdst,ddst,edst;
	 
	 Canny(dst, ddst, 50, 200, 3);
	 
	 cvtColor(ddst, cdst, CV_GRAY2BGR);


	 vector<Vec4i> lines;
	  HoughLinesP(ddst, lines, 1, CV_PI/180, 50, 50, 10 );
	  for( size_t i = 0; i < lines.size(); i++ )
	  {
		Vec4i l = lines[i];
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
	  }


	 imshow("detected lines", cdst);

      
     waitKey();
     return 0;
 }
