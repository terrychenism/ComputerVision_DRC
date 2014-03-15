#ifdef _CH_
#pragma package <opencv>
#endif
 
#ifndef _EiC
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <perception_common/MultisenseImage.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include <time.h>
#endif
 
int thresh = 50;
IplImage* img = 0;
IplImage* img0 = 0;
CvMemStorage* storage = 0;
CvPoint pt[4];
const char* wndname = "Square Detection";
using namespace cv;
using namespace std;


Mat src,dst;
int spatialRad=10,colorRad=10,maxPryLevel=1;



void PrintfContainerElapseTime(char *pszContainerName, char *pszOperator, long lElapsetime)
{
    printf("%s %s time %dsec\n", pszContainerName, pszOperator, lElapsetime/1000);
}
//const Scalar& colorDiff=Scalar::all(1);
double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
 
// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4( IplImage* img, CvMemStorage* storage )
{
    CvSeq* contours;
    int i, c, l, N = 11;
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* gray = cvCreateImage( sz, 8, 1 ); 
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
    IplImage* tgray;
    CvSeq* result;
    double s, t;
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );
    
    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));
    
    // down-scale and upscale the image to filter out the noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8, 1 );
    
    // find squares in every color plane of the image
    for( c = 0; c < 3; c++ )
    {
        // extract the c-th color plane
        cvSetImageCOI( timg, c+1 );
        cvCopy( timg, tgray, 0 );
        
        // try several threshold levels
        for( l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading   
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging) 
                cvCanny( tgray, gray, 0, thresh, 5 );
                // dilate canny output to remove potential
                // holes between edge segments 
                cvDilate( gray, gray, 0, 1 );
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
            }
            
            // find contours and store them all as a list
            cvFindContours( gray, storage, &contours, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
            
            // test each contour
            while( contours )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                result = cvApproxPoly( contours, sizeof(CvContour), storage,
                    CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( result->total == 4 &&
                    fabs(cvContourArea(result,CV_WHOLE_SEQ)) > 1000 &&
                    cvCheckContourConvexity(result) )
                {
                    s = 0;
                    
                    for( i = 0; i < 5; i++ )
                    {
                        // find minimum angle between joint
                        // edges (maximum of cosine)
                        if( i >= 2 )
                        {
                            t = fabs(angle(
                            (CvPoint*)cvGetSeqElem( result, i ),
                            (CvPoint*)cvGetSeqElem( result, i-2 ),
                            (CvPoint*)cvGetSeqElem( result, i-1 )));
                            s = s > t ? s : t;
                        }
                    }
                    
                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence 
                    if( s < 0.3 )
                        for( i = 0; i < 4; i++ )
                            cvSeqPush( squares,
                                (CvPoint*)cvGetSeqElem( result, i ));
                }
                
                // take the next contour
                contours = contours->h_next;
            }
        }
    }
    
    // release all the temporary images
    cvReleaseImage( &gray );
    cvReleaseImage( &pyr );
    cvReleaseImage( &tgray );
    cvReleaseImage( &timg );
    
    return squares;
}
 
 
// the function draws all the squares in the image
void drawSquares( IplImage* img, CvSeq* squares )
{
    CvSeqReader reader;
    //cvShowImage("before draw", img);
    IplImage* cpy = cvCloneImage( img );
    int i;
    
    // initialize reader of the sequence
    cvStartReadSeq( squares, &reader, 0 );
    
    // read 4 sequence elements at a time (all vertices of a square)
    for( i = 0; i < squares->total; i += 4 )
    {
        CvPoint* rect = pt;
        int count = 4;
        
        // read 4 vertices
        memcpy( pt, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );
        memcpy( pt + 1, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );
        memcpy( pt + 2, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );
        memcpy( pt + 3, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );
        
        // draw the square as a closed polyline 
        cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 3, CV_AA, 0 );
    }
    
    // show the resultant image
    cvShowImage( wndname, cpy );
    cvReleaseImage( &cpy );
}
void meanshift_seg(int,void *)
 {

    pyrMeanShiftFiltering(src,dst,spatialRad,colorRad,maxPryLevel);
    //imshow("dst",dst);
 }
  
  
 int main(int argc, char* argv[])
 {
      clock_t  clockBegin, clockEnd; 
     //namedWindow("src",WINDOW_AUTOSIZE);
     //namedWindow("dst",WINDOW_AUTOSIZE);
  ros::init(argc, argv, "ros_opencv_example");
     //src=imread("debris.jpg");
  //src = imread( "/home/tairuichen/Desktop/obj.jpg" );
  ros::NodeHandle nh;
  drc_perception::MultisenseImage camera(nh);

     while(ros::ok())
    {
            if(!camera.giveLeftColorImage(src))
            {
              ros::spinOnce();
              continue;
            }

     //imshow("src",src); 
     //CV_Assert(!src.empty());
     if ( src.empty() ) {
                std::cout << "unable to load an input image\n";
                //system("pause");
                return -1;
        }
     
    //imshow("raw image", src);
     spatialRad=28;
     colorRad=36;
     maxPryLevel=3;
     storage = cvCreateMemStorage(0);

     clockBegin = clock();
     meanshift_seg(0,0);// reduce running time
     clockEnd = clock();
     PrintfContainerElapseTime("mean", "shift", clockEnd - clockBegin);

     //cv::imwrite("/home/tairuichen/Desktop/newpic.jpg",src );
     Mat  cdst,ddst,edst;
     
     Canny(dst, ddst, 50, 200, 3);
     
     cvtColor(ddst, cdst, CV_GRAY2BGR);


     vector<Vec4i> lines;
      
       HoughLinesP(ddst, lines, 1, CV_PI/180, 70,50,10 );

    
      for( size_t i = 0; i < lines.size(); i++ )
      {
        Vec4i l = lines[i];
        line( cdst, Point(l[3], l[0]), Point(l[1], l[2]), Scalar(0,0,255), 3, CV_AA);
      }


     imshow("detected lines", cdst);
     IplImage copy = cdst;
     img = &copy;

     IplImage copy2 = src;
     IplImage *img3 = &copy2;
     
     drawSquares( img3, findSquares4( img, storage ) );

     //imwrite("lines.jpg", cdst);
     waitKey(20);
     ros::spinOnce();
  }
     return 0;
 }

#ifdef _EiC
main(1,"squares.c");
#endif
