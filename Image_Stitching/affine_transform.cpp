#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

/// Global variables
char* source_window = "Source image";
char* warp_window = "Warp";
char* warp_rotate_window = "Warp + Rotate";

void rotateImage(const Mat &input, Mat &output, double alpha, double beta, double gamma, double dx, double dy, double dz, double f)
  {
    alpha = (alpha - 90.)*CV_PI/180.;
    beta = (beta - 90.)*CV_PI/180.;
    gamma = (gamma - 90.)*CV_PI/180.;
    // get width and height for ease of use in matrices
    double w = (double)input.cols;
    double h = (double)input.rows;
    // Projection 2D -> 3D matrix
    Mat A1 = (Mat_<double>(4,3) <<
              1, 0, -w/2,
              0, 1, -h/2,
              0, 0,    0,
              0, 0,    1);
    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<double>(4, 4) <<
              1,          0,           0, 0,
              0, cos(alpha), -sin(alpha), 0,
              0, sin(alpha),  cos(alpha), 0,
              0,          0,           0, 1);
    Mat RY = (Mat_<double>(4, 4) <<
              cos(beta), 0, -sin(beta), 0,
              0, 1,          0, 0,
              sin(beta), 0,  cos(beta), 0,
              0, 0,          0, 1);
    Mat RZ = (Mat_<double>(4, 4) <<
              cos(gamma), -sin(gamma), 0, 0,
              sin(gamma),  cos(gamma), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Composed rotation matrix with (RX, RY, RZ)
    Mat R = RX * RY * RZ;
    // Translation matrix
    Mat T = (Mat_<double>(4, 4) <<
             1, 0, 0, dx,
             0, 1, 0, dy,
             0, 0, 1, dz,
             0, 0, 0, 1);
    // 3D -> 2D matrix
    Mat A2 = (Mat_<double>(3,4) <<
              f, 0, w/2, 0,
              0, f, h/2, 0,
              0, 0,   1, 0);
    // Final transformation matrix
    Mat trans = A2 * (T * (R * A1));
    // Apply matrix transformation
    //warpPerspective(input, output, trans, input.size(), INTER_LANCZOS4);
	warpPerspective(input, output, trans, input.size(), INTER_CUBIC | WARP_INVERSE_MAP);
  }
/** @function main */
 int main( int argc, char** argv )
 {
   Point2f srcTri[3];
   Point2f dstTri[3];

   Mat rot_mat( 2, 3, CV_32FC1 );
   Mat warp_mat( 2, 3, CV_32FC1 );
   Mat src, warp_dst, warp_rotate_dst,dst,img;

   /// Load the image
   img = imread( "img_right_rectified.jpg", 1 );
 //rotateImage(src, dst, 90, 90, 90, 0, 0, 0, 1);
   float rotx = 0, roty= -70 , rotz= 0; // set these first
int f = 2; // this is also configurable, f=2 should be about 50mm focal length

int h = img.rows;
int w = img.cols;

float cx = cosf(rotx), sx = sinf(rotx);
float cy = cosf(roty), sy = sinf(roty);
float cz = cosf(rotz), sz = sinf(rotz);

float roto[3][2] = { // last column not needed, our vector has z=0
    { cz * cy, cz * sy * sx - sz * cx },
    { sz * cy, sz * sy * sx + cz * cx },
    { -sy, cy * sx }
};

float pt[4][2] = {{ -w / 2, -h / 2 }, { w / 2, -h / 2 }, { w / 2, h / 2 }, { -w / 2, h / 2 }};
float ptt[4][2];
for (int i = 0; i < 4; i++) {
    float pz = pt[i][0] * roto[2][0] + pt[i][1] * roto[2][1];
    ptt[i][0] = w / 2 + (pt[i][0] * roto[0][0] + pt[i][1] * roto[0][1]) * f * h / (f * h + pz);
    ptt[i][1] = h / 2 + (pt[i][0] * roto[1][0] + pt[i][1] * roto[1][1]) * f * h / (f * h + pz);
}

cv::Mat in_pt = (cv::Mat_<float>(4, 2) << 0, 0, w, 0, w, h, 0, h);
cv::Mat out_pt = (cv::Mat_<float>(4, 2) << ptt[0][0], ptt[0][1],
    ptt[1][0], ptt[1][1], ptt[2][0], ptt[2][1], ptt[3][0], ptt[3][1]);

cv::Mat transform = cv::getPerspectiveTransform(in_pt, out_pt);

cv::Mat img_in = img.clone();
cv::warpPerspective(img_in, img, transform, img_in.size());


   imshow("perspect",img);
   /// Set the dst image the same type and size as src
   //warp_dst = Mat::zeros( src.rows, src.cols, src.type() );

   ///// Set your 3 points to calculate the  Affine Transform
   //srcTri[0] = Point2f( 0,0 );
   //srcTri[1] = Point2f( src.cols - 1, 0 );
   //srcTri[2] = Point2f( 0, src.rows - 1 );

   //dstTri[0] = Point2f( src.cols*0.0, src.rows*0.3 );
   //dstTri[1] = Point2f( src.cols*0.8, src.rows*0.2 );
   //dstTri[2] = Point2f( src.cols*0.0, src.rows*1.0 );

   ///// Get the Affine Transform
   //warp_mat = getAffineTransform( srcTri, dstTri );

   ///// Apply the Affine Transform just found to the src image
   //warpAffine( src, warp_dst, warp_mat, warp_dst.size() );

   ///** Rotating the image after Warp */

   ///// Compute a rotation matrix with respect to the center of the image
   //Point center = Point( warp_dst.cols/2, warp_dst.rows/2 );
   //double angle = -5.0;
   //double scale = 1.0;

   ///// Get the rotation matrix with the specifications above
   //rot_mat = getRotationMatrix2D( center, angle, scale );

   ///// Rotate the warped image
  // warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );

   /// Show what you got
   //namedWindow( source_window, CV_WINDOW_AUTOSIZE );
   //imshow( source_window, src );

   //namedWindow( warp_window, CV_WINDOW_AUTOSIZE );
   //imshow( warp_window, warp_dst );

   //namedWindow( warp_rotate_window, CV_WINDOW_AUTOSIZE );
   //imshow( warp_rotate_window, warp_rotate_dst );

   /// Wait until user exits the program
   waitKey(0);

   return 0;
  }



//int iAngle = 180;
//int iScale = 50;
//int iBorderMode = 0;
//Mat imgOriginal ;
//int iImageCenterY = 0;
//int iImageCenterX = 0;
//const char* pzRotatedImage = "Rotated Image";
//
//void CallbackForTrackBar(int, void*)
//{
// Mat matRotation = getRotationMatrix2D(  Point( iImageCenterX, iImageCenterY ), (iAngle - 180), iScale / 50.0 );
// 
// // Rotate the image
// Mat imgRotated;
// warpAffine( imgOriginal, imgRotated, matRotation, imgOriginal.size(), INTER_LINEAR, iBorderMode, Scalar() );
//
//  imshow( pzRotatedImage, imgRotated );
// 
//}
//
// int main( int argc, char** argv )
// {
// // Load the image
// imgOriginal = imread( "img_right_rectified.jpg", 1 );
//
//  iImageCenterY = imgOriginal.rows / 2;
// iImageCenterX = imgOriginal.cols / 2;
//
//  //show the original image
// const char* pzOriginalImage = "Original Image";
// namedWindow( pzOriginalImage, CV_WINDOW_AUTOSIZE );
// imshow( pzOriginalImage, imgOriginal );
//
//  //create the "Rotated Image" window and 3 trackbars in it
// namedWindow( pzRotatedImage, CV_WINDOW_AUTOSIZE );
// createTrackbar("Angle", pzRotatedImage, &iAngle, 360, CallbackForTrackBar);
// createTrackbar("Scale", pzRotatedImage, &iScale, 100, CallbackForTrackBar);
// createTrackbar("Border Mode", pzRotatedImage, &iBorderMode, 5, CallbackForTrackBar);
// 
// int iDummy = 0;
//
//  CallbackForTrackBar(iDummy, &iDummy);
//
//  waitKey(0);
//
//  return 0;
//}
