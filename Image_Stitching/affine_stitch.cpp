#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


char* source_window = "Source image";
char* warp_window = "Warp";
char* warp_rotate_window = "Warp + Rotate";
char* pzRotatedImage = "Rotated Image";
Mat img;


int main( int argc, char** argv ){
   Point2f srcTri[3];
   Point2f dstTri[3];

   Mat rot_mat( 2, 3, CV_32FC1 );
   Mat warp_mat( 2, 3, CV_32FC1 );
   Mat src, warp_dst, warp_rotate_dst,dst;


   img = imread( "./image/l_undistorted_perspective.jpg", 1 );
   Mat img2 =imread( "./image/r_undistorted_perspective.jpg", 1 );
    //double w = (double)img.cols;
    //double h = (double)img.rows;
    //double alpha = 0.5;
int alpha_= 90., beta_= 130., gamma_=90.;
    int f_ = 500, dist_ = 800,dx_ = -100, dy_= 0;


Mat destination,destination2;
char* wndname1 = "Source image";
char* wndname2 = "WarpPerspective: ";

string tbarname1 = "Alpha";
string tbarname2 = "Beta";
string tbarname3 = "Gamma";
string tbarname4 = "f";
string tbarname5 = "Distance";
string tbarname6 = "Dx";
string tbarname7 = "Dy";
namedWindow(wndname1, 1);
namedWindow(wndname2, 1);
  //createTrackbar(tbarname1, wndname1, &alpha_, 360);
  //createTrackbar(tbarname2, wndname1, &beta_, 360);
  //createTrackbar(tbarname3, wndname1, &gamma_, 360);
  //createTrackbar(tbarname4, wndname1, &f_, 2000);
  //createTrackbar(tbarname5, wndname1, &dist_, 2000);
  //createTrackbar(tbarname6, wndname1, &dx_, 2000);
  //createTrackbar(tbarname7, wndname1, &dy_, 2000);




while(true) {
    double f, dist , dx ,dy ;
    double alpha, beta, gamma;
   alpha = ((double)alpha_ - 90.)*CV_PI/180;
   beta = ((double)beta_ - 90.)*CV_PI/180;
   gamma = ((double)gamma_ - 90.)*CV_PI/180;
    // alpha =0.0;
    // beta = -3.142;
    // gamma = 2.618;
    f = (double) f_;
    dist = (double) dist_;
    dx = (double)dx_;
    dy = (double)dy_;

    Size taille = img.size();
    double w = (double)taille.width, h = (double)taille.height;


    Mat A1 = (Mat_<double>(4,3) <<
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0,    0,
        0, 0,    1);


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


    Mat R = RX * RY * RZ;

    Mat T = (Mat_<double>(4, 4) <<
        1, 0, 0, dx,
        0, 1, 0, dy,
        0, 0, 1, dist,
        0, 0, 0, 1);

    Mat A2 = (Mat_<double>(3,4) <<
        f, 0, w/2, 0,
        0, f, h/2, 0,
        0, 0,   1, 0);

    double a = 0.966, b=0.259,c=-0.0,d=-0.0;

    /*Mat TR = (Mat_<double>(4,4) <<
    	a*a+b*b-c*c-d*d, 2*b*c-2*a*d, 2*b*d+2*a*c, 0,
    	2*b*c+2*a*d, a*a-b*b+c*c-d*d, 2*c*d-2*a*b,0,
    	2*b*d-2*a*c,2*c*d+2*a*b, a*a-b*b-c*c+d*d,0,
    	0,0,0,1);*/

     double qw = 0.966, qx=0.259,qy=0.0,qz=0.0;
     Mat TR = (Mat_<double>(4,4) <<
		 1 - 2*qy*qy - 2*qz*qz,	2*qx*qy - 2*qz*qw ,	2*qx*qz + 2*qy*qw,0,
		 2*qx*qy + 2*qz*qw ,	1 - 2*qx*qx- 2*qz*qz,	2*qy*qz - 2*qx*qw,0,
		 2*qx*qz - 2*qy*qw , 2*qy*qz + 2*qx*qw ,	1 - 2*qx*qx- 2*qy*qy,0,
		 0,0,0,1);

	Mat transfo = A2 *(T*(R*(TR*A1)));

    //Mat transfo = A2 * (T * (R * A1));


    warpPerspective(img, destination, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);
     //warpPerspective(img2, destination2, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

    imshow(wndname1, destination);
    //imwrite("newimage.jpg",destination);
    imshow(wndname2, img2);
    waitKey();

 }

   return 0;
}
