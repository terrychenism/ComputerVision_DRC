#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

Mat img,img2;

//---for left------------------------------
int alpha_= 90., beta_= 180., gamma_=90.;
int f_ = 456, dist_ = 210,dx_ = 666, dy_= 0;

//for right--------------------------------
int alpha2_= 90., beta2_= 0., gamma2_=90.;
int f2_ = 767, dist2_ = 772,dx2_ = 183, dy2_= 0;

Mat destination,destination2;
char* wndname1 = "left image";
char* wndname2 = "right image";

static void onTrackbar1(int, void*)
{
	double f, dist , dx ,dy ;
    double alpha, beta, gamma;

	alpha = ((double)alpha_ - 90.)*CV_PI/180;
   beta = ((double)beta_ - 90.)*CV_PI/180;
   gamma = ((double)gamma_ - 90.)*CV_PI/180;
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


    double qw = 0.966, qx=0.259,qy=0.0,qz=0.0;
    Mat TR = (Mat_<double>(4,4) <<
		1 - 2*qy*qy - 2*qz*qz,	2*qx*qy - 2*qz*qw ,	2*qx*qz + 2*qy*qw,dx,
		2*qx*qy + 2*qz*qw ,	1 - 2*qx*qx- 2*qz*qz,	2*qy*qz - 2*qx*qw,dy,
		2*qx*qz - 2*qy*qw , 2*qy*qz + 2*qx*qw ,	1 - 2*qx*qx- 2*qy*qy,dist,
		0,0,0,1);


	Mat transfo = A2 *(R*(TR*A1));

    //Mat transfo = A2 * (T * (R * A1));


    warpPerspective(img, destination, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);
    //warpPerspective(img2, destination2, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

    imshow(wndname1, destination);
}

static void onTrackbar2(int, void*)
{
	double f, dist , dx ,dy ;
    double alpha, beta, gamma;

	alpha = ((double)alpha2_ - 90.)*CV_PI/180;
   beta = ((double)beta2_ - 90.)*CV_PI/180;
   gamma = ((double)gamma2_ - 90.)*CV_PI/180;
       f = (double) f2_;
   dist = (double) dist2_;
   dx = (double)dx2_;
   dy = (double)dy2_;
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


	Mat transfo2 = A2 *(T*(R*A1));



    warpPerspective(img2, destination2, transfo2, taille, INTER_CUBIC | WARP_INVERSE_MAP);
  

    imshow(wndname2, destination2);
}
void StitchImages()
{
	Mat result;

	Mat image_left = imread( "./image/left.jpg", 1 );//l_undistorted_perspective
	Mat image_right = imread("./image/right.jpg", 1 );

	double qw = 0.966, qx=0.259,qy=0.0,qz=0.0;
    Mat TR = (Mat_<double>(4,4) <<
		1 - 2*qy*qy - 2*qz*qz,	2*qx*qy - 2*qz*qw ,	2*qx*qz + 2*qy*qw,0,
		2*qx*qy + 2*qz*qw ,	1 - 2*qx*qx- 2*qz*qz,	2*qy*qz - 2*qx*qw,0,
		2*qx*qz - 2*qy*qw , 2*qy*qz + 2*qx*qw ,	1 - 2*qx*qx- 2*qy*qy,500,
		0,0,0,1);
	 Size taille = img.size();
	 double w = (double)taille.width, h = (double)taille.height;
    Mat A1 = (Mat_<double>(4,3) <<
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0,    0,
        0, 0,    1);

	Mat A2 = (Mat_<double>(3,4) <<
        500, 0, w/2, 0,
        0, 500, h/2, 0,
        0, 0,   1, 0);

	Mat T = (Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 500,
        0, 0, 0, 1);
	Mat m_transfMatrix = A2 *T* A1;


	warpPerspective(image_right,result,m_transfMatrix,Size(image_left.cols+image_right.cols,image_left.rows));
	
	Mat half(result,Rect(image_left.cols,0,image_left.cols,image_left.rows));
	
	image_left.copyTo(half);
	imshow("stitching image",result);
	
}

int main( int argc, char** argv ){
    Point2f srcTri[3];
    Point2f dstTri[3];

    Mat rot_mat( 2, 3, CV_32FC1 );
    Mat warp_mat( 2, 3, CV_32FC1 );
    Mat src, warp_dst, warp_rotate_dst,dst;


    img = imread( "./image/l_undistorted_perspective.jpg", 1 );
    img2 =imread( "./image/r_undistorted_perspective.jpg", 1 );


	string tbarname1 = "Alpha";
	string tbarname2 = "Beta";
	string tbarname3 = "Gamma";
	string tbarname4 = "f";
	string tbarname5 = "Distance";
	string tbarname6 = "Dx";
	string tbarname7 = "Dy";
	namedWindow(wndname1, 1);
	namedWindow(wndname2, 1);

	// createTrackbar(tbarname1, wndname1, &alpha_, 360);
	createTrackbar(tbarname2, wndname1, &beta_, 180 , onTrackbar1);
	// createTrackbar(tbarname3, wndname1, &gamma_, 360);
	createTrackbar(tbarname4, wndname1, &f_, 2000, onTrackbar1);
	createTrackbar(tbarname5, wndname1, &dist_, 2000, onTrackbar1);
	createTrackbar(tbarname6, wndname1, &dx_, 2000, onTrackbar1);
	//createTrackbar(tbarname7, wndname1, &dy_, 2000);
	onTrackbar1(0, 0);


   //-------for right-------
	createTrackbar(tbarname2, wndname2, &beta2_, 180 , onTrackbar2);
	createTrackbar(tbarname4, wndname2, &f2_, 2000, onTrackbar2);
	createTrackbar(tbarname5, wndname2, &dist2_, 2000, onTrackbar2);
	createTrackbar(tbarname6, wndname2, &dx2_, 2000, onTrackbar2);
	onTrackbar2(0, 0);
    

	imwrite("./image/right.jpg",destination2);
	imwrite("./image/left.jpg",destination);
	StitchImages();
	cvWaitKey();

	return 0;
}
