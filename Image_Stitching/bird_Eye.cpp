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
int rotx = 0, roty= 24 , rotz= 0;


//void rotateImg(float rotx, float roty, float rotz){
void CallbackForTrackBar(int, void*){
	   //float rotx = 0, roty= -70 , rotz= 0; // set these first
double f = 0.8; // this is also configurable, f=2 should be about 50mm focal length

int h = img.rows;
int w = img.cols;

float cx = cosf(rotx), sx = sinf(rotx);
float cy = cosf(roty), sy = sinf(roty);
float cz = cosf(rotz), sz = sinf(rotz);

float roto[3][2] = { // z=0
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
imshow( pzRotatedImage, img );
}


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
    Mat R =  RZ;
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
    warpPerspective(input, output, trans, input.size());
	imshow("sdfsfs", output);

  }


 int main( int argc, char** argv )
 {
   Point2f srcTri[3];
   Point2f dstTri[3];

   Mat rot_mat( 2, 3, CV_32FC1 );
   Mat warp_mat( 2, 3, CV_32FC1 );
   Mat src, warp_dst, warp_rotate_dst,dst;


   img = imread( "img_left_rectified.jpg", 1 );
   //Mat img2 = imread( "./image/frame0.jpg", 1 );
   //imshow("right",img2);

    double w = (double)img.cols;
    double h = (double)img.rows;
	//double alpha = 0.5;
	int alpha_=90., beta_=45., gamma_=90.;
	int f_ = 500, dist_ = 500,dx_ = 0, dy_= 0;

	Mat destination;
	char* wndname1 = "Source image";
	char* wndname2 = "WarpPerspective: ";

	string tbarname1 = "Alpha";
	string tbarname2 = "Beta";
	string tbarname3 = "Gamma";
	string tbarname4 = "f";
	string tbarname5 = "Dz";
	string tbarname6 = "Dx";
	string tbarname7 = "Dy";
	//namedWindow(wndname1, 1);
	namedWindow(wndname2, 1);
	createTrackbar(tbarname1, wndname2, &alpha_, 180);
	createTrackbar(tbarname2, wndname2, &beta_, 180);
	createTrackbar(tbarname3, wndname2, &gamma_, 180);
	createTrackbar(tbarname4, wndname2, &f_, 2000);
	createTrackbar(tbarname5, wndname2, &dist_, 2000);
	createTrackbar(tbarname6, wndname2, &dx_, 2000);
	createTrackbar(tbarname7, wndname2, &dy_, 2000);

	//imshow(wndname1, img);
	while(true) {
		double f, dist, dx ,dy ;
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

		// Projection 2D -> 3D matrix
		Mat A1 = (Mat_<double>(4,3) <<
			1, 0, -w/2,
			0, 1, -h/2,
			0, 0,    0,
			0, 0,    1);

		// Rotation matrices around the X,Y,Z axis
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

		// Composed rotation matrix with (RX,RY,RZ)
		Mat R = RX * RY * RZ;

		// Translation matrix on the Z axis change dist will change the height
		Mat T = (Mat_<double>(4, 4) <<
			1, 0, 0, dx,
			0, 1, 0, dy,
			0, 0, 1, dist,
			0, 0, 0, 1);

		// Camera Intrisecs matrix 3D -> 2D
		Mat A2 = (Mat_<double>(3,4) <<
			f, 0, w/2, 0,
			0, f, h/2, 0,
			0, 0,   1, 0);

		// Final and overall transformation matrix
		Mat transfo = A2 * (T * (R * A1));

		// Apply matrix transformation
		warpPerspective(img, destination, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

		imshow(wndname2, destination);
		waitKey();


	 }
	
   return 0;
}
