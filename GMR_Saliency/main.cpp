#include <cstring>
#include <stdio.h>
#include "GMRsaliency.h"
// #ifndef DWORD
// #define WINAPI
// typedef unsigned long DWORD;
// typedef short WCHAR;
// typedef void * HANDLE;
// #define MAX_PATH PATH_MAX
// typedef unsigned char BYTE;
// typedef unsigned short WORD;
// typedef unsigned int BOOL;
// #endif
using namespace cv;

Mat image0, image, gray, mask;
int ffillMode = 1;
int loDiff = 10, upDiff = 20;
int connectivity = 8;
int isColor = false;
bool useMask = false;
int newMaskVal = 255;
 
static void onMouse(int event, int x, int y, int, void*)
{
    if (event != CV_EVENT_LBUTTONDOWN)
        return;
 
    Point seed = Point(x, y);
    int lo = ffillMode == 0 ? 0 : loDiff;
    int up = ffillMode == 0 ? 0 : upDiff;
    int flags = connectivity + (newMaskVal << 8) +
        (ffillMode == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);
    int b = (unsigned)theRNG() & 255;
    int g = (unsigned)theRNG() & 255;
    int r = (unsigned)theRNG() & 255;
    Rect ccomp;
 
    Scalar newVal = isColor ? Scalar(0,0,0) : Scalar(r*0.299 + g*0.587 + b*0.114);
    Mat dst = image;
    int area = floodFill(dst, seed, newVal, &ccomp, Scalar(lo, lo, lo),
    Scalar(up, up, up), flags);

}


int main(int argc,char *argv[])
{
	vector<string> imnames;
	Mat sal,img;
	int count=1;
	const char* imname = "00001.jpg";


	image0 =imread(imname);
	image0.copyTo(image);
    cvtColor(image0, gray, CV_BGR2GRAY);

	namedWindow("image", 0);
 
    setMouseCallback("image", onMouse, 0);
    imshow("image", image);
 
    waitKey();
 
    Mat im = image;


	clock_t nTimeStart;
	clock_t nTimeStop;
	nTimeStart = clock();

	GMRsaliency GMRsal;
	sal=GMRsal.GetSal(im);

	nTimeStop = clock();
	cout <<"the average running time:"<<(double)(nTimeStop - nTimeStart)/CLOCKS_PER_SEC <<"sec"<< endl;
	//imshow("sal", sal);

	double adaptive_T = 2.0 * sum( sal )[0] / (sal.cols*sal.rows);
	while (sum( sal > adaptive_T )[0] == 0)
		adaptive_T /= 3;

	imshow( "T", sal > adaptive_T );




	cv::waitKey();
	
	return 0;
}

