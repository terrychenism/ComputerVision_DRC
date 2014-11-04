#include "saliency.h"
#include <cstdio>
#include <time.h>

using namespace cv;
using namespace std;
 
double alpha =2.0;
int beta =100;
Rect box;
bool drawing_box = false;
bool gotBB = false;
void drawBox(cv::Mat& image, CvRect box, cv::Scalar color = cvScalarAll(255), int thick = 1);
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
 
 
    //imshow("image", dst);
    //cout << area << " pixels were repainted\n";
}
 
void drawBox(Mat& image, CvRect box, Scalar color, int thick){
    rectangle(image, cvPoint(box.x, box.y), cvPoint(box.x + box.width, box.y + box.height), color, thick);
}
 
void mouseHandler(int event, int x, int y, int flags, void *param){
    switch (event){
    case CV_EVENT_MOUSEMOVE:
        if (drawing_box){
            box.width = x - box.x;
            box.height = y - box.y;
        }
        break;
    case CV_EVENT_LBUTTONDOWN:
        drawing_box = true;
        box = Rect(x, y, 0, 0);
        break;
    case CV_EVENT_LBUTTONUP:
        drawing_box = false;
        if (box.width < 0){
            box.x += box.width;
            box.width *= -1;
        }
        if (box.height < 0){
            box.y += box.height;
            box.height *= -1;
        }
        gotBB = true;
        break;
    }
}
 
 
int main( int argc, char * argv[] ) {
 
    image0 = imread("00001.jpg", 1);
 
    if (image0.empty())
    {
        cout << "Image empty.\n";
        return 0;
    }
 
    image0.copyTo(image);
    cvtColor(image0, gray, CV_BGR2GRAY);
    //mask.create(image0.rows + 2, image0.cols + 2, CV_8UC1);
 
    namedWindow("image", 0);
 
    setMouseCallback("image", onMouse, 0);
    imshow("image", image);
 
    waitKey();
 
    Mat im = image;

 	clock_t nTimeStart;
	clock_t nTimeStop;
	nTimeStart = clock();

     
    Saliency saliency;
    Mat_<float> sal = saliency.saliency( im );

    nTimeStop = clock();
	cout <<"the average running time:"<<(double)(nTimeStop - nTimeStart)/CLOCKS_PER_SEC <<"sec"<< endl;
     
    double adaptive_T = 3.0 * sum( sal )[0] / (sal.cols*sal.rows);
    while (sum( sal > adaptive_T )[0] == 0)
        adaptive_T /= 1.2;
     
 
    imshow( "sal", sal );
    //imshow( "T", sal > adaptive_T );
    waitKey();
     
    return 0;
}