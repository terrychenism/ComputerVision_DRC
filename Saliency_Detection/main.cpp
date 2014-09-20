#include <cstring>
#include <stdio.h>
#include <time.h>
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

Rect box;
bool drawing_box = false;
bool gotBB = false;
void drawBox(cv::Mat& image, CvRect box, cv::Scalar color = cvScalarAll(255), int thick=1); 


class WatershedSegmenter{
private:
	cv::Mat markers;
public:
	void setMarkers(cv::Mat& markerImage)
	{
		markerImage.convertTo(markers, CV_32S);
	}

	cv::Mat process(cv::Mat &image)
	{
		cv::watershed(image, markers);
		markers.convertTo(markers,CV_8U);
		return markers;
	}
};

void drawBox(Mat& image, CvRect box, Scalar color, int thick){
	rectangle( image, cvPoint(box.x, box.y), cvPoint(box.x+box.width,box.y+box.height),color, thick);
} 

void mouseHandler(int event, int x, int y, int flags, void *param){
	//Mat* tmp = static_cast<Mat*>(param);
	switch( event ){
	case CV_EVENT_MOUSEMOVE:
		if (drawing_box){
			box.width = x-box.x;
			box.height = y-box.y;
		}
		break;
	case CV_EVENT_LBUTTONDOWN:
		drawing_box = true;
		box = Rect( x, y, 0, 0 );
		break;
	case CV_EVENT_LBUTTONUP:
		drawing_box = false;
		if( box.width < 0 ){
			box.x += box.width;
			box.width *= -1;
		}
		if( box.height < 0 ){
			box.y += box.height;
			box.height *= -1;
		}
		//drawBox(*tmp,box,cv::Scalar(255, 255, 0));
		gotBB = true;
		break;
	}
}


int main(int argc,char *argv[])
{
	vector<string> imnames;
	Mat sal,image, dest;
	int count=1;
	const char* imname = "000011.jpg";
	image=imread(imname);
	
	/* ==================== */
	// Create markers 
	cv::Mat markers(image.size(),CV_8U,cv::Scalar(-1));
	//top 
	markers(Rect(0,0,image.cols, 5)) = Scalar::all(1);
	//bottom 
	markers(Rect(0,image.rows-5,image.cols, 5)) = Scalar::all(1);
	//left 
	markers(Rect(0,0,5,image.rows)) = Scalar::all(1);
	//right 
	markers(Rect(image.cols-5,0,5,image.rows)) = Scalar::all(1);
	//mouse control
	int centreW = image.cols/4;
	int centreH = image.rows/4;
	cvNamedWindow("Tracking",CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback( "Tracking", mouseHandler, NULL );
	drawBox(image,box);
	imshow("Tracking", image);
	waitKey();
	markers(box) = Scalar::all(2);
	// markers(Rect((image.cols/2)-(centreW/2),(image.rows/2)-(centreH/2), 100, 150)) = Scalar::all(2);
	markers.convertTo(markers,CV_BGR2GRAY);
	imshow("markers", markers);
	// waitKey();
	//Create watershed segmentation object
	WatershedSegmenter segmenter;
	segmenter.setMarkers(markers);
	cv::Mat wshedMask = segmenter.process(image);
	cv::Mat mask;
	convertScaleAbs(wshedMask, mask, 1, 0);
	double thresh = threshold(mask, mask, 1, 255, THRESH_BINARY);
	bitwise_and(image, image, dest, mask);
	dest.convertTo(dest,CV_8U);
	imshow("mid_result", dest);

	/* =========================== */ 
	clock_t nTimeStart;      
    clock_t nTimeStop;       
    nTimeStart = clock();

	
		
	GMRsaliency GMRsal;
	sal=GMRsal.GetSal(dest);

	
	nTimeStop = clock();
	cout <<"the average running time:"<<(double)(nTimeStop - nTimeStart)<<"seconds"<< endl;
	//imwrite("sal.jpg",sal*255);
	imshow("saliency_result", sal);
	cv::waitKey();
	
	return 0;
}


