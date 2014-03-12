#include "Prehelper.h"
#include "BrowseDir.h"
#include "StatDir.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
using namespace cv;

void normalizeone(const char* dir,IplImage* standard)
{
	CStatDir statdir;
	if (!statdir.SetInitDir(dir))
	{
		puts("Dir not exist");
		return;
	}
	vector<char*>file_vec = statdir.BeginBrowseFilenames("*.*");
	int i;
	for (i=0;i<file_vec.size();i++)
	{
		IplImage* cur_img = cvLoadImage(file_vec[i],CV_LOAD_IMAGE_GRAYSCALE);
		//IplImage*cur_gray = cvCreateImage(cvGetSize(cur_img),cur_img->depth,1);
		cvResize(cur_img,standard,CV_INTER_AREA);
		//cvCvtColor(standard,cur_gray,CV_RGB2GRAY);
		// 		cvNamedWindow("cur_img",CV_WINDOW_AUTOSIZE);
		// 		cvNamedWindow("standard",CV_WINDOW_AUTOSIZE);
		// 		cvShowImage("cur_img",cur_img);
		// 		cvShowImage("standard",standard);
		// 		cvWaitKey();
		cvSaveImage(file_vec[i],cur_img);
	}
}

void CutImg(IplImage* src, CvRect rect,IplImage* res)
{
	CvSize imgsize;
	imgsize.height = rect.height;
	imgsize.width = rect.width;
	cvSetImageROI(src,rect);
	cvCopy(src,res);
	cvResetImageROI(res);
}

int read_img(const string& dir, vector<Mat> &images)
{
	CStatDir statdir;
	if (!statdir.SetInitDir(dir.c_str()))
	{
		cout<<"Direct "<<dir<<"  not exist!"<<endl;
		return 0;
	}
	int cls_id = dir[dir.length()-1]-'0';
	vector<char*>file_vec = statdir.BeginBrowseFilenames("*.*");
	int i,s = file_vec.size();
	for (i=0;i<s;i++)
	{
		Mat graymat = imread(file_vec[i],0);
		//graymat.reshape(1,1);//flatten to one row
		images.push_back(graymat);
	}
	return s;
}

vector<pair<char*,Mat>>  read_img(const string& dir)
{
	CStatDir statdir;
	pair<char*,Mat> pfi;
	vector<pair<char*,Mat>> Vp;
	if (!statdir.SetInitDir(dir.c_str()))
	{
		cout<<"Direct "<<dir<<"  not exist!"<<endl;
		return Vp;
	}
	int cls_id = dir[dir.length()-1]-'0';
	vector<char*>file_vec = statdir.BeginBrowseFilenames("*.*");
	int i,s = file_vec.size();
	for (i=0;i<s;i++)
	{
		pfi.first = file_vec[i];
		pfi.second = imread(file_vec[i]);
		Vp.push_back(pfi);
	}
	return Vp;
}

vector<Rect> detectAndDraw( Mat& img, CascadeClassifier& cascade,
	CascadeClassifier& nestedCascade,
	double scale, bool tryflip, bool draw )
{
	int i = 0;
	double t = 0;
	vector<Rect> faces, faces2;
	const static Scalar colors[] =  { CV_RGB(0,0,255),
		CV_RGB(0,128,255),
		CV_RGB(0,255,255),
		CV_RGB(0,255,0),
		CV_RGB(255,128,0),
		CV_RGB(255,255,0),
		CV_RGB(255,0,0),
		CV_RGB(255,0,255)} ;
	Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

	cvtColor( img, gray, CV_BGR2GRAY );
	resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
	equalizeHist( smallImg, smallImg );

	t = (double)cvGetTickCount();
	cascade.detectMultiScale( smallImg, faces,
		1.1, 2, 0
		|CV_HAAR_FIND_BIGGEST_OBJECT
		//|CV_HAAR_DO_ROUGH_SEARCH
		//|CV_HAAR_SCALE_IMAGE
		,
		Size(30, 30) );
	if( tryflip )
	{
		flip(smallImg, smallImg, 1);
		cascade.detectMultiScale( smallImg, faces2,
			1.1, 2, 0
			|CV_HAAR_FIND_BIGGEST_OBJECT
			//|CV_HAAR_DO_ROUGH_SEARCH
			//|CV_HAAR_SCALE_IMAGE
			,
			Size(30, 30) );
		for( vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); r++ )
		{
			faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
		}
	}
	t = (double)cvGetTickCount() - t;
	printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
	if(draw)
	{
		for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
		{
			Mat smallImgROI;
			vector<Rect> nestedObjects;
			Point center;
			Scalar color = colors[i%8];
			int radius;

			double aspect_ratio = (double)r->width/r->height;
			rectangle( img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
				cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
				color, 3, 8, 0);
			if( nestedCascade.empty() )
				continue;
			smallImgROI = smallImg(*r);
			nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
				1.1, 2, 0
				|CV_HAAR_FIND_BIGGEST_OBJECT
				//|CV_HAAR_DO_ROUGH_SEARCH
				//|CV_HAAR_DO_CANNY_PRUNING
				//|CV_HAAR_SCALE_IMAGE
				,
				Size(30, 30) );
			//draw eyes
			//         for( vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
			//         {
			//             center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
			//             center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
			//             radius = cvRound((nr->width + nr->height)*0.25*scale);
			//             circle( img, center, radius, color, 3, 8, 0 );
			//         }
		}
		cv::imshow( "result", img );
	}
	return faces;
}

void resizeandtogray(char* dir,int K, vector<Mat> &images, vector<int> &labels,vector<Mat> &testimages, vector<int> &testlabels)
{
	IplImage* standard = cvLoadImage("C:\\Users\\Terry\\Desktop\\face\\2.jpg",CV_LOAD_IMAGE_GRAYSCALE);
	string cur_dir;
	char id[5];
	int i,j;
	for(int i=1; i<=K; i++)
	{
		cur_dir = dir;
		cur_dir.append("color\\");	
		_itoa(i,id,10);
		cur_dir.append(id);
		const char* dd = cur_dir.c_str();
		CStatDir statdir;
		if (!statdir.SetInitDir(dd))
		{
			puts("Dir not exist");
			return;
		}
		cout<<"Processing samples in Class "<<i<<endl;
		vector<char*>file_vec = statdir.BeginBrowseFilenames("*.*");
		for (j=0;j<file_vec.size();j++)
		{
			IplImage* cur_img = cvLoadImage(file_vec[j],CV_LOAD_IMAGE_GRAYSCALE);
			cvResize(cur_img,standard,CV_INTER_AREA);
			Mat cur_mat = cvarrToMat(standard,true),des_mat;
			cv::normalize(cur_mat,des_mat,0, 255, NORM_MINMAX, CV_8UC1);
			cvSaveImage(file_vec[j],cvCloneImage(&(IplImage) des_mat));
			if(j!=file_vec.size())
			{
					images.push_back(des_mat);
					labels.push_back(i);
			}
			else
			{
				testimages.push_back(des_mat);
				testlabels.push_back(i);
			}
		}
		cout<<file_vec.size()<<" images."<<endl;
	}
}

Ptr<FaceRecognizer> Recognition(vector<Mat> images, vector<int> labels, vector<Mat> testimages, vector<int> testlabels)  
{  
    Ptr<FaceRecognizer> model = createEigenFaceRecognizer(10);//10 Principal components  
    cout<<"train"<<endl;  
    model->train(images,labels);  
    int i,acc=0,predict_l;  
    for (i=0;i<testimages.size();i++)  
    {  
        predict_l = model->predict(testimages[i]);  
        if(predict_l != testlabels[i])  
        {  
            cout<<"An error in recognition: sample "<<i+1<<", predict "<<  
                predict_l<<", groundtruth "<<testlabels[i]<<endl;  
            imshow("error 1",testimages[i]);  
            waitKey();  
        }  
        else  
            acc++;  
    }  
    cout<<"Recognition Rate: "<<acc*1.0/testimages.size()<<endl;  
    return model;  
}  

IplImage* DetectandExtract(Mat& img, CascadeClassifier& cascade,
	CascadeClassifier& nestedCascade,
	double scale, bool tryflip)
{
	vector<Rect> Rvec = detectAndDraw(img,cascade,nestedCascade,scale,tryflip,0);
	int i,maxxsize=0,id=-1,area;
	for (i=0;i<Rvec.size();i++)
	{
		area = Rvec[i].width*Rvec[i].height;
		if(maxxsize<area)
		{
			maxxsize = area;
			id = i;
		}
	}
	IplImage* transimg = cvCloneImage(&(IplImage)img);
	if(id!=-1)
	{
		CvSize imgsize;
		imgsize.height = Rvec[id].height;
		imgsize.width = Rvec[id].width;
		IplImage* res = cvCreateImage(imgsize,transimg->depth,transimg->nChannels);
		CutImg(transimg,Rvec[id],res);

		return res;
	}
	return NULL;
}