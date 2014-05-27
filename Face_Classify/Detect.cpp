#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include "BrowseDir.h"
#include "StatDir.h"
#include "Prehelper.h"

using namespace std;
using namespace cv;
#define CAM 2
#define PHO 1
#define K 4

string cascadeName = "C:/Users/Terry/Downloads/opencv/sources/data/haarcascades/haarcascade_frontalface_alt.xml";
string nestedCascadeName = "C:/Users/Terry/Downloads/opencv/sources/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

//int main( )
//{
//	CvCapture* capture = 0;
//	Mat frame, frameCopy, image;
//	string inputName;
//	bool tryflip = false;
//	int mode;
//	CascadeClassifier cascade, nestedCascade; 
//	double scale = 1.0;
//	if( !cascade.load( cascadeName ) ||!nestedCascade.load( nestedCascadeName))
//	{
//		cerr << "ERROR: Could not load classifier cascade or nestedCascade" << endl;
//		return -1;
//	}
//
//// 	printf("select the mode of detection: \n1: from picture\t 2: from camera\n");
//// 	scanf("%d",&mode);
//	char** pics = (char**) malloc(sizeof*pics);
//
//	/************************************************************************/
//	/*                                  detect face and save                                    */
//	/************************************************************************/
//	int i,j;
//	cout<<"detect and save..."<<endl;
//	char dir[256] = "C:\\Users\\Terry\\Desktop\\face\\";
//	string cur_dir;  
//    char id[5];  
//    for(i=1; i<=K; i++)  
//    {  
//        cur_dir = dir;  
//        _itoa(i,id,10);  
//        cur_dir.append("color\\");  
//        cur_dir.append(id);  
//        vector<pair<char*,Mat>> imgs=read_img(cur_dir);  
//        for(j=0;j<imgs.size();j++)  
//        {  
//            IplImage* res = DetectandExtract(imgs[j].second,cascade,nestedCascade,scale,tryflip);  
//            if(res)  
//                cvSaveImage(imgs[j].first,res);  
//        }  
//    } 
//	system("pause"); 
//    return 0;  
//}  

int main( )  
{  
    CvCapture* capture = 0;  
    Mat frame, frameCopy, image;  
    string inputName;     
    int mode;  
	IplImage* standard2 = cvLoadImage("D:\\face\\2.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    char dir[256] = "D:\\face\\";
	vector<Mat> images,testimages;  
    vector<int> labels,testlabels;
	string mdir = dir;
	
	
	char id[5];
	mdir.append("test\\");
	_itoa(1,id,10);
	mdir.append(id);
	const char* dd = mdir.c_str();
	CStatDir state;
	if(!state.SetInitDir(dd)){
		puts("dir not exist");
	}
	vector<char*>file_vec= state.BeginBrowseFilenames("*.*");

	for(int m = 0; m< file_vec.size();m++){
		IplImage* cur_img = cvLoadImage(file_vec[m],CV_LOAD_IMAGE_GRAYSCALE);
		cvResize(cur_img,standard2,CV_INTER_AREA);
		Mat cur_mat = cvarrToMat(standard2,true),des_mat;
		cv::normalize(cur_mat,des_mat,0, 255, NORM_MINMAX, CV_8UC1);
		cvSaveImage(file_vec[m],cvCloneImage(&(IplImage) des_mat));

		if(m!=file_vec.size())
		{
			testimages.push_back(des_mat);
			testlabels.push_back(m+1);
		}		
	}

			
    //togray, normalize and resize; load to images,labels,testimages,testlabels  
    resizeandtogray(dir,K,images,labels,testimages,testlabels);   
    //recognition  
    Ptr<FaceRecognizer> model = Recognition(images,labels,testimages,testlabels);  
    char* dirmodel = new char [256];  
    strcpy(dirmodel,dir); strcat(dirmodel,"model.out");  
    FILE* f = fopen(dirmodel,"w");  
    fwrite(model,sizeof(model),1,f);  
    system("pause");  
    return 0;  
}    
