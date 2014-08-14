#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <dirent.h>

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include "BrowseDir.h"
#include "StatDir.h"
#include "Prehelper.h"

using namespace std;
using namespace cv;

#define K 3 // 3 class image 
#define MAX_PATH_LENGTH     512
#define MAX_FILE_EXTENSION  9

unsigned long visit_dirs = 0;
unsigned long visit_files = 0;
    vector<char*> sub_path;
    vector<char*> file_names;
    vector<Mat> test_images;

string VIDEO_PATH = "test.avi";
char dir[256] = "g:\\obj\\";
char* IMG_NAME = "2.jpg";

void listdir(char *path){
    DIR         *ptr_dir;
    struct dirent   *dir_entry;
    int         i = 0;
    char        *child_path;
    char        *file_path;


    child_path = (char*)malloc(sizeof(char)*MAX_PATH_LENGTH);
    if(child_path == NULL){
        printf("allocate memory for path failed.\n");
        return;
    }
    memset(child_path, 0, sizeof(char)*MAX_PATH_LENGTH);

    file_path = (char*)malloc(sizeof(char)*MAX_PATH_LENGTH);
    if(file_path == NULL){
        printf("allocate memory for file path failed.\n");
        free(child_path);
        child_path = NULL;
        return;
    }
    memset(file_path, 0, sizeof(char)*MAX_PATH_LENGTH);

    ptr_dir = opendir(path);
    while((dir_entry = readdir(ptr_dir)) != NULL){
        if(dir_entry->d_type & DT_DIR){
            if(strcmp(dir_entry->d_name,".") == 0 ||
               strcmp(dir_entry->d_name,"..") == 0){
                continue;
            }

            sprintf(child_path, "%s/%s", path, dir_entry->d_name);
            printf("[DIR]%s\n", child_path);


            sub_path.push_back(child_path);
             cout << sub_path.back() << endl;
             string name = child_path;
               
            visit_dirs++;

            listdir(child_path);
        }

        if(dir_entry->d_type & DT_REG){
            sprintf(file_path, "%s/%s", path, dir_entry->d_name);
            printf("[FILE]%s\n", file_path);
            file_names.push_back(file_path);
            Mat v  =   imread(file_path );
            test_images.push_back(v);
            visit_files++;
        }
    }

    free(child_path);
    child_path = NULL;

    free(file_path);
    file_path = NULL;
}





int main( int argc, char** argv )
{  
   
	VideoCapture mCapture(VIDEO_PATH);
	if (!mCapture.isOpened())
	{		
		system("pause");
		return 1;
	}

    Mat frame, frameCopy, image;  
    string inputName;     
    int mode;  
	IplImage* standard2 = cvLoadImage(IMG_NAME,CV_LOAD_IMAGE_GRAYSCALE);
    
	vector<Mat> images,testimages;  
    vector<int> labels,testlabels;
	Mat testimage;
	int testlabel = 3;
	string mdir = dir;
	
	
	char id[5];
	mdir.append("test\\");
	//_itoa(1,id,10);

	snprintf(id, sizeof(id), "%d", 1);
	mdir.append(id);
	// const char* dd = mdir.c_str();
	char * dd = new char [mdir.length()+1];
  	strcpy (dd, mdir.c_str());
	// CStatDir state;
	// if(!state.SetInitDir(dd)){
	// 	puts("dir not exist");
	// }
	// vector<char*>file_vec = state.BeginBrowseFilenames("*.*");

	// for(int m = 0; m < file_vec.size();m++){
	// 	IplImage* cur_img = cvLoadImage(file_vec[m],CV_LOAD_IMAGE_GRAYSCALE);
	// 	cvResize(cur_img,standard2,CV_INTER_AREA);
	// 	Mat cur_mat = cvarrToMat(standard2,true),des_mat;
	// 	cv::normalize(cur_mat,des_mat,0, 255, NORM_MINMAX, CV_8UC1);
	// 	IplImage mat1 = des_mat;
	// 	cvSaveImage(file_vec[m],cvCloneImage(&mat1));

	// 	if(m!=file_vec.size())
	// 	{
	// 		testimages.push_back(des_mat);
	// 		testlabels.push_back(m+1);
	// 	}		
	// }
	listdir("/home/tairuichen/Desktop/data/test/");
	

	Mat mframe;
	Mat last_gray;
	
    Mat standard3 = imread(IMG_NAME,CV_LOAD_IMAGE_GRAYSCALE);
    int mlabel = 1;
    // preprocess test images 
        while(test_images.size() > 0){
        	mframe = test_images.back();

	        cvtColor(mframe, last_gray, CV_RGB2GRAY);
	        resize(last_gray,last_gray,standard3.size());
	        Mat des_mat;
	        cv::normalize(last_gray,des_mat,0, 255, NORM_MINMAX, CV_8UC1);

			testimages.push_back(des_mat);
          	testlabels.push_back(mlabel);
          	mlabel++;
            test_images.pop_back();
            
        }
    // preprocess training images
        test_images.clear();
        cout << test_images.size()<<endl;
        listdir("/home/tairuichen/Desktop/data/train/");
        while(sub_path.size() > 1){
        	listdir(sub_path.back());
        	printf("Total DIR: %ld, Total FILE: %ld\n", sub_path.size(), test_images.size());
        	test_images.clear();
        	while(sub_path.size() > 0){
	        	mframe = test_images.back();

		        cvtColor(mframe, last_gray, CV_RGB2GRAY);

		        resize(last_gray,last_gray,standard3.size());
		        Mat des_mat_train;
		        cv::normalize(last_gray,des_mat_train,0, 255, NORM_MINMAX, CV_8UC1);

				images.push_back(des_mat_train);
	          	labels.push_back(mlabel);
	          	mlabel++;
	            test_images.pop_back();
	        }
	        sub_path.pop_back();
	        waitKey();
        }

        printf("Total DIR: %ld, Total FILE: %ld\n", sub_path.size(), test_images.size());
		while(test_images.size() > 0){
		 	imshow("asdasd" , test_images.back());
		 	test_images.pop_back();
		 	waitKey();
		}
			
    //to gray, normalize and resize; load to images,labels,testimages,testlabels  
    //resizeandtogray(dir,K,images,labels,testimages,testlabels);   
 //    // =========== training ============ 
 //    Ptr<FaceRecognizer> model = Recognition(images,labels,testimages,testlabels); 

	// // ======= prediction =============
	// Mat mframe;
	// Mat last_gray;
	// mCapture >> mframe;
	// while(mCapture.read(mframe)){
	// 	imshow("Tracking", mframe);
	// 	cvtColor(mframe, last_gray, CV_RGB2GRAY);
	// 	// IplImage* cur_img=cvCloneImage(&(IplImage)last_gray);
	// 	IplImage iplimg = last_gray;
	// 	IplImage *cur_img =&iplimg;  
	// 	cvResize(cur_img,standard2,CV_INTER_AREA);
	// 	Mat cur_mat = cvarrToMat(standard2,true),des_mat;
	// 	cv::normalize(cur_mat,des_mat,0, 255, NORM_MINMAX, CV_8UC1);
	// 	testimage = des_mat;
		
	// 	Prediction(testimage,testlabel,model);  
	// 	if ( cvWaitKey(30) == 'q')
	// 		return 0;
	// 	if ( cvWaitKey(30) == 's')
	// 		break;
	// }
    //char* dirmodel = new char [256];  
    //strcpy(dirmodel,dir); strcat(dirmodel,"model.out");  
    //FILE* f = fopen(dirmodel,"w");  
    //fwrite(model,sizeof(model),1,f);  


    //return 0;  
}    
