#include <stdio.h>
#include <iostream>
#include <dirent.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;
#define MAX_PATH_LENGTH     512
#define MAX_FILE_EXTENSION  9

unsigned long visit_dirs = 0;
unsigned long visit_files = 0;
   
    vector<char*> sub_path;
    vector<char*> file_names;
    vector<Mat> images;

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
            images.push_back(v);
            visit_files++;
        }
    }

    free(child_path);
    child_path = NULL;

    free(file_path);
    file_path = NULL;
}

int main(int argc, char* argv[]){

        listdir("/home/tairuichen/Desktop/data/");
        printf("Total DIR: %ld, Total FILE: %ld\n", sub_path.size(), file_names.size());

        vector<char*> v;
        v.push_back("asdasd");
        v.push_back("asdadsadsdasda");
        
        while(images.size() > 0){
            imshow("asdasd" , images.back());
            images.pop_back();
                waitKey();
        }
       
    }
    
    
// int main(int argc, char *argv[])
// {
//      List("/home/tairuichen/Desktop/data/");
//      return 0;
// }
