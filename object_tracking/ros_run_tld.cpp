#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <perception_common/MultisenseImage.h>

using namespace cv;
using namespace std;
//Global variables
Rect box;
bool drawing_box = false;
bool gotBB = false;
bool tl = true;
bool rep = false;
bool fromfile=false;
string video ;

void readBB(char* file){
  ifstream bb_file (file);
  string line;
  getline(bb_file,line);
  istringstream linestream(line);
  string x1,y1,x2,y2;
  getline (linestream,x1, ',');
  getline (linestream,y1, ',');
  getline (linestream,x2, ',');
  getline (linestream,y2, ',');
  int x = atoi(x1.c_str());// = (int)file["bb_x"];
  int y = atoi(y1.c_str());// = (int)file["bb_y"];
  int w = atoi(x2.c_str())-x;// = (int)file["bb_w"];
  int h = atoi(y2.c_str())-y;// = (int)file["bb_h"];
  box = Rect(x,y,w,h);
}
//bounding box mouse callback
void mouseHandler(int event, int x, int y, int flags, void *param){
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
    gotBB = true;
    break;
  }
}

void print_help(char** argv){
  printf("use:\n     %s -p /path/parameters.yml\n",argv[0]);
  printf("-s    source video\n-b        bounding box file\n-tl  track and learn\n-r     repeat\n");
}

void read_options(int argc, char** argv,VideoCapture& capture,FileStorage &fs){
  for (int i=0;i<argc;i++){
      // if (strcmp(argv[i],"-b")==0){
      //     if (argc>i){
      //         readBB(argv[i+1]);
      //         gotBB = true;
      //     }
      //     else
      //       print_help(argv);
      // }
      // if (strcmp(argv[i],"-s")==0){
      //     if (argc>i){
      //         //video = string(argv[i+1]);
      //         video = "../datasets/12_cup/output.mpg";
      //         capture.open(video);
      //         fromfile = true;
      //     }
      //     else
      //       print_help(argv);

      // }
      // if (strcmp(argv[i],"-p")==0){
      //     if (argc>i){
      //         fs.open(argv[i+1], FileStorage::READ);
      //     }
      //     else
      //       print_help(argv);
      // }
      if (strcmp(argv[i],"-no_tl")==0){
          tl = false;
      }
      if (strcmp(argv[i],"-r")==0){
          rep = true;
      }
  }
}

int main(int argc, char * argv[]){
  //-----Init ROS----
  ros::init(argc, argv, "ros_opencv_example");
  ros::NodeHandle nh;
  drc_perception::MultisenseImage camera(nh);
  Mat image;


  while(ros::ok()){

        if(!camera.giveLeftColorImage(image))
        {
            ros::spinOnce();
            continue;
        }
          
        //image = imread( "/home/tairuichen/Desktop/lena.jpg" );
        //cv::Mat image = cv::imread("debris.jpg");
        if ( image.empty() ) {
                std::cout << "unable to load an input image\n";
                
                return -1;
               }
         

        // std::cout << "image: " << image.rows << ", " << image.cols << std::endl;
        // assert(image.type() == CV_8UC3);
        // cv::imshow("image", image);
        

  // VideoCapture capture;
  // capture.open(0);
  FileStorage fs;

  //Read options
  char *para_file = "../parameters.yml";
  fs.open(para_file, FileStorage::READ);

  // video = "../datasets/12_cup/output.mpg";
  // capture.open(video);
  fromfile = true;

  // char* bbfile = "../datasets/12_cup/init.txt";
  // readBB(bbfile);
  //gotBB = true;

//   read_options(argc,argv,capture,fs);
  //Init camera
//   if (!capture.isOpened())
//   {
// 	cout << "capture device failed to open!" << endl;
//     return 1;
//   }
//   //Register mouse callback to draw the bounding box
   cvNamedWindow("Tracking",CV_WINDOW_AUTOSIZE);
   cvSetMouseCallback( "Tracking", mouseHandler, NULL );
//   //TLD framework
   TLD tld;
// //   //Read parameters file
   tld.read(fs.getFirstTopLevelNode());
    Mat frame=image;
    Mat last_gray;
    Mat first;
 

  if (fromfile){
      //capture >> frame;
      cvtColor(frame, last_gray, CV_RGB2GRAY);
      frame.copyTo(first);
  }
   //  std::cout << "image: " << last_gray.rows << ", " << last_gray.cols << std::endl;
   // assert(last_gray.type() == CV_8UC3);
   // cv::imshow("image", last_gray);
//    else{
//       capture.set(CV_CAP_PROP_FRAME_WIDTH,340);
//       capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
//   }

//Initialization
GETBOUNDINGBOX:
  while(!gotBB)
  {
    // if (!fromfile){
    //   capture >> frame;
    // }
    // else
    first.copyTo(frame);

    cvtColor(frame, last_gray, CV_RGB2GRAY);
    drawBox(frame,box);
    imshow("Tracking", frame);
    if (cvWaitKey(33) == 'q')
	    return 0;
  }
  if (min(box.width,box.height)<(int)fs.getFirstTopLevelNode()["min_win"]){
      cout << "Bounding box too small, try again." << endl;
      gotBB = false;
      goto GETBOUNDINGBOX;
  }
  //Remove callback
  cvSetMouseCallback( "Tracking", NULL, NULL );
  printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n",box.x,box.y,box.width,box.height);
  //Output file
  FILE  *bb_file = fopen("bounding_boxes.txt","w");
  //TLD initialization
  tld.init(last_gray,box,bb_file);

  ///Run-time
  Mat current_gray;
  BoundingBox pbox;
  vector<Point2f> pts1;
  vector<Point2f> pts2;
  bool status=true;
  int frames = 1;
  int detections = 1;
  CvScalar box_color = cv::Scalar(255, 255, 0);
REPEAT:
  while(ros::ok()){

        if(!camera.giveLeftColorImage(image))
        {
            ros::spinOnce();
            continue;
        }

        if ( !image.empty() ) {


                
              




    frame = image;
    cvtColor(frame, current_gray, CV_RGB2GRAY);
    //Process Frame
    tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
    //Draw Points
    if (status){
      drawBox(frame,pbox,box_color);
      detections++;
    }
    //Display
    imshow("Tracking", frame);
    //swap points and images
    swap(last_gray,current_gray);
    pts1.clear();
    pts2.clear();
    frames++;
    printf("Detection rate: %d/%d\n",detections,frames);
    if (cvWaitKey(33) == 'q')
      break;


  }
  }
  if (rep){
    rep = false;
    tl = false;
    fclose(bb_file);
    bb_file = fopen("final_detector.txt","w");
    //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
    //capture.release();
    //capture.open(video);
    goto REPEAT;
  }
  fclose(bb_file);
        cv::waitKey(20);
        ros::spinOnce();
      }
      
  return 0;
}
