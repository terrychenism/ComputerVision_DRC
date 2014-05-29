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



#include <sensor_msgs/Image.h>
#include <unistd.h>
#include <perception_common/global.h>
#include <sensor_msgs/image_encodings.h>

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
      if (strcmp(argv[i],"-b")==0){
          if (argc>i){
              readBB(argv[i+1]);
              gotBB = true;
          }
          else
            print_help(argv);
      }
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


cv::Mat g_raw_bgr_image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    g_raw_bgr_image = cv_ptr->image;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'BGR8'.", msg->encoding.c_str());
  }
}


int main(int argc, char * argv[]){
  //-----Init ROS----
  ros::init(argc, argv, "ros_opencv_example");
  // ros::NodeHandle nh;

  ros::NodeHandle nh("~");
  std::string image_name, config_file, init_bb, box_file;
  nh.getParam("image_name",image_name);
  nh.getParam("config_file", config_file);
  nh.getParam("init_bb", init_bb);
  nh.getParam("box_file", box_file);

  // if (!nh.getParam("image_name", image_name)){
  //   ROS_WARN("Parameter <%s> Not Set. Using Default Value of <%s>!",
  //   "image_name", image_name.c_str());
  //   image_name = "/multisense_sl/left/image_rect_color" ;
  // }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(image_name, 1, imageCallback);




  drc_perception::MultisenseImage camera(nh);
  Mat image;

  //================for test image- ===================
    // while(ros::ok()){
    //   if(g_raw_bgr_image.empty()){
    //     ROS_INFO("agsagfsasasfajsifjlasghkdjshgdkasjgladsjgdagdkasgjdasgjkldasgjasgagldkasjgdskagj"); 
    //   }
    //   else  
    //     imshow( "Original image", g_raw_bgr_image);

    //   waitKey(20);
    //   ros::spinOnce();
    // }

//===================== end ======================= 

  while(ros::ok()){

    // if(g_raw_bgr_image.empty()){
    //   ROS_INFO("agsagfsasasfajsifjlasghkdjshgdkasjgladsjgdagdkasgjdasgjkldasgjasgagldkasjgdskagj"); 

    // }
    // else  
    //   imshow( "Original image", g_raw_bgr_image);

    // if(!camera.giveLeftColorImage(image))
    // {
    //     ros::spinOnce();
    //     continue;
    // }
    image = g_raw_bgr_image;
    if(image.empty()){
          ros::spinOnce();
          continue;
    }

    else{



        
    // if ( image.empty() ) {
    //    std::cout << "unable to load an input image\n";               
    //    return -1;
    // }

    //if ( !image.empty()) {
          
      //imshow( "Original image", image);

      // VideoCapture capture;
      // capture.open(0);
      FileStorage fs;
    
      //Read options
      // char *para_file = "src/perception_tracking/parameters.yml";
      // fs.open(para_file, FileStorage::READ);

      char* para_file = new char[config_file.length() + 1];
      strcpy(para_file, config_file.c_str());
      fs.open(para_file, FileStorage::READ);
      delete [] para_file;
      // video = "../datasets/12_cup/output.mpg";
      // capture.open(video);
      fromfile = true;

      // char* bbfile = "src/perception_tracking/src/init.txt";
      // readBB(bbfile);

      char* bbfile = new char[init_bb.length() + 1];
      strcpy(bbfile, init_bb.c_str());
      readBB(bbfile);
      delete [] bbfile;

      gotBB = true;

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
      //cvSetMouseCallback( "Tracking", NULL, NULL );
      printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n",box.x,box.y,box.width,box.height);

      //Output file
      //FILE  *bb_file = fopen("src/perception_tracking/bounding_boxes.txt","w");

      //FILE  *bb_file = fopen(box_file,"w");
      char* BoundBox = new char[box_file.length() + 1];
      strcpy(BoundBox, box_file.c_str());
      FILE  *bb_file = fopen(BoundBox,"w");
      delete [] BoundBox;

      ROS_INFO("agsagfsasasfajsifjlasghkdjshgdkasjgladsjgdagdkasgjdasgjkldasgjasgagldkasjgdskagj"); 
      //TLD initialization
      tld.init(last_gray,box,bb_file);

      // ROS_INFO("agsagfsasasfajsifjlasghkdjshgdkasjgladsjgdagdkasgjdasgjkldasgjasgagldkasjgdskagj"); 
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
      
            // if(!camera.giveLeftColorImage(image))
            // {
            //     ros::spinOnce();
            //     continue;
            // }
        image = g_raw_bgr_image;
        if(image.empty()){
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
        ros::spinOnce();
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

    }  // else end

    cv::waitKey(20);
    ros::spinOnce();
  }
      
  return 0;
}
