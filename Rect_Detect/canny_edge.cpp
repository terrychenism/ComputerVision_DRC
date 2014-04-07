//
// example of Canny edge detection
//
// http://code.google.com/p/robocraft-ros-pkg/
// http://robocraft.ru
//
 
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  
  IplImage *src = 0, *gray = 0, *canny = 0;
  try
  {
    src = bridge.imgMsgToCv(msg, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  if(src)
  {
    //ROS_INFO("get image %d x %d", src->width, src->height);
    gray = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
    canny = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
    
    cvCvtColor(src, gray, CV_RGB2GRAY);
    cvCanny(gray, canny, 10, 100, 3);
    
    cvShowImage("canny", canny);
    
    cvReleaseImage(&gray);
    cvReleaseImage(&canny);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocv_canny");
  ros::NodeHandle nh;
  cvNamedWindow("canny");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("canny");
}
