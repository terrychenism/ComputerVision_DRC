
#include <iostream>
#include <fstream>

/*** INCLUDE FILES ***/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <perception_common/MultisenseImage.h>
#include <perception_common/ImageHelper.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>


using namespace std;
using namespace cv;
using namespace drc_perception;

#define CUSTOM_REPROJECT



boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}





cv::Mat raw_bgr_image;
void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    raw_bgr_image = cv_ptr->image;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert aaaaaaaaaaaa from '%s' to 'BGR8'.", msg->encoding.c_str());
  }
}

// cv::Mat disparity_image;
// void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
// {
//   cv_bridge::CvImagePtr cv_ptr;
//   try
//   {
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     disparity_image = cv_ptr->image;

//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("Could not convert bbbbbbbbbbbbbbb from '%s' to 'BGR8'.", msg->encoding.c_str());
//   }
// }


using namespace drc_perception;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ReprojectImageToPointCloud");
  ros::NodeHandle nh("~");

  drc_perception::MultisenseImage image_assistance(nh);

  StereoPointCloudColor::Ptr cloud(new StereoPointCloudColor);
  cv::Mat color;
  cv::Mat_<float> disp;
  cv::Mat_<double> Q;

  bool valid_Q=false;
  bool new_color=false;
  bool new_disp=false;

  while(ros::ok())
  {
    if(image_assistance.giveLeftColorImage(color))
    {
      new_color=true;
    }
    if(image_assistance.giveDisparityImage(disp))
    {
      new_disp=true;
    }

    if(new_disp&&new_color)
    {
      if(!image_assistance.giveQMatrix(Q))
      {
        ros::spinOnce();
        continue;
      }
      std::cout<<disp.cols<<" x "<<disp.rows<<std::endl;
      PointCloudHelper::generateOrganizedRGBDCloud(disp,color,Q,cloud);
      pcl::PCLPointCloud2 output;
      pcl::toPCLPointCloud2(*cloud,output);
      output.header.frame_id=std::string("left_camera_optical_frame");
      output.header.stamp = ros::Time::now().toNSec();


      new_disp=new_color=false;

      //Create visualizer
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = createVisualizer( cloud );
      
      //loop
      while ( !viewer->wasStopped())
      {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

    }
      ros::spinOnce();
    }

}




