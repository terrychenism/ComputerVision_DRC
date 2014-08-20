
#include <iostream>
#include <fstream>

/*** INCLUDE FILES ***/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


#include <boost/thread/thread.hpp>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <perception_common/MultisenseImage.h>
#include <perception_common/ImageHelper.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>


using namespace std;
using namespace cv;
using namespace drc_perception;

#define CUSTOM_REPROJECT

Rect box;
bool drawing_box = false;
bool gotBB = false;

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

void rotate_image(cv::Mat &src, cv::Mat &dst, int angle)
{
   if(src.data != dst.data){
       src.copyTo(dst);
   }

   angle = ((angle / 90) % 4) * 90;

   bool const flip_horizontal_or_vertical = angle > 0 ? 1 : 0;
   int const number = std::abs(angle / 90);

   for(int i = 0; i != number; ++i){
       cv::transpose(dst, dst);
       cv::flip(dst, dst, flip_horizontal_or_vertical);
   }
}
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ReprojectImageToPointCloud");
  ros::NodeHandle nh("~");

  drc_perception::MultisenseImage image_assistance(nh);

  StereoPointCloudColor::Ptr cloud(new StereoPointCloudColor);
  StereoPointCloudColor::Ptr target_cloud(new StereoPointCloudColor);
  cv::Mat color;
  cv::Mat color_help;
  cv::Mat_<float> disp_help;
  cv::Mat_<float> disp;
  cv::Mat_<double> Q;

  bool valid_Q=false;
  bool new_color=false;
  bool new_disp=false;

  cvNamedWindow("showImage",CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback( "showImage", mouseHandler, NULL );

  ros::Publisher debug_publisher = nh.advertise<pcl::PCLPointCloud2> ("/Point_cloud/RGBD", 1);



  while(ros::ok())
  {
    if(image_assistance.giveLeftColorImage(color_help))
    {
      new_color=true;

    }
    if(image_assistance.giveDisparityImage(disp_help))
    {
      new_disp=true;
    }

    
    
    if(new_disp&&new_color)
    {
      rotate_image(color_help, color, 180); // rotate the image
      rotate_image(disp_help, disp,180); // rotate the image
      imshow("showImage", color);
       
      if(!image_assistance.giveQMatrix(Q))
      {
        ros::spinOnce();
        continue;
      }
      std::cout<<disp.cols<<" x "<<disp.rows<<std::endl;
      //PointCloudHelper::generateOrganizedRGBDCloud(disp,color,Q,cloud);

    cv::Mat xyz;
    int width=disp.cols;
    int height=disp.rows;
    cloud->clear();
    cloud->resize(width*height);
    cloud->height=height;
    cloud->width=width;
    target_cloud->clear();
    target_cloud->resize(width*height);
    target_cloud->height=height;
    target_cloud->width=width;

    pcl::PointCloud<pcl::PointXYZ> cloudnn ;
    cloudnn.width = 544;
    cloudnn.height =1024;
    cloudnn.is_dense = false;
    cloudnn.points.resize(cloudnn.width * cloudnn.height);
    size_t i = 0;

    cv::reprojectImageTo3D(disp, xyz, Q, false);


    for(int u=0;u<disp.rows;u++)
      for(int v=0;v<disp.cols;v++)
      {
        if(disp.at<float>(cv::Point(v,u))==0.0)
          continue;
        cv::Vec3f cv_pt=xyz.at<cv::Vec3f>(cv::Point(v,u));
        drc_perception::StereoPointColor pt;
        pt.x=cv_pt.val[0];
        pt.y=cv_pt.val[1];
        pt.z=cv_pt.val[2];

        cloudnn.points[i].x=cv_pt.val[0];
        cloudnn.points[i].y=cv_pt.val[1];
        cloudnn.points[i].z=cv_pt.val[2];
        i++;
        //std::cout<<"Z value: "<< cv_pt.val[2]<<std::endl;
        cv::Vec3b rgb=color.at<cv::Vec3b>(cv::Point(v,u));
        pt.b=rgb.val[0];
        // pt.g=rgb.val[1];
        // pt.r=rgb.val[2];
        cloud->at(v,u)=pt;
        pt.r=rgb.val[2];
        target_cloud->at(v,u)=pt;

      }
      




      ROS_INFO_STREAM("Organized cloud size: "<<cloud->size());
      
      pcl::PCLPointCloud2 output;
      pcl::toPCLPointCloud2(*cloud,output);
      output.header.frame_id=std::string("left_camera_optical_frame");
      output.header.stamp = ros::Time::now().toNSec();
      debug_publisher.publish(output);
      new_disp=new_color=false; 



      // drc_perception::StereoPointColor ptr;
      // ptr = cloud->at(100,100);
      // cv::Vec3f cv_ptr = ptr;
      // cv_ptr.val = ptr.z;
      // std::cout<<"Z value: "<< cv_ptr.val<<std::endl;
      // std::cout<<"Z value: "<< cv_ptr.val<<std::endl;

      //Create visualizer
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      //viewer = createVisualizer( cloud );

      // ==========================
      // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // viewer->setBackgroundColor (0, 0, 0);
      // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
      // viewer->addCoordinateSystem ( 1.0 );
      // viewer->initCameraParameters ();



      // ================= ndt start ===================
      // pcl::PointCloud<pcl::PointXYZRGB> cloud ;
      // cloud.width = 50;
      // cloud.height =1;
      // cloud.is_dense = false;
      // cloud.points.resize(cloud.width * cloud.height);
      // for(size_t i = 0; i< cloud.points.size(); i++){
      //   cloud.points[i].x = 1024*rand()/(RAND_MAX+1.0f);
      //   cloud.points[i].y = 1024*rand()/(RAND_MAX+1.0f);
      //   cloud.points[i].z = 1024*rand()/(RAND_MAX+1.0f);

      // }
      // size_t i = 0;
      // for(int u=0;u<disp.rows;u++)
      //   for(int v=0;v<disp.cols;v++)
      //   {
      //     if(disp.at<float>(cv::Point(v,u))==0.0)
      //       continue;
      //     cv::Vec3f cv_pt=xyz.at<cv::Vec3f>(cv::Point(v,u));
          
      //     cloud.points[i].x=cv_pt.val[0];
      //     cloud.points[i].y=cv_pt.val[1];
      //     cloud.points[i].z=cv_pt.val[2];
      //     i++;

      //   }
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(&cloudnn);
      std::cout << "target_cloud Loaded " << target_cloud->size () <<  std::endl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(&cloudnn);
      std::cout << "input_cloud Loaded " << input_cloud->size () <<  std::endl;

      
        // Filtering input scan to roughly 10% of original size to increase speed of registration.
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
      approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
      approximate_voxel_filter.setInputCloud (input_cloud);
      approximate_voxel_filter.filter (*filtered_cloud);
      std::cout << "Filtered cloud contains " << filtered_cloud->size () << std::endl;

      // Initializing Normal Distributions Transform (NDT).
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

      // Setting scale dependent NDT parameters
      // Setting minimum transformation difference for termination condition.
      ndt.setTransformationEpsilon (0.01);
      // Setting maximum step size for More-Thuente line search.
      ndt.setStepSize (0.1);
      //Setting Resolution of NDT grid structure (VoxelGridCovariance).
      ndt.setResolution (1.0);

      // Setting max number of registration iterations.
      ndt.setMaximumIterations (35);

      // Setting point cloud to be aligned.
      ndt.setInputSource (filtered_cloud);
      // Setting point cloud to be aligned to.
      ndt.setInputTarget (target_cloud);

      // Set initial alignment estimate found using robot odometry.
      Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
      Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
      Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

      // Calculating required rigid transform to align the input cloud to the target cloud.
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      ndt.align (*output_cloud, init_guess);

      std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
                << " score: " << ndt.getFitnessScore () << std::endl;

      // Transforming unfiltered, input cloud using found transform.
      pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

      // Saving transformed input cloud.
     // pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

      // Initializing point cloud visualizer
      boost::shared_ptr<pcl::visualization::PCLVisualizer>
      viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer_final->setBackgroundColor (0, 0, 0);

      // Coloring and visualizing target cloud (red).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color (target_cloud, 255, 0, 0);
      viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
      viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      1, "target cloud");

      //Coloring and visualizing transformed input cloud (green).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color (output_cloud, 0, 255, 0);
      viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
      viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      1, "output cloud");

      // Starting visualizer
      viewer_final->addCoordinateSystem (1.0,  0);
      viewer_final->initCameraParameters ();

      // Wait until visualizer window is closed.
      while (!viewer_final->wasStopped ())
      {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

      // ===============  end of ndt ===================

      //loop
      // while ( !viewer->wasStopped())
      // {
      //   viewer->spinOnce(100);
      //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      // }

     // if ((char)cvWaitKey(33) == 'q')
             // break;
      cv::waitKey(33); 


    }
      ros::spinOnce();
    }

}




