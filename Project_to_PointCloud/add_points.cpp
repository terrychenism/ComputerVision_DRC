#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <fstream>

struct MyPointType
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float test;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, test, test)
)


int
main (int argc, char** argv)
{

  std::fstream myfile("debri.xyz", std::ios_base::in);

  double a;
  double* array;
  array = new double[8 * 3];
  int i = 0;
  while (myfile >> a){
  array[i] = a;
  i++;
  }
  // for(int i = 0; i< 24 ; i++)
  //   std::cout << array[i] << std::endl;


  // create point cloud

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);


  target_cloud->width  = 2;
  target_cloud->height = 4;
  target_cloud->points.resize (target_cloud->width * target_cloud->height);


  for(int i = 0; i < 8 ; i++){ 
   
    target_cloud->points[i].x = array[i*3];
    target_cloud->points[i].y = array[i*3+1];
    target_cloud->points[i].z = array[i*3+2];
  }


  std::cout << "Loaded " << target_cloud->size() << " data points" << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");


  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0,  0);
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  
  
  return 0;
}

