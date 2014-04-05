#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
double* 3Dpoints;
int width = 924;
int height = 924;

void readFile(){
	std::fstream myfile("array.txt", std::ios_base::in);

	double a;
	
	3Dpoints = new double[width*height * 3];
	int i = 0;
	while (myfile >> a){
		3Dpoints[i] = a;
		i++;
	}
	
}


int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = width;
  cloud.height = height;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  readFile();


  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
	  cloud.points[i].x = 3Dpoints[3*i];
	  cloud.points[i].y = 3Dpoints[3 * i + 1];
	  cloud.points[i].z = 3Dpoints[3 * i + 2];
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  /*for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;*/

  delete[] 3Dpoints;
  return (0);
}
