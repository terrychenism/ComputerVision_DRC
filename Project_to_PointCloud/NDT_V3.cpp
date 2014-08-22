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

Rect box,box_t;
bool drawing_box = false;
bool gotBB = false;

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
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

void drawBox(Mat& image, CvRect box, Scalar color){
  rectangle( image, cvPoint(box.x, box.y), cvPoint(box.x+box.width,box.y+box.height),color);
} 



int main(int argc, char** argv)
{
	ros::init(argc, argv, "ReprojectImageToPointCloud");
	ros::NodeHandle nh("~");
	drc_perception::MultisenseImage image_assistance(nh);


	cv::Mat color, color_t;
	cv::Mat color_help;
	cv::Mat_<float> disp_help;
	cv::Mat_<float> disp, disp_t;
	cv::Mat_<double> Q;

	bool valid_Q=false;
	bool new_color=false;
	bool new_disp=false;
	bool ndt_start = false;
	CvScalar box_color = cv::Scalar(255, 255, 0);
	cv::Mat dispImage, croppedImage, dispImage_t, croppedImage_t;

	cvNamedWindow("showImage",CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback( "showImage", mouseHandler, NULL );
	std::vector<double> v;
	std::vector< std::vector<double> > vv;

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
		if(!new_disp|| !new_color)
	    {
		    ros::spinOnce();
		    continue;
	    }

	    else{
	    	drawBox(color_help,box,box_color);
            imshow("showImage", color_help);
            waitKey();
          
            if(gotBB){
                box.width  = 250;
                box.height = 180;
                croppedImage = color_help(box);
                dispImage = disp_help(box);
                imshow("croppedImage", croppedImage);
                rotate_image(croppedImage, color, 180); // rotate the image
                rotate_image(dispImage, disp, 180); // rotate the image
                gotBB = false;
            }

            if(!image_assistance.giveQMatrix(Q))
		    {
				ros::spinOnce();
				continue;
		    }

		    cv::Mat xyz;
			int width=disp.cols;
			int height=disp.rows;


			pcl::PointCloud<pcl::PointXYZ> cloudnn ;
			
			cloudnn.width = disp.cols;
			cloudnn.height = disp.rows;
			cloudnn.is_dense = false;
			cloudnn.points.resize(cloudnn.width * cloudnn.height);
			size_t i = 0;

			cv::reprojectImageTo3D(disp, xyz, Q, false);

			for(int u=0;u<disp.rows;u++){
				for(int v=0;v<disp.cols;v++)
			    {
					if(disp.at<float>(cv::Point(v,u))==0.0)
				    	continue;
				    cv::Vec3f cv_pt=xyz.at<cv::Vec3f>(cv::Point(v,u));
					cloudnn.points[i].x=cv_pt.val[0];
				    cloudnn.points[i].y=cv_pt.val[1];
				    cloudnn.points[i].z=cv_pt.val[2];
				    i++;
			    }
		    }		    
		    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
		    
            for(int x = 300;x <= 700; x += 15){
            	for(int y = 50; y <= 320;y += 10){
		    //int x = 375;
			//int y = 262;
            		if ((char)cvWaitKey(33) == 'q')
            			break;
            		// v.clear();
            		box_t = cv::Rect(x,y,250,180);           
            		dispImage_t = disp_help(box_t);
		            croppedImage_t =  color_help(box_t);
		            imshow("show target Image", croppedImage_t);
		            rotate_image(croppedImage_t, color_t, 180); // rotate the image
		            rotate_image(dispImage_t, disp_t, 180); // rotate the image  

		            cout << "======================== test sampler =======================" << endl;
		            
		            std::cout<<disp.cols<<" x "<<disp.rows<<std::endl;
		            
		         	// for test cloud
		         	pcl::PointCloud<pcl::PointXYZ> cloud_t;
        			cloud_t.width = disp_t.cols;
			        cloud_t.height = disp_t.rows;
			        cloud_t.is_dense = false;
			        cloud_t.points.resize(cloud_t.width * cloud_t.height);
			        i = 0;

			        cv::reprojectImageTo3D(disp_t, xyz, Q, false);


			        for(int u=0;u<disp_t.rows;u++){
			            for(int v=0;v<disp_t.cols;v++)
			            {
			                if(disp_t.at<float>(cv::Point(v,u))==0.0)
			                	continue;
			                cv::Vec3f cv_pt=xyz.at<cv::Vec3f>(cv::Point(v,u));
			                cloud_t.points[i].x=cv_pt.val[0];
			                cloud_t.points[i].y=cv_pt.val[1];
			                cloud_t.points[i].z=cv_pt.val[2];
			                i++;
			            }
			        }



			        *target_cloud = cloudnn;
		    		*input_cloud = cloud_t;
			        std::cout << "target_cloud Loaded " << target_cloud->size () <<  std::endl;
			   		std::cout << "input_cloud Loaded " << input_cloud->size () <<  std::endl;

			        
			        // Filtering input scan to roughly 10% of original size to increase speed of registration.		        
			        approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
			        // approximate_voxel_filter.setDownsampleAllData(true);	
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
			        Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
			        Eigen::Translation3f init_translation (0, 0, 0);
			        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

			        // Calculating required rigid transform to align the input cloud to the target cloud.
			        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			        ndt.align (*output_cloud, init_guess);

			        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
			        		  //<< " position " << x << " * " << y << " "
			                  << " score: " << ndt.getFitnessScore () << std::endl;			        
			        
			   
			   		

			   		// Transforming unfiltered, input cloud using found transform.
			        pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

			        // // Initializing point cloud visualizer
			        // boost::shared_ptr<pcl::visualization::PCLVisualizer>
			        // viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
			        // viewer_final->setBackgroundColor (0, 0, 0);

			        // // Coloring and visualizing target cloud (red).
			        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
			        // target_color (target_cloud, 255, 0, 0);
			        // viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
			        // viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			        //                                                 1, "target cloud");

			        // //Coloring and visualizing transformed input cloud (green).
			        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
			        // output_color (output_cloud, 0, 255, 0);
			        // viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
			        // viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			        //                                                 1, "output cloud");

			        // // Starting visualizer
			        // viewer_final->addCoordinateSystem (1.0,  0);
			        // viewer_final->initCameraParameters ();

			        // // Wait until visualizer window is closed.
			        // while (!viewer_final->wasStopped ())
			        // {
			        //   viewer_final->spinOnce (100);
			        //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			        // }

			        if(ndt.getFitnessScore () < 0.01) {
			        	cout << " good position " << x << " * " << y << " " << endl;
			        	std::cout << " validation position: " << box.x << " * " << box.y << endl;
			        	waitKey();
			        	break;
			        }


					// v.push_back(ndt.getFitnessScore ());

			   		// v.push_back(y);
			   		// v.push_back(x);
			        // vv.push_back(v);

			        // Transforming unfiltered, input cloud using found transform.
			        // pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
            	}
			} // END OF FOR LOOP

            // std::sort (v.begin(), v.end()); 
            // cout << "the smallest: "<< v[0] << endl;

	    } // END OF ELSE
	    if ((char)cvWaitKey(100) == 'q')
            break;

	    
	    ros::spinOnce();
	} // END OF while(ros::ok())


}

