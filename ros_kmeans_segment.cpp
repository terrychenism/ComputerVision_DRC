#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <cv.h>
#include <cvaux.h>
#include <ml.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;
Mat image;
void show_result(const cv::Mat& labels, const cv::Mat& centers, int height, int width)
{
        std::cout << "===\n";
        std::cout << "labels: " << labels.rows << " " << labels.cols << std::endl;
        std::cout << "centers: " << centers.rows << " " << centers.cols << std::endl;
        assert(labels.type() == CV_32SC1);
        assert(centers.type() == CV_32FC1);
         
        cv::Mat rgb_image(height, width, CV_8UC3);
        cv::MatIterator_<cv::Vec3b> rgb_first = rgb_image.begin<cv::Vec3b>();
        cv::MatIterator_<cv::Vec3b> rgb_last = rgb_image.end<cv::Vec3b>();
        cv::MatConstIterator_<int> label_first = labels.begin<int>();
         
        cv::Mat centers_u8;
        centers.convertTo(centers_u8, CV_8UC1, 255.0);
        cv::Mat centers_u8c3 = centers_u8.reshape(3);
         
        while ( rgb_first != rgb_last ) {
                const cv::Vec3b& rgb = centers_u8c3.ptr<cv::Vec3b>(*label_first)[0];
                *rgb_first = rgb;
                ++rgb_first;
                ++label_first;
        }
        cv::imshow("tmp", rgb_image);
        cv::waitKey();
}
int main(int argc, char * argv[])
{
        ros::init(argc, argv, "ros_opencv_example");

        image = imread( "/home/tairuichen/Desktop/lena.jpg" );
        //cv::Mat image = cv::imread("debris.jpg");
        if ( image.empty() ) {
                std::cout << "unable to load an input image\n";
                return 1;
        }
         
        std::cout << "image: " << image.rows << ", " << image.cols << std::endl;
        assert(image.type() == CV_8UC3);
        cv::imshow("image", image);
 
        cv::Mat reshaped_image = image.reshape(1, image.cols * image.rows);
        std::cout << "reshaped image: " << reshaped_image.rows << ", " << reshaped_image.cols << std::endl;
        assert(reshaped_image.type() == CV_8UC1);
        //check0(image, reshaped_image);
         
        cv::Mat reshaped_image32f;
        reshaped_image.convertTo(reshaped_image32f, CV_32FC1, 1.0 / 255.0);
        std::cout << "reshaped image 32f: " << reshaped_image32f.rows << ", " << reshaped_image32f.cols << std::endl;
        assert(reshaped_image32f.type() == CV_32FC1);
         
        cv::Mat labels;
        int cluster_number = 10;
        cv::TermCriteria criteria; 
    cv::TermCriteria::COUNT, 100, 1;
        cv::Mat centers;
        cv::kmeans(reshaped_image32f, cluster_number, labels, criteria, 1, cv::KMEANS_RANDOM_CENTERS, centers);
 
        show_result(labels, centers, image.rows,image.cols);
        ros::spin();
        return 0;
}




