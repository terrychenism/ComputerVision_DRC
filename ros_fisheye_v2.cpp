#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <perception_common/MultisenseImage.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>


#include <perception_common/global.h>
#include <sensor_msgs/image_encodings.h>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64
#define   M_PI      3.14159265358979323846

using namespace cv;
int sf = 1;
CvMat* mapx_persp1;
CvMat* mapy_persp1;
Mat src1 ;    
Mat dst_persp  ;   


struct ocam_model
{
  double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
};
ocam_model o;



//------------------------------------------------------------------------------
int get_ocam_model(struct ocam_model *myocam_model, char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol; 
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc); 
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;
 
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);          
   return -1;
 }
 
 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }
 
 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);
 return 0;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc); 
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol); 
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );
  
 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;
 
 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }
 
 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );
 
 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp; 
 point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol; 
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc); 
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;
  
  if (norm != 0) 
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;
  
    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height     
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];
     
     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {   
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
         }
}
static void onTrackbar(int, void*)
{
  create_perspecive_undistortion_LUT( mapx_persp1, mapy_persp1, &o, sf );

  cv::Mat mapx_persp = Mat(mapx_persp1,true); // to copy the data
  cv::Mat mapy_persp = Mat(mapy_persp1,true); // to copy the data

  // for( int j = 0; j < src1.rows; j++ )
  //  { for( int i = 0; i < src1.cols; i++ )
  //      {

  //            if( i > src1.cols*0.25 && i < src1.cols*0.75 && j > src1.rows*0.25 && j < src1.rows*0.75 )
  //              {
  //                  // add something


  //               }
  //            else
  //              { mapx_persp.at<float>(j,i) = 0 ;
  //               mapy_persp.at<float>(j,i) = 0 ;
  //              }
  //       }
  //   }
  remap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
  imshow( "rectified image", dst_persp );
}



cv::Mat g_raw_bgr_image;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    g_raw_bgr_image=cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'BGR8'.", msg->encoding.c_str());
  }
}



//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{   


  ros::init(argc, argv, "ros_opencv_example");

  
  ros::NodeHandle nh;
  //drc_perception::MultisenseImage camera(nh);
  image_transport::ImageTransport camera(nh);
  image_transport::Subscriber sub = camera.subscribe("camera/image", 1, imageCallback);

  struct ocam_model o_cata; 
  //system("pwd");
  get_ocam_model(&o, "src/ros_fisheye_rectify/calib/calib_results_fisheye.txt");

  int i;
  printf("pol =\n");    for (i=0; i<o.length_pol; i++){    printf("\t%e\n",o.pol[i]); };    printf("\n");
  printf("invpol =\n"); for (i=0; i<o.length_invpol; i++){ printf("\t%e\n",o.invpol[i]); }; printf("\n");  
  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o.xc,o.yc,o.width,o.height);

  double point3D[3] = { 100 , 200 , -300 };       
  double point2D[2];                         
  world2cam(point2D, point3D, &o); 
  

  printf("\nworld2cam: pixel coordinates reprojected onto the image\n");  
  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);



  cam2world(point3D, point2D, &o); 


  printf("\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);
  

  /* --------------------------------------------------------------------*/  
  //src1 = imread("./test_fisheye.jpg" );   
  while(ros::ok())
    {
      src1 = g_raw_bgr_image;


          if ( src1 .empty() ) {
             std::cout << "unable to load an input image\n";
                
                return -1;
             }
  //src1 = imread( "/home/tairuichen/Desktop/test_fisheye.jpg" );
  dst_persp   = cvCreateImage( cvSize(1200,1200), 8, 3 );   


  //Mat mapx_persp = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  //Mat mapy_persp = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  
 
  // mapx_persp1 =cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  // mapy_persp1 = cvCreateMat(src1.rows, src1.cols, CV_32FC1);

  mapx_persp1 =cvCreateMat(1200, 1200, CV_32FC1);
  mapy_persp1 = cvCreateMat(1200, 1200, CV_32FC1);
  namedWindow("rectified image", 1);
  createTrackbar("scaling factor", "rectified image", &sf, 25, onTrackbar);
  onTrackbar(0, 0);

 
  //create_perspecive_undistortion_LUT( mapx_persp1, mapy_persp1, &o, sf );

  //cv::Mat mapx_persp = Mat(mapx_persp1,true); // to copy the data
  //cv::Mat mapy_persp = Mat(mapy_persp1,true); // to copy the data


  //IplImage* image1=cvCloneImage(&(IplImage)src1);
  //IplImage* image2=cvCloneImage(&(IplImage)dst_persp);

  //cvRemap( src2, src3, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
  //remap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );


  cvNamedWindow( "Original fisheye camera image", 1 );
  imshow( "Original fisheye camera image", src1 );

  //cvNamedWindow( "Undistorted Perspective Image", 1 );
  //imshow( "Undistorted Perspective Image", dst_persp );


  //imwrite("undistorted_perspective.jpg",dst_persp);
  printf("\nImage %s saved\n","undistorted_perspective.jpg");


  cvWaitKey();

  
 /* cvReleaseImage(&src1);

  cvReleaseImage(&dst_persp);

  cvReleaseMat(&mapx_persp);
  cvReleaseMat(&mapy_persp);  
*/
  ros::spinOnce();
  }

  return 0;
}
