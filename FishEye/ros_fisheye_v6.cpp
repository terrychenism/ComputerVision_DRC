/**
 ********************************************************************************************************
 * @file    FishEye.cpp
 * @brief   fisheye image transformation
 * @details transfrom fisheye image to 2D image
 ********************************************************************************************************
 */

/*** INCLUDE FILES ***/
#include "fisheye.h"

ocam_model o;


/**
 * @brief   get transform parameters
 */

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
   ROS_INFO("File %s cannot be opened\n", filename);
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


/**
 * @brief  transform 3D points to 2D points
 */
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

/**
 * @brief   undistortion function
 */
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

/**
 * @brief   create the trackbar
 */
static void onTrackbar(int, void*)
{
  create_perspecive_undistortion_LUT( mapx_persp1, mapy_persp1, &o, sf );

  cv::Mat mapx_persp = Mat(mapx_persp1,true); // to copy the data
  cv::Mat mapy_persp = Mat(mapy_persp1,true); // to copy the data

  remap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

}



//cv::Mat src1;
void rotate_image_90n(cv::Mat &src, cv::Mat &dst, int angle)
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


/**
 * @brief   main function
 */

//-------------main function--------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_opencv_example");
    //load an image from the raw image
    ros::NodeHandle nh;
    //---------------

     char cwd[1024];

     if (getcwd(cwd, sizeof(cwd)) != NULL)
     {
     // 
     }
     else
         ROS_WARN("Could not get the current working dir!");
     
    std::string image_name("/sitcam/right/image_raw");

    std::string proj_param(std::string(cwd) + "/src/drc/wrecs_perception/perception_fisheye_rectify/calib/calib_results_fisheye.txt");

  if (!nh.getParam("image_name", image_name))
    ROS_WARN("Parameter <%s> Not Set. Using Default Value of <%s>!",
        "image_name", image_name.c_str());

  if (!nh.getParam("proj_param", proj_param))
    ROS_WARN("Parameter <%s> Not Set. Using Default Value of <%s>!",
        "proj_param", proj_param.c_str());


    //-----------------


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(image_name, 1, imageCallback);



    //get_ocam_model(&o, "/src/drc/wrecs_perception/perception_fisheye_rectify/calib/calib_results_fisheye.txt");
    char *param= new char[proj_param.length() + 1];
    strcpy(param, proj_param.c_str());
    get_ocam_model(&o, param);// get parameter for projection
    delete [] param;

//------------print parameters------------------------------



    int i;
    ROS_INFO("pol =\n");    for (i=0; i<o.length_pol; i++){    ROS_INFO("\t%e\n",o.pol[i]); };    ROS_INFO("\n");
    ROS_INFO("invpol =\n"); for (i=0; i<o.length_invpol; i++){ ROS_INFO("\t%e\n",o.invpol[i]); }; ROS_INFO("\n");
    ROS_INFO("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o.xc,o.yc,o.width,o.height);

//------------------------------------------------------------
    namedWindow("rectified image", 1);
    cvNamedWindow( "Original fisheye camera image", 1 );
    createTrackbar("scaling factor", "rectified image", &sf, 25, onTrackbar);



//---------------ros processing------------------------------------
    while(ros::ok())
    {


      src1 = g_raw_bgr_image;


      if(!src1.empty())
          {


           imshow( "Original fisheye camera image", src1 ); //show original image
           //imwrite("/home/tairuichen/Desktop/image_right.jpg",src1);

           dst_persp   = cvCreateImage( src1.size(), 8, 3 );//create the dst image
           mapx_persp1 =cvCreateMat(src1.rows, src1.cols, CV_32FC1 ); //create map for projection
           mapy_persp1 = cvCreateMat(src1.rows, src1.cols, CV_32FC1);

           onTrackbar(0, 0);// track bar and do the processing

           rotate_image_90n(dst_persp, dst_persp, 90); // rotate the image
           imshow( "rectified image", dst_persp );

          }

        waitKey(20);
        ros::spinOnce();

    }

}



