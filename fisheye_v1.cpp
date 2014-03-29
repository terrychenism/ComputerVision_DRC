
#include "fisheye.h"
#define   M_PI      3.14159265358979323846

using namespace cv;
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

//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{   

  struct ocam_model o, o_cata; 
  get_ocam_model(&o, "calib_results_fisheye.txt");

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
  Mat src1 = imread("./test_fisheye.jpg" );    
 
  Mat dst_persp   = cvCreateImage( src1.size(), 8, 3 );   


  //Mat mapx_persp = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  //Mat mapy_persp = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
 
  float sf = 4;
  CvMat* mapx_persp1 =cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  CvMat* mapy_persp1 = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
 
  create_perspecive_undistortion_LUT( mapx_persp1, mapy_persp1, &o, sf );

  cv::Mat mapx_persp = Mat(mapx_persp1,true); // to copy the data
  cv::Mat mapy_persp = Mat(mapy_persp1,true); // to copy the data
  //IplImage* image1=cvCloneImage(&(IplImage)src1);
  //IplImage* image2=cvCloneImage(&(IplImage)dst_persp);

  //cvRemap( src2, src3, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
  remap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
  cvNamedWindow( "Original fisheye camera image", 1 );
  imshow( "Original fisheye camera image", src1 );

  cvNamedWindow( "Undistorted Perspective Image", 1 );
  imshow( "Undistorted Perspective Image", dst_persp );
 // cvShowImage( "Undistorted Perspective Image", image2 );

  imwrite("undistorted_perspective.jpg",dst_persp);
  printf("\nImage %s saved\n","undistorted_perspective.jpg");


  cvWaitKey();

  
 /* cvReleaseImage(&src1);

  cvReleaseImage(&dst_persp);

  cvReleaseMat(&mapx_persp);
  cvReleaseMat(&mapy_persp);  
*/

  return 0;
}
