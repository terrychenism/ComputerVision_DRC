
#include "fisheye.h"
#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
struct ocam_model o1, o2;
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
 //double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );
 
 //point3D[0] = invnorm*xp;
 //point3D[1] = invnorm*yp; 
 //point3D[2] = invnorm*zp;
 point3D[0] = xp;
 point3D[1] = yp; 
 point3D[2] = zp;

 //printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);
 
}
//------------------------------------------------------------------------------

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


   //printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
	 num = height*width;
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];
    
	 //array = new double [num*3];
	 

	for (i=0; i<height; i++){
        for (j=0; j<width; j++)
         {   

             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
			 /*cam2world(M, m, ocam_model);
			 array[3*(924*i+j)] = M[0];
			 array[3*(924*i+j)+1] = M[1];
			 array[3*(924*i+j)+2] = M[2];*/

         }
	}

	//writeFile();
	//delete[] array;
	mapx_persp_left = Mat(mapx); // to copy the data
	mapy_persp_left = Mat(mapy); // to copy the data
	//cam2world(M, m, ocam_model);
}

void writeFile(){
	
	ofstream outfile("array.txt");
	for (int m = 0; m<num; m++)
	outfile << array[3*m] << "   " << array[3*m + 1] << "   " <<array[3*m + 2] << endl;
	outfile.close();
	
}

void rotate_image_90n(cv::Mat &src, cv::Mat &dst, int angle)
{   
   if(src.data != dst.data){
       src.copyTo(dst);
   }

   angle = ((angle / 90) % 4) * 90;

   //0 : flip vertical; 1 flip horizontal
   bool const flip_horizontal_or_vertical = angle > 0 ? 1 : 0;
   int const number = std::abs(angle / 90);          

   for(int i = 0; i != number; ++i){
       cv::transpose(dst, dst);
       cv::flip(dst, dst, flip_horizontal_or_vertical);
   }
}

static void onTrackbar1(int, void*)
{
  create_perspecive_undistortion_LUT( mapx_persp1, mapy_persp1, &o1, sf );

mapx_persp_left = Mat(mapx_persp1); // to copy the data
mapy_persp_left  = Mat(mapy_persp1); // to copy the data
 
  remap(src1, dst_persp1, mapx_persp_left, mapy_persp_left, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0) );
  imshow( "rectified image1", dst_persp1 );
  //imwrite("undistorted_perspective1.jpg",dst_persp);
}
//------------------------------------------------------------------------------
static void onTrackbar2(int, void*)
{
  create_perspecive_undistortion_LUT( mapx_persp2, mapy_persp2, &o2, sf );

 mapx_persp_right = Mat(mapx_persp2); // to copy the data
 mapy_persp_right  = Mat(mapy_persp2); // to copy the data
 
 /*hconcat(mapx_persp_right,mapx_persp_left,mapx_persp_right);
 hconcat(mapy_persp_right,mapy_persp_left,mapy_persp_right);*/

  remap(src2, dst_persp2, mapx_persp_right, mapy_persp_right, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0) );
  imshow( "rectified image2", dst_persp2 );

}
//---------------------------------------------------------------------------
int main(int argc, char *argv[])
{   

  struct ocam_model o_cata; 
  get_ocam_model(&o1, "calib_results_fisheye1.txt");
  get_ocam_model(&o2, "calib_results_fisheye2.txt");
  int i;
  printf("pol =\n");    for (i=0; i<o1.length_pol; i++){    printf("\t%e\n",o1.pol[i]); };    printf("\n");
  printf("invpol =\n"); for (i=0; i<o1.length_invpol; i++){ printf("\t%e\n",o1.invpol[i]); }; printf("\n");  
  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o1.xc,o1.yc,o1.width,o1.height);
                       

  
  /* --------------------------------------------------------------------*/
  /* WORLD2CAM projects 3D point into the image                          */
  /* --------------------------------------------------------------------*/
  double point3D[3] = { 100 , 200 , -300 };       // a sample 3D point
  double point2D[2];                              // the image point in pixel coordinates  
  world2cam(point2D, point3D, &o1); // The behaviour of this function is the same as in MATLAB
  printf("\nworld2cam: pixel coordinates reprojected onto the image1\n");  
  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);



  cam2world(point3D, point2D, &o1); 
  printf("\ncam2world: coordinates back-projected onto the unit sphere1 (x^2+y^2+z^2=1)\n");
  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);

  /*cam2world(point3D, point2D, &o2); 
  printf("\ncam2world: coordinates back-projected onto the unit sphere2 (x^2+y^2+z^2=1)\n");
  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);*/

  

  /* --------------------------------------------------------------------*/  
  
  src1 = imread("./img_l.jpg" );
  src2 = imread("./img_r.jpg" );

  dst_persp1   = cvCreateImage( src1.size(), 8, 3 );   
  dst_persp2   = cvCreateImage( src2.size(), 8, 3 );   

  
 
  mapx_persp1 = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  mapy_persp1 = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  mapx_persp2 = cvCreateMat(src2.rows, src2.cols, CV_32FC1);
  mapy_persp2 = cvCreateMat(src2.rows, src2.cols, CV_32FC1);

  //namedWindow("rectified image1", 1);
  //namedWindow("rectified image2", 1);
  //createTrackbar("scaling factor", "rectified image1", &sf, 25, onTrackbar1);
  //createTrackbar("scaling factor", "rectified image2", &sf, 25, onTrackbar2);
  //onTrackbar1(0, 0);
  //onTrackbar2(0, 0);

	create_perspecive_undistortion_LUT( mapx_persp1, mapy_persp1, &o1, sf );
	mapx_persp_left = Mat(mapx_persp1); // to copy the data
	mapy_persp_left = Mat(mapy_persp1); // to copy the data
	//for( int j = 0; j < mapx_persp_left.rows; j++ ){ 
	//	for( int i = 924; i < mapx_persp_left.cols; i++ ){

	//			 mapx_persp_left.at<float>(j,i) = 0 ;
	//			 mapy_persp_left.at<float>(j,i) = 0 ;
	//			
	// }
 // }

	remap(src1, dst_persp1, mapx_persp_left, mapy_persp_left, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0) );
	
	cout<<"image1 process"<<endl;



	//create_perspecive_undistortion_LUT( mapx_persp2, mapy_persp2, &o2, sf );
	//mapx_persp_right = Mat(mapx_persp2); // to copy the data
	//mapy_persp_right = Mat(mapy_persp2); // to copy the data
	//remap(src2, dst_persp2, mapx_persp_right, mapy_persp_right, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0) );

   imshow( "rectified image1", dst_persp1 );

   //imshow( "rectified image2", dst_persp2 );



  cvWaitKey();

 

  return 0;
}

