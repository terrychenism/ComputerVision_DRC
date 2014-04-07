


#include "ScanningMethod.h"
#include"cv.h"
#include"cxcore.h"
#include"highgui.h"

#include<iostream>
using namespace std;

//根据校正图像 U坐标 计算鱼眼图像的X坐标
float CalculateFisheye_x( int u, int v, float x0, float y0, float r);
float Calculate_dx( int v, float y0, float r);



int main( int argc, char** argv )
{
	IplImage* pImg;
	if( ( pImg = cvLoadImage( "10.jpg", CV_LOAD_IMAGE_ANYCOLOR ) ) == 0)
		return 0;
	float x0;
	float y0;
	float r;

	IplImage* imgTemp = ScaningMethod( pImg, &x0, &y0, &r);
	if( imgTemp == 0 )
	{
		cout << "标定鱼眼图像中心失败" <<endl;
		return 0;
	}
	cout << x0 << endl << y0 << endl <<  r << endl;

	
	IplImage* imgCalib = cvCreateImage( cvGetSize( imgTemp ), IPL_DEPTH_8U, 3);

	for( int j = 0; j < imgCalib->height; j++)
		for( int i = 0; i < imgCalib->width; i++)	
		{
			float x = CalculateFisheye_x( i, j, imgTemp->width/2, imgTemp->height/2, r );
			int ix = (int)x;
			int dx = Calculate_dx( j, imgTemp->height/2, r );

			
			/* if( abs(x - (int)x0) > dx+50 )						// 有错，可以不要，该段的功能为：
			 {														//当计算得到的x值超出鱼眼图像的有效区域时，
																	//x对应的校正后的图像像素点值设为0
				
				((uchar*)(imgCalib->imageData + imgCalib->widthStep*j))[i*3] = 0;
				((uchar*)(imgCalib->imageData + imgCalib->widthStep*j))[i*3 + 1] = 0;
				((uchar*)(imgCalib->imageData + imgCalib->widthStep*j))[i*3 + 2] = 0;
			 }
			 else*/
			 {
				//   计算出鱼眼图像的（x，y）坐标后，使用双线性插值法填充校正后图像
				 ((uchar*)(imgCalib->imageData + imgCalib->widthStep*j))[i*3] =
					((uchar*)(imgTemp->imageData + imgTemp->widthStep*j))[ix*3] * (1-abs(x-ix))+
					((uchar*)(imgTemp->imageData + imgTemp->widthStep*j))[(ix+1)*3] * (1-abs(x-(ix+1)));

				((uchar*)(imgCalib->imageData + imgCalib->widthStep*j))[i*3+1] =
					((uchar*)(imgTemp->imageData + imgTemp->widthStep*j))[ix*3+1] * (1-abs(x-ix))+
					((uchar*)(imgTemp->imageData + imgTemp->widthStep*j))[(ix+1)*3+1] * (1-abs(x-(ix+1)));

				((uchar*)(imgCalib->imageData + imgCalib->widthStep*j))[i*3+2] =
					((uchar*)(imgTemp->imageData + imgTemp->widthStep*j))[ix*3+2] * (1-abs(x-ix))+
					((uchar*)(imgTemp->imageData + imgTemp->widthStep*j))[(ix+1)*3+2] * (1-abs(x-(ix+1)));			 
			 }
		}
	
	cvNamedWindow( "raw image", 1 );//创建窗口

	cvNamedWindow( "correct", 1 );
	cvShowImage( "raw", imgTemp );

	cvSaveImage("round.jpg", imgTemp);
    cvShowImage( "correct", imgCalib );//显示图像

    cvWaitKey(0); //等待按键

    cvDestroyWindow( "correct" );//销毁窗口
    cvReleaseImage( &imgTemp ); //释放图像
	cvReleaseImage( &imgCalib );
	cvReleaseImage( &pImg );

	return 0;

}



float CalculateFisheye_x( int u, int v, float x0, float y0, float r)
{
	float xh = u - x0; 
	float yi = y0 - v;
	float dx = sqrt( r*r - yi*yi );
	int xk = ( x0 + xh * dx / r);
	return xk;
}


float Calculate_dx( int v, float y0, float r )
{
	float yi = y0 - v;
	int dx = sqrt( r*r - yi*yi);
	return dx;
}
