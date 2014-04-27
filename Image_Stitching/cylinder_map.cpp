#include "ZMTY.h"
#include <stdio.h>
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>

IplImage* doZMBH(IplImage* in,int n)
{
	ZMTY example;
	example.ZMTYInitial(in->width,in->height,n);
	IplImage* out =cvCreateImage(cvSize(in->width,in->height),in->depth,in->nChannels);
	CvScalar s0;
	CvScalar s1;
	CvScalar s2;
	CvScalar s3;
	CvScalar s4;

	for (double i=0;i<out->height;i++)
	{
		for (double j=0;j<out->width;j++)
		{
			example.ZMTYfCaculate1(j,i);
            if (example.x>0&&example.x<in->width-1&&example.y>0&&example.y<in->height-1)
            {
				double u,v;
				u=example.x-int(example.x);v=example.y-int(example.y);
				s0=cvGet2D(in,int(example.y),int(example.x));
				s1=cvGet2D(in,int(example.y),int(example.x)+1);
				s2=cvGet2D(in,int(example.y)+1,int(example.x));
				s3=cvGet2D(in,int(example.y)+1,int(example.x)+1);
                for (int rgb=0;rgb<3;rgb++)
                {
					s4.val[rgb]=int((1-u)*(1-v)*s0.val[rgb]+(1-u)*v*s1.val[rgb]+u*(1-v)*s2.val[rgb]+u*v*s3.val[rgb]);
                }
				
				cvSet2D(out,i,j,s4);
            }
			if (example.x==in->width-1||example.y==in->height-1)
			{
				s4=cvGet2D(in,int(example.y),int(example.x));
				cvSet2D(out,i,j,s4);
			}
		}
	}
	return(out);
};


int main(int argc, char* argv[])
{
	Mat img=imread("l_undistorted_perspective.jpg",1);
	imshow("raw image",img);
	IplImage* src =new IplImage(img);
	IplImage* out=doZMBH(src,4);
	Mat imgMat(out);  
	imshow("cylinder image",imgMat);
	waitKey();
	return 0;
}

