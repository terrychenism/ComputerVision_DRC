#include "ZMTY.h"
#include <stdio.h>
#include <math.h>
#define  PI 3.14159265358979
void ZMTY::ZMTYInitial(int newW,int newH,int newn)
{
	W=newW;
	H=newH;
	n=newn;
	hfov=2*PI/n;
	f=W/2/tan(hfov/2);
}
/////////////////////////////////////////////////
void ZMTY::ZMTYCaculate1(double newx,double newy)
{
	x=newx,y=newy;
	t=f/sqrt((x-W/2)*(x-W/2)+f*f);
	u=f*(x-W/2)/sqrt((x-W/2)*(x-W/2)+f*f);
    v=f*(y-H/2)/sqrt((x-W/2)*(x-W/2)+f*f);
	w=f*f/sqrt((x-W/2)*(x-W/2)+f*f);
	x1=f*hfov/2+f*atan(u/w);
	y1=v+H/2;
}
void ZMTY::ZMTYfCaculate1(double newx1,double newy1)
{
	x1=newx1;
	y1=newy1;
	x=f*tan(x1/f-hfov/2)+W/2;
	y=(y1-H/2)*sqrt((x-W/2)*(x-W/2)+f*f)/f+H/2;	
}
/////////////////////////////////////////////////
void ZMTY::ZMTYCaculate2(double newx,double newy)
{
	x=newx,y=newy;
	x1=f*sin(hfov/2)+f*sin(atan((x-W/2)/f));
	y1=H/2+f*(y-H/2)/sqrt((x-W/2)*(x-W/2)+f*f);
}
void ZMTY::ZMTYfCaculate2(double newx1,double newy1)
{
	x1=newx1;
	y1=newy1;
	x=f*tan(asin(x1/f-sin(hfov/2)))+W/2;
	y=(y1-H/2)*sqrt((x-W/2)*(x-W/2)+f*f)/f+H/2;
}
/////////////////////////////////////////////////
