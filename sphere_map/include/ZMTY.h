#ifndef ZMTY_H
#define ZMTY_H
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979

class ZMTY
{
public:double x,y;
	   double x1,y1;
public:void ZMTYInitial(int newW,int newH,int newn);
	   void ZMTYCaculate1(double newx,double newy);
	   void ZMTYCaculate2(double newx,double newy);
	   
	   void ZMTYfCaculate1(double newx1,double newy1);
       void ZMTYfCaculate2(double newx1,double newy1);
	   
protected:int W,H;
		  int n;
private:double hfov;
		double f;
		double u,v,w;
		double t;
		
};
#endif
