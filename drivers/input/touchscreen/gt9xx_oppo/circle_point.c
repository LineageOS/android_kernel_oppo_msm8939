/************************************************************************************
** File: - mediatek\custom\common\kernel\touchpanel\GT9XX\circle_point.c
** VENDOR_EDIT
** Copyright (C), 2008-2014, OPPO Mobile Comm Corp., Ltd
**
** Description:
**     calculate GT9XX circle gesture coordinate
** Version: 1.0
** Date created: 09:32:46,23/01/2014
** Author: rndng.shi@BasicDrv.TP
************************************************************************************/

#include "circle_point.h"
#define  abs(x)   ( ( (x)>0 ) ? (x) : -(x) )

void GetCirclePoints(Point *input_points, int number ,Point  *pPnts)
{
	int i = 0;
	int k,j,m,n;
	int max_y,min_y,max_x,min_x;
	max_y=input_points[0].y;
	min_y=input_points[0].y;
	max_x=input_points[0].x;
	min_x=input_points[0].x;
	k = 0;
	j = 0;
	m = 0;
	n = 0;
	for(i=0;i<number;i++)
	{
	  if(input_points[i].y>max_y)
	  {
		   max_y=	input_points[i].y;
		   k=i;
	  }
	}
	pPnts[2]=input_points[k];

	for(i=0;i<number;i++)
	{
	  if(input_points[i].y<min_y)
	  {
		   min_y=	input_points[i].y;
		   j=i;
	  }
	}
	pPnts[0]=input_points[j];

	for(i=0;i<number;i++)
	{
	  if(input_points[i].x>max_x)
	  {
		   max_x=	input_points[i].x;
		   m=i;
	  }
	}
	pPnts[3]=input_points[m];

	for(i=0;i<number;i++)
	{
	  if(input_points[i].x<min_x)
	  {
		   min_x=	input_points[i].x;
		   n=i;
	  }
	}
	pPnts[1]=input_points[n];

}

int cross(Point a, Point b, Point c)

{
    return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x))/100;
}
