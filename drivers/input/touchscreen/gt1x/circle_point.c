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
/*
static int distance(Point p1, Point p2)
{
	return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
}
/
static PointsCouple find_longestdist_points(Point *p_array, int p_num)
{
	int longest_dist = 0;
	int temp_dist = 0;
	int i = 0;
	int j = 0;
	Point begin_point;
	PointsCouple longestdist_points;

	for(i=0; i<p_num; i++)
	{
		begin_point = p_array[i];
		for( j=i+1; j<p_num; j++)
		{
			temp_dist = distance(begin_point, p_array[j]);
			if(temp_dist > longest_dist)
			{
				longest_dist = temp_dist;
				longestdist_points.begin = begin_point;
				longestdist_points.end = p_array[j];
			}
		}
	}

	return longestdist_points;
}
 
static Point find_shortestdist_point(Point center_point, Point *p_array, int p_num)
{
	int shortest_dist = 0;
	int temp_dist = 0;
	int i = 1;
	Point shortestdist_point = {0,0};

	shortest_dist = distance(center_point, p_array[0]);
	for(i=1; i<p_num; i++)
	{
		temp_dist = distance(center_point, p_array[i]);
		if(temp_dist < shortest_dist)
		{
			shortest_dist = temp_dist;
			shortestdist_point = p_array[i];
		}
	}

	return shortestdist_point;
}
*/
void GetCirclePoints(Point *input_points, int number ,Point  *pPnts)
{

	int i=0;
	int k,j,m,n;
	int max_y,min_y,max_x,min_x;
	max_y=input_points[0].y;
	min_y=input_points[0].y;
	max_x=input_points[0].x;
	min_x=input_points[0].x;
	k=0;
	j=0;
	m=0;
	n=0;
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
/*
double cross(Point a, Point b, Point c) 

{ 

   return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x); 

}
*/
/*
int ClockWise(Point p,int n)
{

	int i,j,k;

	int count = 0;

	double z;

	if (n < 3)
		return -1;

	for (i=0;i<n;i++) 
	{
		j = (i + 1) % n;
		k = (i + 2) % n;
		if( (p[i].x==p[j].x) && (p[j].x==p[j].y) )
		   continue;
		z = (p[j].x - p[i].x) * (p[k].y - p[j].y);
		z -= (p[j].y - p[i].y) * (p[k].x - p[j].x);
		if (z < 0)
			count--;
		else if (z > 0)
			count++;
	}
    
	printk("ClockWise count = %d\n",count);
	if (count > 0)
		return 1; 
	else if (count < 0)
		return 0;
	else
		return 0;
}
*/