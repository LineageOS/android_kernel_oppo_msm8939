#ifndef __CIRCLE_POINT_H__
#define __CIRCLE_POINT_H__

typedef struct 
{
	int x;
	int y;
} Point;

typedef struct
{
	Point begin;
	Point end;
}PointsCouple;

extern void GetCirclePoints(Point *input_points, int number ,Point  *pPnts);
//extern double cross(Point a, Point b, Point c);
//extern int ClockWise(Point *p,int n);
#endif