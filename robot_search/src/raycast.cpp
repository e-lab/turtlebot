#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <math.h>

#include "point.hpp"
#include "mapUtils.h"
#include "global.h"

#define PI 3.14159

using namespace std;
//typedef boost::numeric::ublas::matrix<int> matrix;


//This section is for analyzing and printing maps
const int num_free = 0;
const int num_obst = 100;
const int num_unknown = -1;

const int num_free_scanned = 50;
const int num_hit = -100;


//function prototypes
list<Point> bresLine(Point, Point);
double Q_to_radians(double, double);
Point findEnd(Point, double, double);
Point raycast(const signed char*, int, Point, double, double, double, bool&, signed char*);

int randint(int, int);
//////////////////////////////////////////////////////////////

//http://www.codeproject.com/Articles/15604/Ray-casting-in-a-2D-tile-based-environment
list<Point> bresLine(Point p0, Point p1)
{
	list<Point> ptList;
	Point nxtPt;

	int x0 = p0.x();
	int y0 = p0.y();

	int x1 = p1.x();
	int y1 = p1.y();


	bool isSteep = (abs(y1 - y0) > abs(x1 - x0));

	if (isSteep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}

	int deltax = abs(x1 - x0);
	int deltay = abs(y1 - y0);
	
	int error = 0;

	int y=y0;

	int ystep = (y0 < y1) ? 1 : -1;
	int xstep = (x0 < x1) ? 1 : -1; 

	//I Apologize for how complicated this next line is
	//If you need help, google bresenham's line algorithm
	//and ternary (?) operators
	//(:

	for (int x = x0;  ((x0<x1) ? (x <= x1) : (x >= x1)) ; x+=xstep)
	{
		if (isSteep) {nxtPt = Point(y,x);}
		else { nxtPt = Point(x,y);}
	
		ptList.push_back(nxtPt); //adds the new point to the list

		error += deltay;
		if (2 *error >=deltax)
		{
			y+= ystep;
			error -= deltax;
		}
	}
	
	
	return ptList;
}

//RAY STUFF
double Q_to_radians(double qw, double qz)
{
	//Converts from ROS's quaternion to an angle
	//http://www.ogre3d.org/tikiwiki/Quaternion+and+Rotation+Primer#Benefits_of_Quaternions

	double ang_w = 2*acos(qw);
	double ang_z = 2*asin(qz);
	double final_ang;
	final_ang = ( (ang_z <= 0) ? ang_w : (2*PI-ang_w) );
	return final_ang;	
}	

Point findEnd(Point start, double ang, double dist)
{
	Point diff(dist*cos(ang), dist*sin(ang));
	Point end = start.add(diff);
	end.round();
	return end;
}

Point raycast(const signed char* map, int map_w, Point startPt, double ang, double check_dist, double backup_dist, bool &validPt)
{
	Point checkPt = findEnd(startPt, ang, check_dist);
	Point movePt;

	double move_dist;
	bool hitSomething = false;

	//CLEANUP//currently bresline and raycast are decoupled, but could be faster if you combined
	list<Point> ptList = bresLine(startPt, checkPt);
	list<Point>::iterator it;
	Point pt;
	int ptVal;
	

	//cout<<endl<<endl<<"START X:"<<startPt.x()<<" Y:"<<startPt.y()<<endl;
	//cout<<"END X:"<<endPt.x()<<" Y:"<<endPt.y()<<endl;

	it=ptList.begin();	

	while(it != ptList.end() && !hitSomething)
	//searches through all the points
	
	{
		pt = *it;
		ptVal = readMap(map, map_w, pt.x(), pt.y());
		//cout<<"x_"<<pt.x()<<"_y_"<<pt.y()<<endl;

		switch(ptVal)
		{
			case num_obst:
				//HIT SOLID OBJECT
				validPt = false;
				hitSomething = true;
				break;

			case num_unknown:
				validPt = true;
				hitSomething = true;
				break;

			case num_free:
				//writeMap(debugMap, map_w, pt.x(), pt.y(), num_free_scanned);
				break;

			default:
				//writeMap(debugMap, map_w, pt.x(), pt.y(), num_free_scanned);
				break;				
		}
		it++;
	}
	it--;

	if (!hitSomething)
	{
		//cout<<"path is clear"<<endl;
		validPt = true;
	}
	move_dist = startPt.dist(pt) - backup_dist; // 
	movePt = findEnd(startPt, ang, move_dist);
	return movePt;
}








