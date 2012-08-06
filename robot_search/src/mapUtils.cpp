#include<list>
#include<string.h>
#include<stdlib.h>
#include "global.h"
#include "point.hpp"

using namespace std;
//MAP STUFF

//Function prototypes
Point botPosToMatrix(Point, double, double, double);
int readMap(const signed char*, int, int, int);
signed char* cloneMap(const signed char*, int, int);
void writeMap(const signed char*, int, int, int, char);
void writeSquare(signed char*, int, int, int, int);
bool isSolid(const signed char*, int, int, int);
void paintLine(const signed char*, int, list<Point>, int);
void printMap(const signed char*, int);
int randint(int low, int high);

Point botPosToMatrix(Point bot, double o_dx, double o_dy,double res_fact)
{
	Point matPt = bot.sub(Point(o_dx,o_dy));
	matPt = matPt.scale(res_fact);
	matPt.round();
	return matPt;
}

Point matrixToMapPos(Point matxPt, double o_dx, double o_dy, double res_fact)
{
	Point mapPt = matxPt.scale(1/res_fact);
	mapPt = mapPt.add(Point(o_dx,o_dy));
	return mapPt;
}

int readMap(const signed char* map, int width, int x, int y)
{
	return (int)map[width*y+x];
}

signed char* cloneMap(const signed char* constMap, int wid, int hi)
{
	signed char clonedMap[1000*1000];
	memcpy(clonedMap, constMap, wid*hi);
	return clonedMap; 
}

void writeMap(signed char* map, int width, int x, int y, signed char newVal)
{
	map[width*y+x] = newVal;
}

void writeSquare(signed char* map, int map_w, int x, int y, int sqr_hw)
{
	int paintNum = 100;
	for (int i = -sqr_hw; i<sqr_hw; i++)
	{
		for (int j = -sqr_hw; j<sqr_hw; j++)
		{
			writeMap(map, map_w, x+i, y+j, paintNum);
		}
	} 
}

void paintLine(signed char* map, int map_w, list<Point> linePts, int newVal)
{
	list<Point>::iterator i;
	Point pt;
	for(i=linePts.begin(); i!=linePts.end(); i++)
	{
	
		pt = *i;
		//R//map(pt.y(), pt.x()) = newVal;
		writeMap(map, map_w, pt.x(), pt.y(), newVal);
	}
}

/*matrix randomMap(int h, int w)
{
	matrix rMap(h,w);
	int randN;
	
	for (int row = 0; row<h; row++)
	{
		for (int col=0; col<w; col++)
		{
			randN = randint(1,12);
			rMap(row,col) = (randN == 1) ? num_obst : num_free; //this gives it a 1/8 chance of placing a 1
			
			if ((col==0)||(row==0)||(col==numCols-1)||(row==numRows-1))
			{
				rMap(row,col) = num_obst;
			}

		}
		
	}
	return rMap;
}*/

void printMap(signed char* map, int w, int h)
{
	//cout<<"PRINT"<< endl;
	int total = 0;
	int foo;

	for (int row = 0; row<h; row++)
	{
		for (int col=0; col<w; col++)
		{

			/*if (readMap(map, w, row, col)==num_free)
			{
				cout<<char_free;
			}
			else if (readMap(map, w, row, col)==num_obst)
			{		
				cout<<char_obst;
			}
			else if (readMap(map, w, row, col)==num_free_scanned)
			{
				cout<<char_free_scanned;
			}
			else if (readMap(map, w, row, col)==num_hit)
			{
				cout<<char_hit;
			}*/
			
			foo=readMap(map, w, row, col);
			if (foo != -1 && foo!=0)
			{
				//cout<<foo<<" ";
			}
			++total;
		}
		//cout <<endl;
	}
}


//RANDOM STUFF
int randint(int low, int high)
{
	int ri = low + (rand()%(high+1));
	return ri;
}
