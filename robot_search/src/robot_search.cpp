#include<ros/ros.h>
//
//bot location
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>
//map reader
#include<nav_msgs/OccupancyGrid.h>
//
//
#include<iostream>
#include<string.h>
#include<list>
#include "raycast.h"
#include "mapUtils.h"

#define PI 3.14159

typedef geometry_msgs::PoseStamped rospose;
using namespace ros;
using namespace std;

const double bot_diam = 0.35;
const double kinect_fov = 64.6;

double ray_check_dist = 1.5+(bot_diam/2);//in meters
double backup_dist = 1*bot_diam; //in meters, the distance from a wall the robot should move to
int paintNum = 50;

int sweep_ang = 180;
int sweep_num = 40;



const signed char* myMap;
//signed char* debugMap;
int myMap_w;
int myMap_h;
double myMap_res_factor; // 1/resolutions

double origin_dx;
double origin_dy;

Point botPos;
double botAngle;

bool isMap;
list<Point> placed_nodes;

//function prototypes
void store_map_cb(const nav_msgs::OccupancyGrid::ConstPtr&);
void storeBotPose(tf::TransformListener&, tf::StampedTransform&);
int node_sweep(double, int);
int myRound(double);
list<Point> robot_search();
//void pubMap(const nav_msgs::OccupancyGrid::ConstPtr&);
void paintNodes();

list<Point> robot_search(){
	//Initialize seeker node	
	NodeHandle n;
	isMap = false;
	placed_nodes.clear();
	//subscribe to the map data	
	Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map",2,store_map_cb);
	while(ok() && !isMap){
		spinOnce();
	}
	return placed_nodes;
}

/*void pubMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	nav_msgs::OccupancyGrid map2;
	NodeHandle n2;
	Publisher pub = n2.advertise<nav_msgs::OccupancyGrid>("/map_2",10);
	map2 = *msg;
	map2.header.stamp = Time::now();
	for(int i = 0; i<myMap_w*myMap_h; i++)
		map2.data[i] = debugMap[i];
	Rate rate(3);
	int count = 0;
	// let it publish twice
	while(count != 3){
		pub.publish(map2);
		rate.sleep();
		count++;
	}
	
}*/
void store_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg){	
	//stores map data in global variables
	myMap_w = msg->info.width;
	myMap_h = msg->info.height;
	myMap_res_factor = 1/(msg->info.resolution);

	origin_dx = msg->info.origin.position.x;
	origin_dy = msg->info.origin.position.y;

	myMap = &msg->data[0];
	//debugMap = cloneMap(myMap, myMap_w, myMap_h);
	//cout << msg->info.origin.position.x << " " << msg->info.origin.position.y << endl;


	if (!isMap) //for verifying we actually got a map, and this is the first time
	{
		tf::TransformListener tf_listener(Duration(10));
		tf::StampedTransform st_tf;
		isMap = true;
		storeBotPose(tf_listener, st_tf);
			
		node_sweep(sweep_ang, sweep_num);
		//paintNodes();
		//pubMap(msg);
		//printMap(debugMap,myMap_w,myMap_h);
	}
}



void storeBotPose(tf::TransformListener& tf_list, tf::StampedTransform& stamped_tf)
{
	bool error = true;
	while(error){
		try{
			error = false;
			tf_list.lookupTransform("/map","/base_link",Time(0),stamped_tf);
		}
		catch(tf::TransformException ex){
			error = true;
		}
	}

	rospose base, map;
	base.header.frame_id = "base_link";
	base.pose.position.x = 0;
	base.pose.position.y = 0;
	base.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	Time current_time = Time::now();
	tf_list.getLatestCommonTime("base_link","map",current_time,NULL);
	base.header.stamp = current_time;
	tf_list.transformPose("map", base, map);

	//CONVERT metric coordinates to map coordinates
	botPos = Point(map.pose.position.x, map.pose.position.y);


	//cout<<"BotX:"<<botPos.x()<<" BotY:"<<botPos.y()<<endl;
	botAngle = Q_to_radians(map.pose.orientation.w, map.pose.orientation.z);
	botAngle = 2*PI - botAngle; //corrects for difference in angle direction
	//cout<<"angle"<<botAngle<<endl;
}

int node_sweep(double ang_max, int n)
{

	//returns a count of the number of placed nodes
	int nodeCount = 0;
	int segCount = 0;//a count for one particular segment of the scanned arc	

	double ang_n = ang_max/(n-1);
	double ang_n_rad = ang_n*(PI/180);
	double ang_max_rad = ang_max*(PI/180);
	
	//Theta = 2*asin(w/2r) 
	//formula for the lone angle of an isosceles triangle given the sides
	double ang_seg_min_rad = 2*asin( (2*bot_diam) / (2*ray_check_dist) ); 
	double ang_seg_max_rad = kinect_fov*(PI/180);
	double ang_seg_total_rad;
	//double and_seg_rad;

	
	//store these locally in case they get overridden
	Point bPosit = botPosToMatrix(botPos, origin_dx, origin_dy, myMap_res_factor); // converts from metric coordinates to matrix index
	double ang_sweep_start_rad = botAngle-(ang_max_rad/2); //essentially turns the raylooker all the way to the left
	//double ang_node_start_rad;

	Point rayPt;
	Point nodePt;
	list<Point> segPtList;
	bool isPtValid = false;

	for (int i=0; i<(n+1); i++)
	{
		if (i<n)
		{		
			rayPt = raycast(myMap, 
								myMap_w,
								bPosit, 
								(ang_sweep_start_rad + i*ang_n_rad),
								ray_check_dist*myMap_res_factor,
								backup_dist*myMap_res_factor,
								isPtValid);
		}
		else
		{
			isPtValid = false; // simulates hitting an invalid point
									 // at the end of the sweep to force the 
									//program to try placing nodes one last time
		}


		if (isPtValid)
		{
			//writeSquare(debugMap, myMap_w, rayPt.x(), rayPt.y(), 1);
			//cout<<"open space at x:"<<rayPt.x()<<", y:"<<rayPt.y()<<endl;
			segCount++;
			segPtList.push_back(rayPt);
		}
		else
		//Only considers node placement after hitting an invalid point
		{	
			ang_seg_total_rad = segCount*ang_n_rad;
			if (ang_seg_total_rad > ang_seg_min_rad) //evaluates whether the cleared region is large enough to place nodes in
			{
				//robot's confirmed that a segment of the arc is clear
				double rem = fmod(ang_seg_total_rad, ang_seg_max_rad); 
				int nNode = floor(ang_seg_total_rad/ang_seg_max_rad) + 1;
				double ray_div;
				
				if ((nNode == 1) && (rem < ang_seg_min_rad))
				{
					nNode=0;
				}

				// don't place nodes on the edge of the free region
				ray_div = (double)segCount / (nNode+1); //this is the number of rays per node
				//cout<<"ray_div is "<<ray_div<<"for a total seg count of "<<segCount<<endl;
				list<Point>::iterator it;

				it = segPtList.begin();
				int j=1;
				int listLen = segPtList.size();
				for(int i=0; i<listLen; i++)
				{

					if (i==myRound(j*ray_div))
					{
						//ACTUALLY places the node
						nodePt = *it;
						//writeSquare(debugMap, myMap_w, nodePt.x(), nodePt.y(), 3);
						placed_nodes.push_back(nodePt);
						nodeCount++;
						j++;

						if (j>nNode) { break; }
					}
					
					it++;
								
				}
				
				
			}
			/*else if (segCount > 0)
			{
				cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
				cout<<"the raycast returned an open space,"<<endl;
				cout<<"but it was probably too narrow an opening to move through"<<endl;
				cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
			}*/
			
			//Reset until the next open segment
			segCount = 0;
			segPtList.clear();
		}
	}

	cout<<"Placed "<<nodeCount<<" nodes"<<endl;
	return nodeCount;
}


int myRound(double num)
{
	return (num - floor(num) < 0.5) ? floor(num) : ceil(num);
}










