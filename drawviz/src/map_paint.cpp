#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

//bot location
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include <stdlib.h>
#include <iostream>

typedef visualization_msgs::Marker V_Marker;
using namespace ros;
using namespace std;

V_Marker path_strip, nodes_list;
double update_path_diff = 0.1; //[m]

geometry_msgs::Point last_path_pt;
geometry_msgs::Point last_node_pt;
int i=0;
int j=0;

//function prototypes
geometry_msgs::Point getBotPos(tf::TransformListener&, tf::StampedTransform&);
bool isNewPt(geometry_msgs::Point, geometry_msgs::Point);
void paintNewNode(const geometry_msgs::Pose::ConstPtr&);
void init_markers();

int main( int argc, char** argv )
{
	init(argc, argv, "points_and_lines");
	NodeHandle nh;
	Subscriber nodes_sub = nh.subscribe("/map_nodes", 10, paintNewNode);
	Publisher marker_pub = nh.advertise<V_Marker>("visualization_marker", 10);

	Rate r(5);
	tf::TransformListener tf_listener(Duration(10));
	tf::StampedTransform st_tf;

	geometry_msgs::Point path_p;
	last_path_pt = getBotPos(tf_listener, st_tf);
	last_node_pt = last_path_pt;

	init_markers();
	while(ok())
	{
		
		path_strip.header.stamp = nodes_list.header.stamp = Time::now();
		
		path_p = getBotPos(tf_listener, st_tf);

		if (isNewPt(path_p, last_path_pt))
		{
			path_strip.points.push_back(path_p);	
		}
		
		
		
		marker_pub.publish(path_strip);
		marker_pub.publish(nodes_list);
		spinOnce();
		r.sleep();
	}

}

geometry_msgs::Point getBotPos(tf::TransformListener& tf_list, tf::StampedTransform& stamped_tf)
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

	geometry_msgs::PoseStamped base, map;
	base.header.frame_id = "base_link";
	base.pose.position.x = 0;
	base.pose.position.y = 0;
	base.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	Time current_time = Time::now();
	tf_list.getLatestCommonTime("base_link","map",current_time,NULL);
	base.header.stamp = current_time;
	tf_list.transformPose("map", base, map);
	return map.pose.position;
}


bool isNewPt(geometry_msgs::Point newPt, geometry_msgs::Point lastPt)
{
	bool isNew = false;

	if (abs(newPt.x - lastPt.x) > update_path_diff)
	{
		isNew = true;
	}	
	if (abs(newPt.y - lastPt.y) > update_path_diff)
	{
		
		isNew = true;
	}
	
	if (isNew)
	{
		lastPt = newPt;
	}
	return isNew;
}

void paintNewNode(const geometry_msgs::Pose::ConstPtr& msg)
{
	geometry_msgs::Point node_p;
	node_p = msg->position;
	
	if	(isNewPt(node_p, last_node_pt))
	{
		nodes_list.points.push_back(node_p);
		cout<<"new node"<<endl;
	}
}


void init_markers()
{
	//initialize markers
	path_strip.header.frame_id = nodes_list.header.frame_id = "/map";
	path_strip.ns = nodes_list.ns = "painted_path";
	path_strip.action = nodes_list.action = V_Marker::ADD;
	
	path_strip.id = 0;
	nodes_list.id = 1;

	path_strip.type = V_Marker::LINE_STRIP;
	nodes_list.type = V_Marker::CUBE_LIST;

	path_strip.scale.x = 0.025;
	nodes_list.scale.x = 0.2;
	nodes_list.scale.y = 0.2;
	nodes_list.scale.z = 0.2;

	path_strip.color.r = 0.87f;
	path_strip.color.g = 0.91f;
	path_strip.color.b = 0.10f;
	path_strip.color.a = 1.0;

	nodes_list.color.r = 0.02f;
	nodes_list.color.g = 0.83f;
	nodes_list.color.b = 0.60f;
	nodes_list.color.a = 1.0;
}
