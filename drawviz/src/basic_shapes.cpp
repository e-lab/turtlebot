//#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

//using namespace std;
using namespace ros;
int main (int argc, char** argv)
{
	init(argc, argv, "basic_painter");
	NodeHandle nh;
	Rate r(1);

	Publisher paint_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	uint32_t shape = visualization_msgs::Marker::CUBE;

	while(ok())
	{
		visualization_msgs::Marker marker;
		
		marker.header.frame_id = "/map";
		marker.header.stamp = Time::now();
		
		
		marker.ns = "basic_shapes";
		marker.id = 0;

		marker.type = shape;

		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 0.5;

		marker.color.r = 0.016f;
		marker.color.g = 0.830f;
		marker.color.b = 0.600f;
		marker.color.a = 0.75;

		marker.lifetime = ros::Duration();

		paint_pub.publish(marker);

		switch (shape)
		{
			case visualization_msgs::Marker::CUBE:
				shape = visualization_msgs::Marker::SPHERE;
				marker.color.g = 0.1f;
				break;
			case visualization_msgs::Marker::SPHERE:
				shape = visualization_msgs::Marker::ARROW;
				marker.color.g = 0.3f;				
				break;
			case visualization_msgs::Marker::ARROW:
				shape = visualization_msgs::Marker::CYLINDER;
				marker.color.g = 0.5f;				
				break;
			case visualization_msgs::Marker::CYLINDER:
				shape = visualization_msgs::Marker::CUBE;
				marker.color.g = 0.83f;				
				break;
		}
		r.sleep();
	}

}
