#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include<stdlib.h>
#include<math.h>

using namespace ros;
using namespace std;

#define PI 3.14159

const int RATE = 100; //must be between 50-100
// default speeds
double radius_s = 0.3;
double theta_s = 0.7;

// simple rounding of numbers
double round(double num){
     if (num-(int)num >= 0.5)
         return ceil(num);
     else
         return floor(num);
}

void basic_mover(double radius, double theta){
	double time;
	int count = 0;
	theta = theta*PI/180;
	if(radius < 0)
        radius_s = -radius_s;
   if(theta < 0)
        theta_s = -theta_s;
	NodeHandle n;
	// publish to /cmd_vel, the topic that controls the speed of the Turtlebot
   Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
   geometry_msgs::Twist vel;
   // for publishing at a rate of 10 Hz
   Rate rate(RATE);
	time = theta/theta_s * RATE;
	time = round(time);
   while(ok() && time != 0){
       vel.angular.z = theta_s;
       pub.publish(vel);
       
       if(++count >= time)
           break;
		 rate.sleep();
   }
	vel.angular.z = 0;
	time = radius/radius_s * RATE;
	time = round(time);
	while(ok() && time != 0){
       vel.linear.x = radius_s;
       pub.publish(vel);
       
       if(++count >= time)
           break;
		rate.sleep();
   }
}
