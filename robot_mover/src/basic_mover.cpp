#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include<stdlib.h>
#include<math.h>

using namespace ros;
using namespace std;

// simple rounding of numbers
float round(float num){
     if (num-(int)num >= 0.5)
         return ceil(num);
     else
         return floor(num);
}

int main(int argc, char **argv){
     float time;
     // default linear speed is 0.3 m/s, default angular speed is 0.5 rad/s
     float radius, theta, radius_s = 0.3, theta_s = 0.5;
     /* overwrite the default speed if they are specified by the arguments
        absolute to make sure speed is positive */
     if(argc == 5){
         radius_s = abs(atof(argv[3]));
         theta_s = abs(atof(argv[4])*3.142/180);
     }
     radius = atof(argv[1]);
     theta = atof(argv[2])*3.142/180;
     // make the speed negative if the arguments specify negative radius or negative angle
     if(radius < 0)
        radius_s = -radius_s;
     if(theta < 0)
        theta_s = -theta_s;
/*
     cout << "Radius: " << radius << endl;
     cout << "Angle: " << theta << endl;
     cout << "Linear speed: " << radius_s << endl;
     cout << "Angular speed: " << theta_s << endl;  */
     init(argc, argv, "basic_mover");
     NodeHandle n;
     // publish to /cmd_vel, the topic that controls the speed of the Turtlebot
     Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
     geometry_msgs::Twist vel;
     // for publishing at a rate of 10 Hz
     Rate rate(10);
     int count = 0;
     // avoid division by zero
     if(theta_s == 0)
        time = 0;
     // calculate the number of times to publish
     else
     	time = theta/theta_s * 10;
     time = round(time);
     while(ok() && time != 0){
         vel.angular.z = theta_s;
         pub.publish(vel);
         rate.sleep();
         if(++count >= time)
             break;
     }
     vel.angular.z = 0;
     if(radius_s == 0)
         time = 0;
     else
    	 time = radius/radius_s * 10;
     time = round(time);
     count = 0;
     while(ok() && time != 0){
         vel.linear.x = radius_s;
         pub.publish(vel);
         rate.sleep();
         if(++count >= time)
             break;
     }
     return 0;
}
