#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include<stdlib.h>
#include<math.h>

//only for debug
#include "std_msgs/String.h"

#include "ramp_calc.h"
#include "global.h"

using namespace ros;
using namespace std;


/*	This function is designed to move in a smoother fashion
 * with a ramp up and ramp down in the velocity.
 *
 * v(t) = _.----._ instead of __-----__
 *
 * this version assumes known velocity for the constant portion (v_c)
 * and ramp up time (t_r)
 *
 *
 *
 */

//CONSTANTS, L = linear, A = angular, f = final dist/angle, _c = constant
double Lf, Af;
double L_dist_left, A_dist_left;

//the following are used in ramp_calc.cpp
double Lv_c = 0.3; // (meter/sec)
double Av_c = 0.6;	// (rad/sec)
double Ld_ramp = 0.5;
double Ad_ramp = degToRad(1);	// (cm), (rad) ramp distance

const double time_step = 0.1; //(seconds) 

const double Lv_min = 0.1;
const double Av_min = 0.99*Av_c;



//VARIABLE, time
double t;
	
double Lv = Lv_min;
double Av = Av_min;
string Lv_name = "lin_vel";
string Av_name = "ang_vel";

bool L_isNeg = false;
bool A_isNeg = false;

//INITIALIZATIONS

//Setup publishers out here, so they can be seen below
Publisher vel_pub;

//msgs
geometry_msgs::Twist vel;

//function prototypes
int setVel(NodeHandle*);		//sets vel on rosparam server
bool getVel(NodeHandle*);	//gets vel from rosparam server
void pubVel(); 				//publishes vel to a topic


int main(int argc, char **argv)
{

	//INPUTS
		//overwrite default variables if input exists
	if (argc == 1) { return 1; }

	else if (argc > 1) { Lf = atof(argv[1]); }
	if (argc > 2) { Af = degToRad(atof(argv[2])); }
	if (argc > 3) { Lv_c = atof(argv[3]); }
	if (argc > 4) { Av_c = atof(argv[4]); }
	if (argc > 5) { Ld_ramp = atof(argv[5]); }
	
	//INITIALIZE the node
	init(argc, argv, "smooth_mover");
	NodeHandle n;       

	//INITIALIZING the rampup
	calc_dv_long();
	calc_dv_short(Lf, Af);
	
	//get the first velocity from the parameter server		
	getVel(&n);

	
	L_dist_left = Lf; //this variable is used to plan motion
	A_dist_left = Af;

	// "/cmd_vel" is the topic name, and 5 is the queue buffer
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",5);

    //for publishing at a rate of 10 Hz
	Rate loop_rate(1/time_step);
	

	while (ok())
	{	
		Lv = getNewV(Lv, Lf, L_dist_left, false);
		L_dist_left -= Lv*time_step;

		Av = getNewV(Av, Af,  A_dist_left, true);	
		A_dist_left -= Av*time_step;

		if	(Lv<=Lv_min)
		{
			Lv = 0;
		}
		if (Av <= Av_min)
		{
			Av = 0;
		}
		if ((Lv <= Lv_min)&&(Av <= Av_min))
		{
			setVel(&n);
			pubVel();
			break;
		} //if the robot stops or drops below the whine speed
																	//then stop 


			
		//set parameters in rosparam server
		setVel(&n);
		pubVel();
		loop_rate.sleep(); 	//sleeps for the rest of the rate cycle	
	
	}
	
	return 0;	
}


int setVel(NodeHandle *nPtr)
{
	nPtr->setParam(Lv_name, double(Lv));
	nPtr->setParam(Av_name, double(Av));
	return 0;
}

bool getVel(NodeHandle *nPtr)
{	
	bool success = true;
	//NOTE: currently rosparam does not support doubles
	//as a workaround we create a temporary variable (double)
	double Lv_dub;
	double Av_dub;
	if (Lf != 0)
	{
		if(nPtr->getParam(Lv_name, Lv_dub))
		{
			Lv = Lv_dub; //cast as double
			cout << "grabbed " << Lv << "\n";
			if (Lv < Lv_min) {Lv = Lv_min;}
			
			//ASSUME: always using a positive velocity
			if (Lf < 0)
			{
				L_isNeg = true;
				Lf = -Lf;
			}

		}
		else
		{
			cout << "linear velocity does not exist yet\n";
			success = false;
		}
	}

	if (Af != 0)
	{
		if(nPtr->getParam(Av_name, Av_dub))
		{
			Av = Av_dub;
			cout << "grabbed " << Av << "\n";
			if (Av < Av_min) {Av = Av_min;}
			

			if (Af < 0) //make everything positive, do the negative at publishing
			{
				A_isNeg = true;
				Af = -Af;
			}
			
		}
		else
		{
			cout << "Angular velocity does not exist yet\n";
			success = false;
		}
	}
	return success;
}

void pubVel()
{
	//Custom publish function that makes the code a little cleaner
	//ROS_INFO("%s", d_vel.data.c_str());

	//recalculate vel
	vel.linear.x = (L_isNeg) ? -Lv : Lv; //flip sign if is negative
	vel.angular.z = (A_isNeg) ? -Av : Av;

	vel_pub.publish(vel);
}




















