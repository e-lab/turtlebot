#include<ros/ros.h>
#include<turtlebot_node/TurtlebotSensorState.h>
#include<turtlebot_node/SetTurtlebotMode.h>
#include<turtlebot_node/SetDigitalOutputs.h>
#include<iostream>

using namespace ros;
using namespace std;

int main(int argc, char **argv){
	init(argc, argv, "turtlebot_init");
	NodeHandle n;
	ServiceClient mode_client = n.serviceClient<turtlebot_node::SetTurtlebotMode>("/turtlebot_node/set_operation_mode");
	turtlebot_node::SetTurtlebotMode mode_service;
	mode_service.request.mode = turtlebot_node::TurtlebotSensorState::OI_MODE_FULL;
	if (mode_client.call(mode_service)){
		cout << "Turtlebot is set to full mode" << endl;
	}
	else
		cout << "Failed to set Turtlebot to full mode" << endl;
	ServiceClient breaker_client = n.serviceClient<turtlebot_node::SetDigitalOutputs>("/turtlebot_node/set_digital_outputs");
	turtlebot_node::SetDigitalOutputs breaker_service;
	breaker_service.request.digital_out_0 = 1;
	if (breaker_client.call(breaker_service)){
		cout << "Turtlebot breaker 0 for Kinect is turned on" << endl;
	}
	else
		cout <<"Failed to turn on Turtlebot breaker 0 for Kinect" << endl;
	return 0;
}
