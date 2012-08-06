#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<iostream>
#include<math.h>

using namespace ros;
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	init(argc, argv, "navigate_goals");
	NodeHandle n;
	
	MoveBaseClient client("move_base", true);
	while(!client.waitForServer(Duration(2.0))){
		cout << "Still waiting for move_base to connect" << endl;
	}
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = Time::now();
	double x, y, angle;
	cout << "Enter x: ";
	cin >> x;
	cout << "Enter y: ";
	cin >> y;
	cout << "Enter angle: ";
	cin >> angle;
	angle = angle*3.14159/180;
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation.w = cos(angle/2);
	goal.target_pose.pose.orientation.z = sin(angle/2);
	
	client.sendGoal(goal);
	client.waitForResult();
	
	if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		cout << "Success!!!" << endl;
	}
	else{
		cout << "Epic fail" << endl;
	}
	
	return 0;
}
