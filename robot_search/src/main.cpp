#include<ros/ros.h>
#include<iostream>
#include<list>
#include<string>
#include<sstream> //for number to string conversion
#include<sys/stat.h>
#include<sys/mman.h>
#include<fcntl.h>
#include<unistd.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include "node.hpp"
#include "mapUtils.h"
#include "robot_search.hpp"
#include "mover.h"

#define PI 3.14159
//#include <dos.h> //used to sleep();

using namespace std;
using namespace ros;

const double min_check_ang_deg = 10; //the angle at which the turn_n_check function evaluates the _OLflag to see if it needs to stop sweeping
const double half_sweep_ang_deg = 45;
bool isResuming = false;

extern Point botPos;
extern double myMap_res_factor;
extern double origin_dx;
extern double origin_dy;
//extern signed char* debugMap;
extern int myMap_w;
bool backtrack = false;
int RATE;
int fd;
bool *shared_ptr;

multimap<Index, Node, compare> Node::node_list; // static member variable for Node class

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void init_OLflag();
bool check_OLflag();
void close_OLflag();
geometry_msgs::PoseStamped maptoBase(double, double, tf::TransformListener&, tf::StampedTransform&, double&);
void move_robot(double x, double y, tf::TransformListener&, tf::StampedTransform&);
void move_goal(double);
void turn_around();
void turn_n_check(double);
void sweep_n_check(double);
void tree_traverse(Node*, tf::TransformListener&, tf::StampedTransform&);
//void paintNodes();
void pubNode(Point);

int main(int argc, char **argv){
	init(argc, argv, "seeker");
	NodeHandle n;
	tf::TransformListener tf_listener;
	tf::StampedTransform st_tf;
	init_OLflag();
	system("cd ~/online-learner; sudo torch run.lua -F m -S -b 184 -d 4 --source=ros &");
	Node* start = new Node(botPos.x(), botPos.y());
	tree_traverse(start, tf_listener, st_tf);
	cout << "Done" << endl;
	close_OLflag();
	/*double dist = 0, ang = 0;
		cout << "Enter distance :";
		cin >> dist;
		cout << "Enter angle :";
		cin >> ang;
		basic_mover(dist,ang);*/
	return 0;
}

void tree_traverse(Node* parent, tf::TransformListener& tf_list, tf::StampedTransform& stamped_tf){
	//pause here if OL flag says it has a result
	
	bool found_obj = false;
	isResuming = false;
	found_obj = check_OLflag();
	if (found_obj)
	{
		cout<< "SUCCESSFULLY FOUND OBJECT!" << endl;
		close_OLflag();
		getchar();
		exit(0);
	}	
	/*while (pause)
	{
		sleep(3);
		pause = check_OLflag();
		isResuming = true;	
	}*/
	
	//if you just unpaused a second ago, send nav goal to current node	
	if (isResuming)
	{
		cout << "Restarting" << endl;//send_nav_goal(parent) <- something like that
	}	

	bool turn_flag = false;
	list<Node*>::iterator i;
	list<Node*> child_list = parent->getChildren();
	if (child_list.empty()){
		list<Point> nodes = robot_search();
		if(nodes.empty()){
			cout << "Turning 180 degrees" << endl;
			turn_n_check(180);
			turn_flag = true;
			nodes = robot_search();
			cout << "No nodes :(" << endl;
		}
		if(!nodes.empty()){
			list<Point>::iterator it;
			addnode:
			int count = 0;
			for(it = nodes.begin(); it != nodes.end(); it++){
				Point mapPos = matrixToMapPos(*it, origin_dx, origin_dy, myMap_res_factor);
				if(!Node::checkNearby(mapPos.x(), mapPos.y())){
					Node* child = parent->addChild(mapPos.x(), mapPos.y());
					Node::add_node(*child);
					pubNode(child->getPoint());
					count++;
				}
				else{
					cout << mapPos.x() << " " << mapPos.y() << " got dropped" << endl;
				}
			}
			if (count == 0){
				if (!turn_flag){
					cout << "Turning 180 degrees" << endl;
					turn_n_check(180);
					nodes = robot_search();
					turn_flag = true;
					//cout << "All nodes got dropped" << endl;
					goto addnode;
				}
				else
				{
					//Already tried turning, but no new results, so backtrack
					//Found dead-end
					cout << "No nodes after turn around" << endl;
					backtrack = true;
					return;
				}
			}
		}
		else
		{
			//Found dead-end
			backtrack = true;
			return;
		}	
	}

	child_list = parent->getChildren();
	backtrack = false;
	for(i=child_list.begin(); i!=child_list.end(); i++){
		storeBotPose(tf_list, stamped_tf);
		Point currentPos(botPos.x(),botPos.y());
		Point movePos = (*i)->getPoint();
		/*cout << "Current Position of robot " << currentPos.x() << " " << currentPos.y() << endl;
		cout << "Destination Postion of robot " << movePos.x() << " " << movePos.y() << endl;
		cout << "Enter any key to continue" << endl;
		getchar();
		*/
		move_robot(movePos.x(), movePos.y(), tf_list, stamped_tf);
		sweep_n_check(half_sweep_ang_deg); //this sweeps but gets interrupted if it sees the objectf
		(*i)->explored(true);
		tree_traverse(*i, tf_list, stamped_tf);
	}
}

void init_OLflag(){
	fd = shm_open("/OL_flag", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	if(fd<0){
		shm_unlink("/OL_flag");
		fd = shm_open("/OL_flag", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	}
	cout << "File descriptor: " << fd << endl;
	ftruncate(fd, sizeof(bool));
	shared_ptr = (bool*)mmap(0, sizeof(bool), (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
}

bool check_OLflag(){
	lockf(fd, F_LOCK, sizeof(bool));
	bool flag = *shared_ptr;
	lockf(fd, F_ULOCK, sizeof(bool));
	return flag;
}

void close_OLflag(){
	shm_unlink("/OL_flag");
}

geometry_msgs::PoseStamped maptoBase(double x, double y, tf::TransformListener& tf_list, tf::StampedTransform& stamped_tf, double& angle_relative_rad)
{
	bool error = true;
	while(error){
		try{
			error = false;
			tf_list.lookupTransform("/base_link", "/map", Time(0), stamped_tf);
		}
		catch(tf::TransformException ex){
			error = true;
		}
	}
	geometry_msgs::PoseStamped map, base;
	map.header.frame_id = "map";
	map.pose.position.x = x;
	map.pose.position.y = y;
	map.pose.orientation.w = 1.0;
	Time current_time = Time::now();
	tf_list.getLatestCommonTime("map","base_link",current_time,NULL);
	map.header.stamp = current_time;
	tf_list.transformPose("base_link", map, base);
	double x_length, y_length;
	x_length = base.pose.position.x;
	y_length = base.pose.position.y;
	angle_relative_rad = atan2(y_length, x_length);
	base.pose.orientation.w = cos(angle_relative_rad/2);
	base.pose.orientation.z = sin(angle_relative_rad/2);
	return base;
}

void move_robot(double x, double y, tf::TransformListener& tf_list, tf::StampedTransform& stamped_tf){
	//moves robot using navigation goals

	double angle_rad;
	geometry_msgs::PoseStamped base;
	base = maptoBase(x,y,tf_list,stamped_tf,angle_rad);
	//if(backtrack){
		MoveBaseClient client("move_base", true);
		while(!client.waitForServer(Duration(2.0))){
			//cout << "Still waiting for move_base to connect" << endl;
		}
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose = base;
		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = Time::now();
	
		//cout << "Target goal " << goal.target_pose.pose.position.x << " " << goal.target_pose.pose.position.y << endl;
		/*
		char input;
		cout << "Send goal? ";
		cin >> input;
		if(input != 'y')
			return;*/
		client.sendGoal(goal);
		while(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
		client.getState() != actionlib::SimpleClientGoalState::ABORTED){
			sleep(0.1);
			bool flag = check_OLflag();
			if(flag){
				client.cancelAllGoals();		
				cout<< "SUCCESSFULLY FOUND OBJECT DURING BACKTRACK!" << endl;
				close_OLflag();
				exit(0);
			}
		}
		if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			//cout << "Reached destination" << endl;
		}
		else{
			cout << "Failed to reach destinmoveation" << endl;
		}
	//}
	/*else{
		double angle = angle_rad * 180 / PI;
		double x_length, y_length, total_length;
		x_length = base.pose.position.x;
		y_length = base.pose.position.y;
		total_length = sqrt(x_length*x_length + y_length*y_length);
		basic_mover(0,angle);
		basic_mover(total_length,0);
		//string angle_str = boost::to_string(angle);
		//string total_length_str = boost::to_string(total_length);
		//cout << "Angle " << angle_str;
		//cout << " Total Length " << total_length_str << endl;
		//string cmd = "~/ros_workspace/robot_mover/bin/basic_mover 0 " + angle_str;
		//system(cmd.c_str());
		//cmd = "~/ros_workspace/robot_mover/bin/basic_mover " + total_length_str + " 0";
		//system(cmd.c_str());
		//cout << "Executed smooth_mover" << endl;
	}*/
}

void move_goal(double angle){
	double angle_rad = angle*PI/180;
	MoveBaseClient client("move_base", true);
	while(!client.waitForServer(Duration(2.0))){
		//cout << "Still waiting for move_base to connect" << endl;
	}
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.pose.orientation.w = cos(angle_rad/2);
	goal.target_pose.pose.orientation.z = sin(angle_rad/2);
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = Time::now();
	client.sendGoal(goal);
	while(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
	client.getState() != actionlib::SimpleClientGoalState::ABORTED){
		sleep(0.1);
		bool flag = check_OLflag();
		if(flag){
			cout<< "SUCCESSFULLY FOUND OBJECT!" << endl;
			close_OLflag();
			getchar();
			exit(0);
		}
	}
	if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		//cout << "Reached destination" << endl;
	}
	else{
		cout << "Failed to reach destination" << endl;
	}
}
void turn_around(){
	system("~/ros_workspace/robot_mover/bin/basic_mover 0 180");
	cout << "Turned around successfully" << endl;
}

void turn_n_check(double target_ang){
	//NOTE: all angles in this function are in degrees
	//returns whether it found the object (true) or not (false)
	/*double ang_moved = 0;
	double ang_incr = min_check_ang_deg;
	double ang_move_now;

	bool found_obj = false;	
	
	//string move_path = "~/ros_workspace/robot_mover/bin/basic_mover 0 ";
	//string move_cmd;

	int L_or_R = (target_ang > 0) ? 1 : -1; //decides whether to rotate left or right
	target_ang *= L_or_R; //essentially, takes the absolute value
	
	while ( !found_obj and (ang_moved < target_ang) )
	{

		if ((ang_moved + min_check_ang_deg) > target_ang)
		{
			//this block prevents overshooting the target angle
			ang_incr = target_ang - ang_moved;
		}
		else
		{
		 	ang_incr = min_check_ang_deg; //not necessary?
		}

		ang_move_now = ang_incr * L_or_R;
		basic_mover(0, ang_move_now);
		//move_cmd = move_path + boost::to_string(ang_move_now); //concatenates the string conversion of the angle with the template move path
		//system(move_cmd.c_str()); //actually executes the turn
		
		ang_moved += ang_incr;	
		found_obj = check_OLflag(); //checks whether it can see the object yet, or not
		//cout<<"now "<<ang_move_now<<" total moved "<<ang_moved<<endl;
	}*/
	cout << "Target angle " << target_ang << endl;
	move_goal(target_ang);
	//return found_obj;
}

void sweep_n_check(double half_sweep_ang)
{
	//sweeps left and right, continually checking for the object
	bool found_obj = false;
	int phase = 0;
	while ( (!found_obj) and (phase <3) )
	{
		switch (phase)
		{
			case 0:
				turn_n_check(half_sweep_ang);
				break;
			case 1:
				turn_n_check(-2*half_sweep_ang);
				break;
			case 2:
				turn_n_check(half_sweep_ang);
				break;
		}
		phase++;
	}
}

/*void paintNodes(){
		multimap<Index, Node, compare> node_list = Node::getList();
		multimap<Index, Node, compare>::iterator node_it;
		for(node_it = node_list.begin(); node_it != node_list.end(); node_it++){
			Point node_pos = botPosToMatrix(node_it->second.getPoint(), origin_dx, origin_dy, myMap_res_factor);
			if(node_it->second.getStatus())
				writeSquare(debugMap, myMap_w, node_pos.x(), node_pos.y(), 1);
			else
				writeSquare(debugMap, myMap_w, node_pos.x(), node_pos.y(), 2);
		}
}*/

void pubNode(Point point){
	cout << "publishing" << endl;
	NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Pose>("/map_nodes",1);
	geometry_msgs::Pose node;
	node.position.x = point.x();
	node.position.y = point.y();
	node.orientation.w = 1.0;	
	Rate rate(10);
	int count = 0;
	while(count!=5){
		pub.publish(node);
		rate.sleep();
		count++;
	}
}
