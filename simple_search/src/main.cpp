//NEEDS MOVE ALL THE WAY AND TURN N CHECK


//------ROS---------------------//
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
//------------------------------//

#include<stdlib.h>
#include<time.h> //for random number seed
#include<iostream>
#include<string>
#include<sstream> //for number to string conversion

//-----------------------------//
//for shared memory/OL_flag stuff
#include<sys/stat.h>
#include<sys/mman.h>
#include<fcntl.h>
#include<unistd.h>
//-----------------------------//



using namespace ros;
using namespace std;

//for shared memory
int fd;
bool *shared_ptr;
bool blankedOnce = false;

int dummy = 0;

int current_dir = 0; // -1 means right, 1 means left

bool kin_data_isFresh = false;
int kin_num_pts = 360;

double kin_min; //[m], the smallest valid distance measured
double kin_range_min; //[m], the minimum distance the sensor can measure, around 0.45m
double kin_range_max; //[m], the maximum distance the sensor can measure, aroun 11m
//vector<float> kin_data;
vector<float>::const_iterator k_it;
vector<float>::const_iterator k_start;
vector<float>::const_iterator k_end;

const double min_obj_dist = 0.75;// [m] smallest dist the robot will allow between itself and an object
const double min_move_dist = 0.05; // [m] the smallest distance the robot will actually move
const double min_check_ang_deg = 30; // [degress] the angle at which the turn_n_check function evaluates the _OLflag to see if it needs to stop sweeping
const double min_check_dist = 0.1;

const double RIGHT_ANGLE = 85; //less than 90 to correct for overturning


//--function prototypes--

//MOVEMENT
bool move_to_wall();
//bool move_n_check(double);

int sweep_n_check();
int sweep_n_check_rand();
bool turn_n_check(double);

bool pick_dir_n_turn(int);
void move_bot(double, double);
int left_or_right();

//DATA
void updateKinData(const sensor_msgs::LaserScan::ConstPtr&);
double get_kin_min();	//waits for fresh data, then grabs the min
double calc_kin_min(); //directly grabs without checking if it's fresh
bool check_kinect_for_wall();
void check_n_exit(bool);

//SHM
void init_OLflag();
bool check_OLflag();
void close_OLflag();

//OVERALL
void dive_n_explore();



int main(int argc, char **argv){
	cout<<"Simple Search Started"<<endl;
	//nodes
	system("mplayer  -really-quiet ~/ros_workspace/simple_search/sound/start.mp3");
	init(argc, argv, "simple_seeker");
	NodeHandle nh;
	system("cd ~/online-learner; torch run.lua -F m -S -d 4 -b 184 --source=ros &");
	Subscriber kin_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",100,updateKinData);
	init_OLflag();
	

	while(ok())
	{
		dive_n_explore();//goes down one hallway, checking for the obj
		spinOnce();
	}
	

	close_OLflag();
	return 0;
}


//---------------------------------------------------------------------------//
//		OVERALL CONTROL
//---------------------------------------------------------------------------//


void dive_n_explore()
{
	int wall_open_num = 0;//used to decide where to go next
	current_dir = 0;

	//dive forward
	bool found_obj = move_to_wall();
	check_n_exit(found_obj);
	//check left and right
	wall_open_num = sweep_n_check_rand();
	
	//pick a direction
	found_obj = pick_dir_n_turn(wall_open_num);
	check_n_exit(found_obj);
}
//---------------------------------------------------------------------------//
//		MOVEMENT
//---------------------------------------------------------------------------//
bool move_to_wall()
{
	//moves towards a wall, updating kinect and online-learner (OL) information
	//somewhat equivalent to move_all_the_way (can be found in ~/online-learner/aux/old/junk.lua

	bool found_obj = false;
	double target_dist = get_kin_min(); //distance to the nearest object
	double dist_moved = 0;
	double dist_incr = min_check_dist;	
	double dist_move_now;

	int F_or_B = (target_dist > 0) ? 1 : -1; //decides whether to move forward or backward
	target_dist *= F_or_B; //essentially, takes the absolute value
	while (!found_obj and (target_dist >= min_obj_dist) and (dist_moved < target_dist))
	{
		if ((dist_moved + min_check_dist) > target_dist)
		{
			//prevents overshooting
			dist_move_now = target_dist - dist_moved;
		}
		else
		{
			dist_move_now = dist_incr;
		}
		
		dist_moved += dist_move_now;
		if (dist_move_now < min_move_dist) {dist_move_now = 0;}
		dist_move_now *= F_or_B;
		move_bot(dist_move_now, 0);

		if (kin_data_isFresh)
		{
			//recalculates the distance if the kinect data is new
			dist_moved = 0;
			target_dist = calc_kin_min();
			kin_data_isFresh = false;
		}
		found_obj = check_OLflag();		
		spinOnce();
	}
	return found_obj;
}
/*
bool move_n_check(double target_dist)
{
	bool found_obj = false;
	//returns whether it found the object (true) or not (false)
	double dist_moved = 0;
	double dist_incr = min_check_dist;
	double dist_move_now;

	int F_or_B = (target_dist > 0) ? 1 : -1; //decides whether to move forward or backward
	target_dist *= F_or_B; //essentially, takes the absolute value
	cout<<"moving forward"<<endl;
	while ( !found_obj and (dist_moved < target_dist) )
	{
		if ((dist_moved + min_check_dist) > target_dist)
		{
			//this block prevents overshooting the target distance
			dist_incr = target_dist - dist_moved;
		}
		else
		{
		 	dist_incr = min_check_dist;
		}


		dist_move_now = dist_incr * F_or_B;
		move_bot(dist_move_now, 0);
		dist_moved += dist_incr;	

		found_obj = check_OLflag();
		cout<<"now "<<dist_move_now<<" total moved "<<dist_moved<<endl;
	}
	return found_obj;
}*/


int sweep_n_check()
{
	//NOTE: this code is modeled after main.cpp in ~/ros_workspace/robot_search/src
	//but is instead checking for path availablity (wall_open_num)
	//and object recognition (found_obj)

	const int half_sweep_ang = RIGHT_ANGLE;
	int phase = 0;
	int wall_open_num = 0;
	//return 0 for no available
	// +1 for left available
	// +2 for right avaialable
	// i.e. 3 means both left and right are available

	bool found_obj = false;

	
	while ( (!found_obj) and (phase <3 ) )
	{
		switch (phase)
		{
			case 0:
				//turn left
				found_obj = turn_n_check(half_sweep_ang);
				wall_open_num += (check_kinect_for_wall() ? 0 : 1);
				current_dir = 1;
				break;
			case 1:
				//turn right
				found_obj = turn_n_check(-2*half_sweep_ang);
				wall_open_num += (check_kinect_for_wall() ? 0 : 2);
				current_dir = -1;				
				break;
			case 2:
				//taking out turn to center
				//found_obj = turn_n_check(half_sweep_ang);
				break;
		}
		
		phase++;
	}
	check_n_exit(found_obj);//exits if found	
	return wall_open_num;
}

int sweep_n_check_rand()
{
	int L_or_R = left_or_right();
	int half_sweep_ang = RIGHT_ANGLE * L_or_R;
	int phase = 0;
	int wall_open_num = 0;

	//return 0 for no available
	// +1 for left available
	// +2 for right avaialable
	// i.e. 3 means both left and right are available

	bool found_obj = false;
	
	bool is_wall = true;
	
	while ( (!found_obj) and (phase <2) and (wall_open_num == 0))
	{
		switch (phase)
		{
			case 0:
				//turn one way
				found_obj = turn_n_check(half_sweep_ang);
				current_dir = L_or_R;
				break;
			case 1:
				//turn right
				found_obj = turn_n_check(-2*half_sweep_ang);
				current_dir = -L_or_R;				
				break;
		}
		is_wall = check_kinect_for_wall();
		if (!is_wall)
		{
			if (current_dir == 1)
			{
				//left is open
				wall_open_num = 1;
			}
			else
			{
				//right is open
				wall_open_num = 2;
			}
		}

		
		cout<<current_dir<<" is blocked?"<<check_kinect_for_wall()<<endl;
		phase++;
	}
	check_n_exit(found_obj);//exits if found	
	return wall_open_num;
}
bool turn_n_check(double target_ang)
{
	bool found_obj = false;
	//NOTE: all angles in this function are in degrees
	//returns whether it found the object (true) or not (false)
	double ang_moved = 0;
	double ang_incr = min_check_ang_deg;
	double ang_move_now;

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
		move_bot(0, ang_move_now);
		ang_moved += ang_incr;	

		found_obj = check_OLflag();
		//cout<<"now "<<ang_move_now<<" total moved "<<ang_moved<<endl;
	}
	
	return found_obj;
}

bool pick_dir_n_turn(int walls_open_num)
{	
	//assumes you are already facing right
	double current_ang = RIGHT_ANGLE*current_dir;

	//all angles in degrees
	double turn_ang = 0;
	bool found_obj;

	switch (walls_open_num)
	{
		case 0:
			//blocked
			turn_ang = -2*RIGHT_ANGLE;
			system("mplayer  -really-quiet ~/ros_workspace/simple_search/sound/dead_end.mp3");
			break;

		case 1:
			//left
			turn_ang = RIGHT_ANGLE;
			break;

		case 2:
			//right
			turn_ang = -RIGHT_ANGLE;
			break;

	/*	case 3:		
			turn_ang = RIGHT_ANGLE * left_or_right();*/
	}

	//prevents you from turning right, back to center then right again
	turn_ang -= current_ang;
	//cout<<"decided to turn "<<turn_ang<<endl;
	found_obj = turn_n_check(turn_ang);
	return found_obj;
}




void move_bot(double linear_dist, double angular_dist)
{
	//const string s_move_path = "~/ros_workspace/robot_mover/bin/smooth_mover";
	const string move_path = "~/ros_workspace/robot_mover/bin/basic_mover";
	//string move_path;
	/*if (angular_dist != 0)
	{
		move_path = b_move_path;
	}
	else
	{
		move_path = s_move_path;
	}*/

	const string spc = " ";
	string move_cmd;
	stringstream ss_lin;//for num-string conversion
	stringstream ss_ang;
	
	ss_lin<<linear_dist;
	ss_ang<<angular_dist;

	move_cmd = move_path + spc + ss_lin.str() + spc + ss_ang.str();
	system(move_cmd.c_str());
}

int left_or_right()
{
	srand(time(0)); //sets random seed
	int l_or_r = rand() % 2;	
	//randomly chooses between left and right
	l_or_r = (l_or_r == 0) ? -1 : 1;	
	return l_or_r;
}

//---------------------------------------------------------------------------//
//		DATA HANDLING/CHECKING
//---------------------------------------------------------------------------//

void updateKinData(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	kin_data_isFresh = true;
	//store data
	//maybe only check constants once...
	kin_range_min = scan_msg->range_min;
	kin_range_max = scan_msg->range_max;
	kin_num_pts = ((scan_msg->angle_max) - (scan_msg->angle_min)) / scan_msg->angle_increment;

	//kin_data = scan_msg->ranges;
	k_start = scan_msg->ranges.begin();
	k_end = scan_msg->ranges.end();
	
}

double get_kin_min()
{
	//waits, calculates, then retreives the distance to the closest object;
	kin_data_isFresh = false;
	
	Rate loop_rate(5); //check for kinect data at 5Hz
	// now that you've moved, spin until new data appears
	while (!kin_data_isFresh and ok())
	{
		//waiting for data
		loop_rate.sleep();
		spinOnce();
	}

	kin_min = calc_kin_min();
	return kin_min;
}

double calc_kin_min()
{
	double k_min = kin_range_max; //local version of kin_min	
	for (k_it = k_start; *k_it < kin_range_max; k_it++)
	{
		//skipping weird anomolies at the front of the scan
	}
	for (k_it=k_it; k_it < k_end; k_it++)
	{
		if ((*k_it >= kin_range_max) or (*k_it < kin_range_min))
		{
			//invalid point
		} 
		else
		{
			if (*k_it < k_min)
			{
				//updates minimum
				k_min = *k_it;
			}
		}
	}

	if (k_min == kin_range_max)
	{
		if (blankedOnce)
		{
			k_min = 0;
			cout<<"blanked out, too close?"<<endl;
			blankedOnce = false;
		}
		else
		{
			k_min = 0.9;
			cout<<"maybe blanked"<<endl;
			blankedOnce = true;
		}
	}
	else
	{
		blankedOnce = false;
	}

	cout <<"distance to wall is: "<<k_min<<endl;
	return k_min;
}
	


bool check_kinect_for_wall()
{
	bool isWall = false;
	isWall = (get_kin_min() <= min_obj_dist) ? true : false;
	return isWall;
}

void check_n_exit(bool objIsFound)
{
	if (!objIsFound)
	{
		return;
	}
	else
	{
		cout<< "SUCCESSFULLY FOUND OBJECT!" << endl;
		system("mplayer -really-quiet ~/ros_workspace/simple_search/sound/found_obj.mp3");
		close_OLflag();
		getchar();
		exit(0);
	}
}

//---------------------------------------------------------------------------//
//		SHARED MEMORY
//---------------------------------------------------------------------------//

void init_OLflag(){
	fd = shm_open("/OL_flag", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	if(fd<0){
		shm_unlink("/OL_flag");
		fd = shm_open("/OL_flag", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	}
	ftruncate(fd, sizeof(bool));
	shared_ptr = (bool*)mmap(0, sizeof(bool), (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
}

bool check_OLflag(){
	lockf(fd, F_LOCK, sizeof(bool));
	bool flag = *shared_ptr;
	lockf(fd, F_ULOCK, sizeof(bool));
	return flag;
	return false;
}
void close_OLflag(){
	shm_unlink("/OL_flag");
}





















