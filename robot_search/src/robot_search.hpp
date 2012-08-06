#include<nav_msgs/OccupancyGrid.h>

void store_map_cb(const nav_msgs::OccupancyGrid::ConstPtr&);
void storeBotPose(tf::TransformListener&, tf::StampedTransform&);
int node_sweep(double, int);
int myRound(double);
std::list<Point> robot_search();
void pubMap(const nav_msgs::OccupancyGrid::ConstPtr&);
