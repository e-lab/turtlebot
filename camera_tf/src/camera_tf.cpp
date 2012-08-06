#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<math.h>

using namespace ros;

int main(int argc, char** argv){
	init(argc, argv, "camera_tf");
	tf::TransformBroadcaster broadcast;
	Rate rate(5);
	while(ok()){
		tf::StampedTransform camera_optical_frame(tf::Transform(
			tf::Quaternion(0,0,-sqrt(0.5),sqrt(0.5)) * tf::Quaternion(-sqrt(0.5),0,0,sqrt(0.5)),
			tf::Vector3(0.10, 0, 0.55)), Time::now(), "base_link", "camera");
		broadcast.sendTransform(camera_optical_frame);
		tf::StampedTransform camera_frame(tf::Transform(tf::Quaternion(0,0,0,1.0), tf::Vector3(0.10,0,0.55)),
			Time::now(), "base_link", "camera_frame");
		broadcast.sendTransform(camera_frame);
		rate.sleep();
	}
	return 0;
}
