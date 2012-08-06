#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<sys/stat.h>
#include<sys/mman.h>
#include<fcntl.h>
#include<unistd.h>
#include<iostream>
#include<stdio.h>

using namespace ros;
using namespace std;

int rgb_fd, depth_fd;
const int DATA_SIZE = 640*480*3;
const int DEPTH_DATA_SIZE = 640*480;

typedef struct image_data{
	unsigned char data[DATA_SIZE];
}image_data;

typedef struct depth_data{
	float data[DEPTH_DATA_SIZE];
}depth_data;

image_data* rgb_mapped_ptr;
depth_data* depth_mapped_ptr;

void rgb_image_callback(const sensor_msgs::ImageConstPtr& ros_image_ptr){
	lockf(rgb_fd,F_LOCK,sizeof(image_data));
	rgb_mapped_ptr = (image_data*)mmap(0, sizeof(image_data), (PROT_READ|PROT_WRITE), MAP_SHARED, rgb_fd, 0);
	vector<unsigned char>::const_iterator vec_it;
	int count = 0;
	for(vec_it = ros_image_ptr->data.begin(); vec_it!=ros_image_ptr->data.end(); vec_it++){
		rgb_mapped_ptr->data[count] = *vec_it;
		count++;
	}
	lockf(rgb_fd,F_ULOCK,sizeof(image_data));
}

void depth_image_callback(const sensor_msgs::ImageConstPtr& ros_image_ptr){
	lockf(depth_fd,F_LOCK,sizeof(depth_data));
	depth_mapped_ptr = (depth_data*)mmap(0, sizeof(depth_data), (PROT_READ|PROT_WRITE), MAP_SHARED, depth_fd, 0);
	vector<unsigned char>::const_iterator vec_it;
	float *float_ptr;
	int count = 0;
	for(vec_it = ros_image_ptr->data.begin(); vec_it!=ros_image_ptr->data.end(); vec_it=vec_it+4){
		float_ptr = (float*)&(*vec_it);
		if(*float_ptr != *float_ptr)
			*float_ptr = 10; // arbitrary number for bad data
		depth_mapped_ptr->data[count] = *float_ptr;
		count++;
	}
	lockf(depth_fd,F_ULOCK,sizeof(depth_data));
}

int main(int argc, char** argv){
	init(argc, argv, "image_share");
	NodeHandle n;
	// opens the shared memory files
	rgb_fd = shm_open("/rgb_image_share", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	if (rgb_fd<0){
		shm_unlink("/rgb_image_share");
		rgb_fd = shm_open("/rgb_image_share", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	}
	depth_fd = shm_open("/depth_image_share", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	if (depth_fd<0){
		shm_unlink("/depth_image_share");
		depth_fd = shm_open("/depth_image_share", (O_CREAT|O_EXCL|O_RDWR), (S_IRUSR|S_IWUSR));
	}
	ftruncate(rgb_fd, sizeof(image_data));
	ftruncate(depth_fd, sizeof(depth_data));

	rgb_mapped_ptr = (image_data*)mmap(0, sizeof(image_data), (PROT_READ|PROT_WRITE), MAP_SHARED, rgb_fd, 0);
	depth_mapped_ptr = (depth_data*)mmap(0, sizeof(depth_data), (PROT_READ|PROT_WRITE), MAP_SHARED, depth_fd, 0);

	Subscriber rgb_sub = n.subscribe("/camera/rgb/image_color", 1, rgb_image_callback);
	Subscriber depth_sub = n.subscribe("/camera/depth/image", 1, depth_image_callback);
	spin();
	shm_unlink("/rgb_image_share");
	shm_unlink("/depth_image_share");
	return 0;
}
