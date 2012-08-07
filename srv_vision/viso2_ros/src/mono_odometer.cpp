#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <turtlebot_node/TurtlebotSensorState.h> // added by YL

#include <viso_mono.h>

#include "odometer_base.h"
#include "odometry_params.h"
// added by YL
// covariance constants and assignment methods are from turtlebot_node.py
const boost::array<double, 36> ODOM_POSE_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0, 
                											        0, 1e-3, 0, 0, 0, 0,
               											        0, 0, 1e6, 0, 0, 0,
																        0, 0, 0, 1e6, 0, 0,
																		  0, 0, 0, 0, 1e6, 0,
																		  0, 0, 0, 0, 0, 1e3}};

const boost::array<double, 36> ODOM_POSE_COVARIANCE2 = {{1e-9, 0, 0, 0, 0, 0, 
																			0, 1e-3, 1e-9, 0, 0, 0,
																			0, 0, 1e6, 0, 0, 0,
																			0, 0, 0, 1e6, 0, 0,
																			0, 0, 0, 0, 1e6, 0,
																			0, 0, 0, 0, 0, 1e-9}};

const boost::array<double, 36> ODOM_TWIST_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0, 
																			0, 1e-3, 0, 0, 0, 0,
																			0, 0, 1e6, 0, 0, 0,
																			0, 0, 0, 1e6, 0, 0,
																			0, 0, 0, 0, 1e6, 0,
																			0, 0, 0, 0, 0, 1e3}};

const boost::array<double, 36> ODOM_TWIST_COVARIANCE2 = {{1e-9, 0, 0, 0, 0, 0, 
																		    0, 1e-3, 1e-9, 0, 0, 0,
																			 0, 0, 1e6, 0, 0, 0,
																			 0, 0, 0, 1e6, 0, 0,
																			 0, 0, 0, 0, 1e6, 0,
																			 0, 0, 0, 0, 0, 1e-9}};
// end

namespace viso2_ros
{

class MonoOdometer : public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryMono> visual_odometer_;
  VisualOdometryMono::parameters visual_odometer_params_;

  image_transport::CameraSubscriber camera_sub_;

  bool replace_;

public:

  MonoOdometer(const std::string& transport) : OdometerBase(), replace_(false)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    camera_sub_ = it.subscribeCamera("/camera/rgb/image_color", 1, &MonoOdometer::imageCallback, this, transport);
    // make sure there is a covariance to start with (YL)
    setPoseCovariance(ODOM_POSE_COVARIANCE2);
	 setTwistCovariance(ODOM_TWIST_COVARIANCE2);
    // end
  }
  // added a public function to set covariance (YL)
  void setCovariance(bool flag){
	  if(flag){
         setPoseCovariance(ODOM_POSE_COVARIANCE2);
	 		setTwistCovariance(ODOM_TWIST_COVARIANCE2);
     }
     else{
         setPoseCovariance(ODOM_POSE_COVARIANCE);
	 		setTwistCovariance(ODOM_TWIST_COVARIANCE);
     }
  }

protected:

  void imageCallback(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      // read calibration info from camera info message
      // to fill remaining odometer parameters
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
		std::cout << "print" << model.fx() << std::endl;	
      visual_odometer_params_.calib.f  = model.fx();
      visual_odometer_params_.calib.cu = model.cx();
      visual_odometer_params_.calib.cv = model.cy();   
      visual_odometer_.reset(new VisualOdometryMono(visual_odometer_params_));
      if (image_msg->header.frame_id != "") setSensorFrameId(image_msg->header.frame_id);
      ROS_INFO_STREAM("Initialized libviso2 mono odometry "
                      "with the following parameters:" << std::endl << 
                      visual_odometer_params_);
    }
    // convert image if necessary
    uint8_t *image_data;
    int step;
    cv_bridge::CvImageConstPtr cv_ptr;
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
      step = image_msg->step;
    }
    else
    {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
      image_data = cv_ptr->image.data;
      step = cv_ptr->image.step[0];
    }

    // run the odometer
    int32_t dims[] = {image_msg->width, image_msg->height, step};
    // on first run, only feed the odometer with first image pair without
    // retrieving data
    if (first_run)
    {
      visual_odometer_->process(image_data, dims);
    }
    else
    {
      if(visual_odometer_->process(image_data, dims))
      {
        replace_ = false;
        Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.", 
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << camera_motion);

        btMatrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        btVector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }
      else
      {
        ROS_DEBUG("Call to VisualOdometryMono::process() failed. Assuming motion too small.");
        replace_ = true;
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }
    }
  }
};

} // end of namespace

// added by YL
bool covariance_flag = true;
void sensor_callback(const turtlebot_node::TurtlebotSensorState::ConstPtr& state){
	if(state->requested_left_velocity == 0 && state->requested_right_velocity == 0 && state->distance == 0)
		covariance_flag = true;
	else
		covariance_flag = false;
}
//end

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_odometer");
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("mono_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }
  std::string transport = argc > 1 ? argv[1] : "raw";
  std::cout << transport << std::endl;  
  viso2_ros::MonoOdometer odometer(transport);
  // added by YL
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<turtlebot_node::TurtlebotSensorState>("turtlebot_node/sensor_state", 1, sensor_callback);
  while(ros::ok()){
    ros::spinOnce();
    odometer.setCovariance(covariance_flag);
  }
  //end
  return 0;
}

