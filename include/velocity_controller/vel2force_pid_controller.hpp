#ifndef VEL2FORCE_PID_CONTROLLER_H_INCLUDED
#define VEL2FORCE_PID_CONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
#include <boost/thread.hpp>

class Vel2ForcePidController
{
  
public:
  Vel2ForcePidController(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Vel2ForcePidController();
  void run();
  
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber targetvel_sub_;
  ros::Subscriber currentvel_sub_;
  ros::Publisher target_force_pub;
  std::mutex mtx_;
  
  std_msgs::Float32 pub_data_port_, pub_data_stbd_;
  
  void targetvel_sub_callback_(const sensor_msgs::Joy::ConstPtr msg);
  void currentvel_sub_callback_(const sensor_msgs::Joy::ConstPtr msg);
  void update_force_();
  void publish_force_();
  
};

#endif  /*VEL2FORCE_PID_CONTROLLER_H_INCLUDED*/
