#ifndef VEL2FORCE_PID_CONTROLLER_H_INCLUDED
#define VEL2FORCE_PID_CONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

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
  /*instances*/
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber targetvel_sub_;
  ros::Subscriber currentvel_sub_;
  ros::Publisher target_force_pub;
  std::mutex mtx_;

  /*member variables*/
  std_msgs::Float32MultiArray out_force_, target_vel_, current_vel_; /* [F_x, F_y, N_z] */

  //PID
  std::vector<float> error_accumulation_; /*(u, v, r)*/
  std::vector<int> error_min_; /*(u, v, r)*/
  std::vector<float> pid_coef_p_; /*(u, v, r)*/
  std::vector<float> pid_coef_i_; /*(u, v, r)*/
  std::vector<float> pid_coef_d_; /*(u, v, r)*/

  /*methods*/
  void targetvel_sub_callback_(const std_msgg::Float32MultiArray msg);
  void currentvel_sub_callback_(const std_msgg::Float32MultiArray msg);
  void update_force_();
  void publish_force_();
  
};

#endif  /*VEL2FORCE_PID_CONTROLLER_H_INCLUDED*/
