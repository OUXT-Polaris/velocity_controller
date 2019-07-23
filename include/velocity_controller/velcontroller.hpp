#ifndef VELCONTROLLER_H_INCLUDED
#define VELCONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
//#include <ros/common.h>　/*ROS_ASSERT()*/
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
#include <boost/thread.hpp>

class VelController
{
  
public:
  VelController(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~VelController();
  void run();
  
private:
  /*instances*/
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber targetvel_sub_;
  ros::Subscriber currentvel_sub_;
  ros::Publisher thrustout_pub_;
  std::mutex mtx_;

  /*member variables*/
  double target_vel_[3], current_vel[3]; /*u, v, r*/
  //geometry_msgs::Twist out_force_, /* [F_x, F_y, N_z] */

  //PID
  //std::vector<int> error_min_; /*(u, v, r)*/
  double pid_coef_p_[3]; /*(u, v, r)*/
  double pid_coef_i_[3]; /*(u, v, r)*/
  double pid_coef_d_[3]; /*(u, v, r)*/
  double error_accum_[3]; /*(U, v, r)*/
  double error_limit_[3];

  /*methods*/
  void targetvel_sub_callback_(const geometry_msgs::Twist msg);
  void currentvel_sub_callback_(const geometry_msgs::Twist msg);
  void update_();
  void publish_();
  
};

#endif  /*VELCONTROLLER_H_INCLUDED*/
