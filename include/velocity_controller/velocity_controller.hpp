#ifndef VELOCITY_CONTROLLER_H_INCLUDED
#define VELOCITY_CONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>
#include <dynamic_reconfigure/server.h>
#include <velocity_controller/ControlParamConfig.h>

/* Headers in STL */
#include <mutex>



class VelocityController
{
  
public:
  VelocityController(ros::NodeHandle nh, ros::NodeHandle pnh);/*Constructor*/
  ~VelocityController();	/*Destructor*/
  void run();	/*Start program*/
  
private:
  /*instances*/
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_target_velocity_;
  ros::Subscriber sub_current_velocity_;
  ros::Publisher thrust_pub_;
  dynamic_reconfigure::Server<velocity_controller::ControlParamConfig> reconfserv_;
  dynamic_reconfigure::Server<velocity_controller::ControlParamConfig>::CallbackType reconfcbptr_;
  std::mutex mtx_;
  
  
  /*Member Variables*/
  double target_velocity_y_; /* [u, v, r] m/s*/
  double target_velocity_yaw_; /* [u, v, r] m/s*/
  double current_velocity_y_; /* [u, v, r] m/s*/
  double current_velocity_yaw_; /* [u, v, r] m/s*/
  double velocity_difference_y_old;
  double velocity_difference_yaw_old;
  double thrust_left_old_;
  double thrust_right_old_;
  double base_thrust_;
  double rotation_thrust_;
  double param_max_velocity_y_; /* [u, v, r] m/s : Max Speeds of the ship*/
  double param_max_velocity_yaw_; /* [u, v, r] m/s : Max Speeds of the ship*/
  double param_max_thrust_deviation_; /* [u, v, r] % : Max gradient of Speeds*/
  double param_p_gain_y_; /* [u, v, r] m/s : Parameter of control gain (Proportional) */
  double param_p_gain_yaw_;
  double param_i_gain_y;
  double param_i_gain_yaw;
  usv_control_msgs::AzimuthThrusterCatamaranDriveStamped thrust_cmd_;
  
  /*Parameters*/
  std::string sub_topicname_target_velocity_;
  std::string sub_topicname_current_velocity_;
  std::string pub_topicname_thrust_;
  std::string robot_frame_;
  
  /*methods*/
  void subcb_veltarg(const geometry_msgs::TwistStamped::ConstPtr msg);
  void subcb_velcurr(const geometry_msgs::TwistStamped::ConstPtr msg);
  void reconf_cbfunc(velocity_controller::ControlParamConfig &config, uint32_t level);
  void update_thrust();
  void publish_thrust();  
};

#endif  /*VELOCITY_CONTROLLER_H_INCLUDED*/
