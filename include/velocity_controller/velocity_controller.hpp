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
  //Input
  double target_velocity_y_; /* Target velocity of Y-Axis m/s*/
  double target_velocity_yaw_; /* Target angular velocity of Yaw rad/s*/
  double current_velocity_y_; /* Current Velocity of Y-Axis m/s*/
  double current_velocity_yaw_; /* Current angular velocity of Yaw rad/s*/
  
  //Temporary Variables
  double velocity_difference_y_old; /*e_y*/
  double velocity_difference_yaw_old; /*e_yaw*/
  double thrust_left_old_; /*T_l*/
  double thrust_right_old_; /*T_r*/
  double base_thrust_; /*dT_b*/
  double rotation_thrust_; /*dT_t*/

  //Storage Variables used in Call Back Function
  double param_max_velocity_y_; /* Max velocity of Y-Axis */
  double param_max_velocity_yaw_; /* Max angular velocity of Yaw */
  double param_max_thrust_deviation_; /* Amount of deviation of changing output in each Step */
  double param_rotation_effect_;
  double param_p_gain_y_; /* Proportional Gain of Y-Axis*/
  double param_p_gain_yaw_; /* Proportional Gain of Yaw*/
  double param_i_gain_y_; /* Integral Gain of Y-Axis*/
  double param_i_gain_yaw_; /* Integral Gain of Yaw*/
  usv_control_msgs::AzimuthThrusterCatamaranDriveStamped thrust_cmd_;
  
  /*Parameters of this class*/
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
