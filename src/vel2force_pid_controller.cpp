// Headers in this package
#include <ros/ros.h>
#include <velocity_controller/vel2force_pid_controller.hpp>

Vel2ForcePidController::Vel2ForcePidController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  std::string joy_topic = "joy";
  joy_sub_ = nh_.subscribe(joy_topic,100,&VrxJoystickOperator::joyCallback,this);
  motor_port_pub_ = nh_.advertise<std_msgs::Float32>("left_thrust_cmd", 100);
  motor_stbd_pub_ = nh_.advertise<std_msgs::Float32>("right_thrust_cmd", 100);
}

Vel2ForcePidController::~Vel2ForcePidController():
{
  
}

