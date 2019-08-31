#ifndef VEL_CONTROLLER_H_INCLUDED
#define VEL_CONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>
#include <dynamic_reconfigure/server.h>
#include <velocity_controller/ControlConstConfig.h>

/* Headers in STL */
#include <mutex>



class VelController
{
  
public:
  VelController(ros::NodeHandle nh, ros::NodeHandle pnh);/*Constructor*/
  ~VelController();	/*Destructor*/
  void run();	/*Start program*/
  
private:
  /*instances*/
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_veltarg_;
  ros::Subscriber sub_velcurr_;
  ros::Publisher thrust_pub_;
  dynamic_reconfigure::Server<velocity_controller::ControlConstConfig> reconfserv_;
  dynamic_reconfigure::Server<velocity_controller::ControlConstConfig>::CallbackType reconfcb_;
  std::mutex mtx_;
  
  
  /*Member Variables*/
  double veltarg_[3]; /* [u, v, r] m/s*/
  double velcurr_[3]; /* [u, v, r] m/s*/
  double Tbase_;
  double Vctrl_max_[3]; /* [u, v, r] m/s : Max Speeds of the ship*/
  double Tdiff_max_[3]; /* [u, v, r] % : Max gradient of Speeds*/
  double Pgain_[3]; /* [u, v, r] m/s : Parameter of control gain (Proportional) */
  usv_control_msgs::AzimuthThrusterCatamaranDriveStamped thrust_cmd;
  
  /*Parameters*/
  std::string topicname_sub_veltarg;
  std::string topicname_sub_velcurr;
  std::string topicname_pub_thrust;
  
  /*methods*/
  void subcb_veltarg(const geometry_msgs::Twist::ConstPtr msg);
  void subcb_velcurr(const geometry_msgs::Twist::ConstPtr msg);
  void reconf_cbfunc(velocity_controller::ControlConstConfig &config, uint32_t level);
  void update_thrust();
  void publish_thrust();
  void indicate_cmdline();
  
};

#endif  /*VEL_CONTROLLER_H_INCLUDED*/
