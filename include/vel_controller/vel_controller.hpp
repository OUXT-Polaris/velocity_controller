#ifndef VEL_CONTROLLER_H_INCLUDED
#define VEL_CONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
/* #include <boost/thread.hpp> */

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
	std::mutex mtx_;
	
	/*Member Variables*/
    float veltarg_[3]; /* [u, v, r] m/s*/
    float velcurr_[3]; /* [u, v, r] m/s*/
    float Tbase_;
    usv_control_msgs::AzimuthThrusterCatamaranDriveStamped thrust_cmd;

	/*Parameters*/
    std::string topicname_sub_veltarg;
    std::string topicname_sub_velcurr;
    std::string topicname_pub_thrust;
    float Vctrl_max[]; /* [u, v, r] m/s : Max Speeds of the ship*/
	float Tdiff_max[]; /* [u, v, r] % : Max gradient of Speeds*/
	float Pgain[]; /* [u, v, r] m/s : Parameter of control gain (Proportional) */

	

	/*methods*/
	void subcb_veltarg(const geometry_msgs::Twist msg);
	void subcb_velcurr(const geometry_msgs::Twist msg);
	void update_thrust();
	void publish_thrust();
    void indicate_cmdline();
  
};

#endif  /*VEL_CONTROLLER_H_INCLUDED*/
