#ifndef VEL_CONTROLLER_H_INCLUDED
#define VEL_CONTROLLER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

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
	ros::Publisher thrust_left_pub_;
	ros::Publisher thrust_right_pub_;
	std::mutex mtx_;
	
	/*Member Variables*/
	float veltarg_[3]; /* [u, v, r] m/s*/
	float velcurr_[3]; /* [u, v, r] m/s*/
	float Tbase_;
	std_msgs::Float32 thrust_left;
	std_msgs::Float32 thrust_right;
	

	/*Parameters*/
    float Vctrl_max[]; /* [u, v, r] m/s : Max Speeds of the ship*/
	float Tdiff_max[]; /* [u, v, r] % : Max gradient of Speeds*/
	float Pgain[]; /* [u, v, r] m/s : Parameter of control gain (Proportional) */
	

	/*methods*/
	void subcb_veltarg(const geometry_msgs::Twist msg);
	void subcb_velcurr(const geometry_msgs::Twist msg);
	void update_thrust();
	void publish_thrust();
  
};

#endif  /*VEL_CONTROLLER_H_INCLUDED*/
