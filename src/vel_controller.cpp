// Headers in this package
#include <ros/ros.h>
#include <vel_controller/vel_controller.hpp>

VelController::VelController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh),
  Tbase_(0.0)
{
  /*Init Constants*/
  const float VelController::Vctrl_max[] = {10.0, 0.0, 5.0};
  const float VelController::Tdiff_max[] = {0.1, 0.0, 0.1};
  const float VelController::Pgain[] = {1.0, 1.0, 1.0};
  
  /* Read ROS Param */
  std::string topicname_sub_veltarg = "vel_targ";
  std::string topicname_sub_velcurr = "vel_curr";
  std::string topicname_pub_thrust_left = "thrust_left";
  std::string topicname_pub_thrust_right = "thrust_right";
  
  /*Attach Subscriber callback*/
  sub_veltarg_ = nh_.subscribe(topicname_veltarg, 100, &VelController::subcb_veltarg, this);
  sub_velcurr_ = nh_.subscribe(topicname_velcurr, 100, &VelController::subcb_velcurr, this);
  
  /*attach Publisher*/
  thrust_left_pub_ = pnh_.advertise<std_msgs::Float32>(topicname_pub_thrust_left, 100);
  thrust_right_pub_ = pnh_.advertise<std_msgs::Float32>(topicname_pub_thrust_right, 100);
}

VelController::~VelController():
{
  
}

VelController::run()
{
  ros::Rate rate(100);
  
  while(ros::ok())
  {		
	update_thrust();
	publish_thrust();
	rate.sleep();
  }
  
}




void VelController::subcb_veltarg(const geometry_msgs::Twist msg)
{
	mtx_.lock();
	veltarg_[0] = msg.linear.y;
	veltarg_[1] = msg.linear.x;
	veltarg_[2] = msg.angular.z;
	mtx_.unlock();
}

void VelController::subcb_velcurr(const geometry_msgs::Twist msg)
{
	mtx_.lock();
	velcurr_[0] = msg.linear.y;
	velcurr_[1] = msg.linear.x;
	velcurr_[2] = msg.angular.z;
	mtx_.unlock();
}

void VelController::update_thrust()
{
	int cnt = 0;
	float Vdiff[3] = {0.0, 0.0, 0.0}; /*u, v, r*/
	float Vdiff_idx[3] = {0.0, 0.0, 0.0};
	float T_Left = 0.0;
	float T_Right = 0.0;
	float T_Left_abs = 0.0;
	float T_Right_abs = 0.0;

	/* -1- Getting Current Velocity Status */
	mtx_.lock();
	for(cnt=0; cnt<3; cnt++)
	{
		Vdiff[cnt] = veltarg_[cnt] - velcurr_[cnt];
	}
	mtx_.unlock();

	/* -2- Normalize Vdiff */
	for(cnt=0; cnt<3; cnt++)
	{
		Vdiff_idx[cnt] = Vdiff_[cnt] / Vctrl_max[cnt];
		
		if( Vdiff_idx[cnt] > Tdiff_max[cnt] )
		{
			Vdiff_idx[cnt] = Tdiff_max[cnt];
		}
		else if( Vdiff_idx[cnt] < -Tdiff_max[cnt] )
		{
			Vdiff_idx[cnt] = -Tdiff_max[cnt];
		}
		else
		{
			/*** DO NOTHING ***/
		}
	}

	/* -3- Calculate Base Thrust */
	Tbase_ =  Tbase_ + Pgain[0] * Vdiff_idx[0];

	
	/* -4- Calculate Differential Thrust */
	T_Left = Tbase + Pgain[2] * (Vdiff_idx[2] / 2);
	T_Right = Tbase - Pgain[2] * (Vdiff_idx[2] / 2);

	
	/* -5- Adjust to Maximum Thrust */
	T_Left_abs = std::abs(T_Left);
	T_Right_abs = std::abs(T_Right);

	if( T_Left_abs * T_Right_abs > 1.0 )
	{
		if( T_Left_abs > T_Right_abs )
		{
			T_Right = T_Right / T_Left;
			T_Left = 1.0;
		}
		else
		{
			T_Left = T_Left / T_Right;
			T_Right = 1.0;
		}
	}

	mtx_.lock();
	thrust_left.data = T_Left;
	thrust_right.data = T_Right;
	mtx_unlock();
		
}


void VelController::publish_thrust_()
{
	mtx_.lock();
	thrust_left_pub_.publish(thrust_left);
	thrust_right_pub_.publish(thrust_right);	
	mtx_.unlock();
}