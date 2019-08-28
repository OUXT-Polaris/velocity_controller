// Headers in this package
#include <ros/ros.h>
#include <vel_controller/vel_controller.hpp>

VelController::VelController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh),
  Tbase_(0.0)
{
  int cnt = 0;
  
  /*Init Variables*/
  thrust_cmd.command.left_thrust_cmd = 0.0;
  thrust_cmd.command.right_thrust_cmd = 0.0;
  for(cnt=0; cnt<3; cnt++)
  {
	veltarg_[cnt] = 0.0;
	velcurr_[cnt] = 0.0;
  }
  
  /*Init Constants*/
  const double Vctrl_max[] = {10.0, 0.0, 5.0};
  const double Tdiff_max[] = {0.1, 0.0, 0.1};
  const double Pgain[] = {1.0, 1.0, 1.0};
  
  /* Create ROS Param */
  pnh_.param<std::string>("target_velocity", topicname_sub_veltarg, "/target_velocity");
  pnh_.param<std::string>("current_velocity", topicname_sub_velcurr, "/pose_to_twist/current_twist");
  pnh_.param<std::string>("thrust_command", topicname_pub_thrust, "/control_command");
  
  /*Attach Subscriber callback*/
  sub_veltarg_ = nh_.subscribe(topicname_sub_veltarg, 100, &VelController::subcb_veltarg, this);
  sub_velcurr_ = nh_.subscribe(topicname_sub_velcurr, 100, &VelController::subcb_velcurr, this);
  
  /*attach Publisher*/
  thrust_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(topicname_pub_thrust, 100);
}

VelController::~VelController()
{

}

void VelController::run()
{
  ros::Rate rate(100);

  while(ros::ok())
  {		
	update_thrust();
	publish_thrust();
	//indicate_cmdline();
	rate.sleep();
  }
  
}


void VelController::subcb_veltarg(const geometry_msgs::Twist::ConstPtr msg)
{
	mtx_.lock();
	veltarg_[0] = msg->linear.y;
	veltarg_[1] = msg->linear.x;
	veltarg_[2] = msg->angular.z;
	ROS_INFO("%lf", veltarg_[0]);
	mtx_.unlock();

}

void VelController::subcb_velcurr(const geometry_msgs::Twist::ConstPtr msg)
{
	mtx_.lock();
	velcurr_[0] = msg->linear.y;
	velcurr_[1] = msg->linear.x;
	velcurr_[2] = msg->angular.z;
	mtx_.unlock();
}

void VelController::update_thrust()
{
	int cnt = 0;
	double Vdiff[3] = {0.0, 0.0, 0.0}; /*u, v, r*/
	double Vdiff_idx[3] = {0.0, 0.0, 0.0};
	double T_Left = 0.0;
	double T_Right = 0.0;
	double T_Left_abs = 0.0;
	double T_Right_abs = 0.0;

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
		Vdiff_idx[cnt] = Vdiff[cnt] / Vctrl_max[cnt];

		
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
		//ROS_INFO("%lf", Vdiff_idx[cnt]);
	}

	/* -3- Calculate Base Thrust */
	Tbase_ =  Tbase_ + Pgain[0] * Vdiff_idx[0];

	
	/* -4- Calculate Differential Thrust */
	T_Left = Tbase_ + Pgain[2] * (Vdiff_idx[2] / 2);
	T_Right = Tbase_ - Pgain[2] * (Vdiff_idx[2] / 2);

	
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
	thrust_cmd.command.left_thrust_cmd = T_Left;
	thrust_cmd.command.right_thrust_cmd = T_Right;
	mtx_.unlock();
		
}


void VelController::publish_thrust()
{
	mtx_.lock();
	thrust_pub_.publish(thrust_cmd);
	mtx_.unlock();
}


void VelController::indicate_cmdline()
{
  printf("[Target]:\tX:%.1fm/s\t\tY:%.1fm/s\t\tR:%.1frad/s\n", veltarg_[0], veltarg_[1], veltarg_[2]);
  printf("[Current]:\tX:%.1fm/s\t\tY:%.1fm/s\t\tR:%.1frad/s\n", velcurr_[0], velcurr_[1], velcurr_[2]);
  printf("[Control]:\tPORT:%.1f%%\t\tSTBD:%.1f%%\n", thrust_cmd.command.left_thrust_cmd*100, thrust_cmd.command.right_thrust_cmd*100);
  printf("\n");
}
