// Headers in this package
#include <velocity_controller/velocity_controller.hpp>
#include <boost/bind.hpp>

VelocityController::VelocityController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  int cnt = 0;
  
  /*Variables Initializing*/
  thrust_cmd.command.left_thrust_cmd = 0.0;
  thrust_cmd.command.right_thrust_cmd = 0.0;
  for(cnt=0; cnt<3; cnt++)
	{
	  target_velocity_[cnt] = 0.0;
	  current_velocity_[cnt] = 0.0;
	}
  base_thrust_ = 0.0;
  
  /* Create ROS Param */
  pnh_.param<std::string>("target_velocity", sub_topicname_target_velocity_, "/target_velocity");
  pnh_.param<std::string>("current_velocity", sub_topicname_current_velocity_, "/pose_to_twist/current_twist");
  pnh_.param<std::string>("thrust_command", pub_topicname_thrust_, "/control_command");
  pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
  
  /*Attach Subscriber callback*/
  sub_target_velocity_ = nh_.subscribe(sub_topicname_target_velocity_, 100, &VelocityController::subcb_veltarg, this);
  sub_current_velocity_ = nh_.subscribe(sub_topicname_current_velocity_, 100, &VelocityController::subcb_velcurr, this);
  
  /*Attach Publisher*/
  thrust_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(pub_topicname_thrust_, 100);
  
  /*Attach Reconfigure Server*/
  reconfcbptr_ = boost::bind(&VelocityController::reconf_cbfunc, this, _1, _2);
  reconfserv_.setCallback(reconfcbptr_);
}
  

VelocityController::~VelocityController()
{
  
}

void VelocityController::run()
{
  ros::Rate rate(100);
  
  while(ros::ok())
	{		
	  update_thrust();
	  publish_thrust();
	  ros::spinOnce();
	  rate.sleep();
	}
  
 }


void VelocityController::subcb_veltarg(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  mtx_.lock();
  target_velocity_[0] = msg->twist.linear.y;
  target_velocity_[1] = msg->twist.linear.x;
  target_velocity_[2] = msg->twist.angular.z;
  mtx_.unlock();
  
}

void VelocityController::subcb_velcurr(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  mtx_.lock();
  current_velocity_[0] = msg->twist.linear.y;
  current_velocity_[1] = msg->twist.linear.x;
  current_velocity_[2] = msg->twist.angular.z;
  mtx_.unlock();
}

void VelocityController::reconf_cbfunc(velocity_controller::ControlParamConfig &config, uint32_t level)
{
  mtx_.lock();
  param_max_velocity_[0] = config.VctrlMax_X;
  param_max_velocity_[1] = config.VctrlMax_Y;
  param_max_velocity_[2] = config.VctrlMax_Yaw;

  param_max_thrust_deviation_[0] = config.TdiffMax_X;
  param_max_thrust_deviation_[1] = config.TdiffMax_Y;
  param_max_thrust_deviation_[2] = config.TdiffMax_Yaw;

  param_p_gain_[0] = config.PGain_X;
  param_p_gain_[1] = config.PGain_Y;
  param_p_gain_[2] = config.PGain_Yaw;
  mtx_.unlock();
}

void VelocityController::update_thrust()
{
  int cnt = 0;
  double Vdiff[3] = {0.0, 0.0, 0.0}; /*u, v, r*/
  double Vdiff_idx[3] = {0.0, 0.0, 0.0};
  double T_Left = 0.0;
  double T_Right = 0.0;
  double T_Left_abs = 0.0;
  double T_Right_abs = 0.0;
  double Vctrl_max[3];
  double Tdiff_max[3];
  double Pgain[3];
  
  mtx_.lock();
  for(cnt=0; cnt<3; cnt++)
  {
	Vctrl_max[cnt] = param_max_velocity_[cnt];
	Tdiff_max[cnt] = param_max_thrust_deviation_[cnt];
	Pgain[cnt] = param_p_gain_[cnt];
  }
  mtx_.unlock();
  
  /* -1- Getting Current Velocity Status */
  mtx_.lock();
  for(cnt=0; cnt<3; cnt++)
  {
	Vdiff[cnt] = target_velocity_[cnt] - current_velocity_[cnt];
  }
  mtx_.unlock();
  ROS_INFO("Vel-DEV  X:%.3lf  Y:%.3lf  YAW:%.3lf", Vdiff[0], Vdiff[1], Vdiff[2]);
  
  /* -2- Normalize Vdiff */
  for(cnt=0; cnt<3; cnt++)
  {
	Vdiff_idx[cnt] = Vdiff[cnt] / Vctrl_max[cnt];
	ROS_INFO("Vel-DEVIDX  X:%.3lf", Vdiff_idx[cnt]);	
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
  ROS_INFO("Vel-DEV(STD,CAP)  X:%.6lf  Y:%.6lf  YAW:%.6lf", Vdiff_idx[0], Vdiff_idx[1], Vdiff_idx[2]);

  /* -3- Calculate Base Thrust */
  base_thrust_ =  base_thrust_ + Pgain[0] * Vdiff_idx[0];
  if(base_thrust_ > 1.0)
  {
	base_thrust_ = 1.0;
  }
  else if(base_thrust_ < -1.0)
  {
	base_thrust_ = -1.0;
  }
  else
  {
	/*DO NOTHING*/
  }
  ROS_INFO("Base Thrust: %.3lf", base_thrust_);
	
  /* -4- Calculate Differential Thrust */
  T_Left = base_thrust_ + Pgain[2] * (Vdiff_idx[2] / 2);
  T_Right = base_thrust_ - Pgain[2] * (Vdiff_idx[2] / 2);
  ROS_INFO("Thrust(RAW): [LEFT]%.3lf, [RIGHT]%.3lf", T_Left, T_Right);
  
  
  /* -5- Adjust to Maximum Thrust */
  T_Left_abs = std::abs(T_Left);
  T_Right_abs = std::abs(T_Right);
  
  if( T_Left_abs >1.0 || T_Right_abs > 1.0 )
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
  ROS_INFO("Thrust(FINAL): [LEFT]%.3lf, [RIGHT]%.3lf\n", T_Left, T_Right);
  
  mtx_.lock();
  thrust_cmd.command.left_thrust_cmd = T_Left;
  thrust_cmd.command.right_thrust_cmd = T_Right;
  mtx_.unlock();
  
}


void VelocityController::publish_thrust()
{
  mtx_.lock();
  thrust_cmd.header.frame_id = robot_frame_;
  thrust_cmd.header.stamp = ros::Time::now();
  thrust_pub_.publish(thrust_cmd);
  mtx_.unlock();
}
