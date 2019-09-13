ls
// Headers in this package
#include <velocity_controller/velocity_controller.hpp>
#include <boost/bind.hpp>

VelocityController::VelocityController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  int cnt = 0;
  
  /*Variables Initializing*/
  thrust_cmd_.command.left_thrust_cmd = 0.0;
  thrust_cmd_.command.right_thrust_cmd = 0.0;
  for(cnt=0; cnt<3; cnt++)
	{
	  target_velocity_[cnt] = 0.0;
	  current_velocity_[cnt] = 0.0;
	}
  velocity_difference_y_old = 0.0;
  velocity_difference_yaw_old = 0.0;
  thrust_left_old_ = 0.0;
  thrust_right_old_ = 0.0;
  base_thrust_ = 0.0;
  rotation_thrust_ = 0.0;
  
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
  target_velocity_y_ = msg->twist.linear.y;
  target_velocity_yaw_ = msg->twist.angular.z;
  mtx_.unlock();
}


void VelocityController::subcb_velcurr(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  mtx_.lock();
  current_velocity_y_ = msg->twist.linear.y;
  current_velocity_yaw_ = msg->twist.angular.z;
  mtx_.unlock();
}


void VelocityController::reconf_cbfunc(velocity_controller::ControlParamConfig &config, uint32_t level)
{
  mtx_.lock();
  param_max_velocity_y_ = config.VctrlMax_Y;
  param_max_velocity_yaw_ = config.VctrlMax_Yaw;
  param_max_thrust_deviation_ = config.TdiffMax;
  param_p_gain_y_ = config.PGain_X;
  param_p_gain_yaw_ = config.PGain_Yaw;
  param_i_gain_y_ = config.IGain_X;
  param_i_gain_yaw_ = config.IGain_Yaw;
  mtx_.unlock();
}


void VelocityController::update_thrust()
{
  /*Parameters*/
  double max_velocity_y = 0.0;
  double max_velocity_yaw = 0.0;
  double max_thrust_deviation = 0.0;
  double p_gain_y = 0.0;
  double p_gain_yaw = 0.0;
  double i_gain_y = 0.0;
  double i_gain_yaw = 0.0;
  
  /*Variables*/
  double velocity_difference_y = 0.0;
  double velocity_difference_yaw = 0.0;
  double max_thrust = 0.0;
  double T_Left = 0.0;
  double T_Right = 0.0;
  double T_Left_abs = 0.0;
  double T_Right_abs = 0.0;


  
  /* -0- Getting Parameters */
  mtx_.lock();
  max_velocity_y = param_max_velocity_y_;
  max_velocity_yaw = param_max_velocity_yaw_;
  max_thrust_deviation = param_max_thrust_deviation;
  p_gain_y = param_p_gain_y_;
  p_gain_yaw = param_p_gain_yaw_;
  i_gain_y = param_i_gain_y_;
  i_gain_yaw = param_i_gain_yaw_;

  
  /* -1- Calculate difference between target and current velocity */
  velocity_difference_y = target_velocity_y_ - current_velocity_y_;
  velocity_difference_yaw = target_velocity_yaw_ - current_velocity_yaw_;
  mtx_.unlock();
  ROS_INFO("Vel-Diff  Y:%.3lf  YAW:%.3lf", velocity_difference_y, velocity_difference_yaw);

  
  /* -2- Calculate Base Thrust and Rotation Thrust */
  base_thrust_ = p_gain_y * (velocity_difference_y - velocity_difference_y_old) + i_gain_y * velocity_difference_y;
  rotation_thrust_ = p_gain_yaw * (velocity_difference_yaw - velocity_difference_yaw_old) + i_gain_yaw * velocity_difference_yaw;
  ROS_INFO("Fwd:%.3lf, Rot:%.3lf", base_thrust_, rotation_thrust_);

  
  /* -3- Calculate Left and Right Thrust */
  T_Left = base_thrust_ - rotation_thrust;
  T_Right = base_thrust_ + rotation_thrust;
  ROS_INFO("Thrust(RAW): [LEFT]%.3lf, [RIGHT]%.3lf", T_Left, T_Right);
  
  /* -4- Normalize Thrust */
  max_thrust =
	p_gain_y * max_velocity_y + i_gain_yaw * max_velocity_y
	+p_gain_yaw * max_velocity_yaw + i_gain_yaw * max_velocity_yaw;
  T_Left /= max_thrust;
  T_Right /= max_thrust;

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
  ROS_INFO("Thrust(Norm): [LEFT]%.3lf, [RIGHT]%.3lf", T_Left, T_Right);
  

  /* -5- Limit Thrust change */
  //Left
  if( T_Left - thrust_left_old > max_thrust_deviation;)
  {
	T_Left = thrust_left_old + max_thrust_deviation;
  }
  else
  {
	T_Left = thrust_left_old + T_Left;
  }

  //Right
  if( T_Right - thrust_right_old > max_thrust_deviation;)
  {
	T_Right = thrust_right_old + max_thrust_deviation;
  }
  else
  {
	T_Right = thrust_right_old + T_Right;
  }
  ROS_INFO("Thrust(FINAL): [LEFT]%.3lf, [RIGHT]%.3lf\n", T_Left, T_Right);


  /* -6- Send Thruster Topic */
  mtx_lock();
  thrust_cmd_.command.left_thrust_cmd = T_Left;
  thrust_cmd_.command.right_thrust_cmd = T_Right;
  mtx_.unlock();


  /* -7- Refresh Old Values */
  velocity_difference_y_old = velocity_difference_y; 
  velocity_difference_yaw_old = velocity_difference_yaw;
  thrust_left_old_ = T_Left;
  thrust_right_old_ = T_Right;
  
}


void VelocityController::publish_thrust()
{
  mtx_.lock();
  thrust_cmd_.header.frame_id = robot_frame_;
  thrust_cmd_.header.stamp = ros::Time::now();
  thrust_pub_.publish(thrust_cmd_);
  mtx_.unlock();
}
