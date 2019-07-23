// Headers in this package
#include <ros/ros.h>
#include <velocity_controller/velcontroller.hpp>

VelController::VelController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  std::string targetvel_topic = "";
  std::string currentvel_topic = "";
  targetvel_sub_ = nh_.subscribe(targetvel_topic, 100, &VelController::targetvel_sub_callback, this);
  currentvel_sub_ = nh_.subscribe(currentvel_topic, 100, &VelController::currentvel_sub_callback, this);
  thrustout_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>("thrust_force", 100);

  pid_coef_p = {3.0, 3.0, 3.0};
  pid_coef_i = {2.0, 2.0, 2.0};
  pid_coef_d = {0.0, 0.0, 0.0};

  error_accum_ = {1.0, 1.0, 1.0};
  error_limit_ = {3.0, 3.0, 3.0};
}

VelController::~VelController():
{
  
}

VelController::run()
{
  boost::thread thread_update(boost::bind(&VelController::update_, this));
  boost::thread thread_publish(boost::bind(&VelController::publish_, this));
}

void VelController::targetvel_sub_callback_(const geometry_msgs::Twist msg)
{
  mtx_.lock();
  target_vel_[0] = 0;
  target_vel_[1] = 0;
  target_vel_[2] = 0;

  target_vel_[0] = msg.linear.x;
  target_vel_[1] = msg.linear.y;
  target_vel_[2] = msg.angular.z;
  mtx_.unlock();
}

void VelController::currentvel_sub_callback_(const geometry_msgs::Twist msg)
{
  mtx_.lock();
  current_vel_[0] = 0;
  current_vel_[1] = 0;
  current_vel_[2] = 0;

  current_vel_[0] = msg.linear.x;
  current_vel_[1] = msg.linear.y;
  current_vel_[2] = msg.angular.z;
  mtx_.unlock();
}

void Vel2ForcePidController::update_()
{
  /*General Local Variables*/
  const unsigned int demension = 3;
  int cnt = 0;
  
  /*Temporary Variables*/
  double target[demension], current[demension], error[demension];
  double force_target[demension];

  while(ros::ok())
	{
	  
	  /*Initialize*/
	  for(cnt=0; cnt<demension; cnt++)
	  {
		target[cnt] = 0;
		current[cnt] = 0;
		error[cnt] = 0;
		force_target[cnt] = 0;
	  }


	  /*Get target and current values*/
	  mtx_.lock();
	  for(cnt=0; cnt<demension; cnt++)
	  {
		target[cnt] = target_vel_[cnt];
		current[cnt] = current_vel_[cnt];
	  }
	  mtx.unlock();

	  /*Calculate Error*/
	  for(cnt=0; cnt<demension; cnt++)
	  {
		error[cnt] = target[cnt] - current[cnt];
	  }
	  
	  /*Update Error Accumulations for "I" element*/
	  for(cnt=0; cnt<demension; cnt++)
	  {
		if(error_limit >= 0)
		{
		
		  if(
			 error_accum_[cnt] + error[cnt] > -error_limit_[cnt] &&
			 error_accum_[cnt] + error[cnt] < error_limit_[cnt]
			)
		  {
			error_accumu_[cnt] += error[cnt];
		  }
		  else
		  {
			/*DO NOTHING  (Anti Windup)*/ 
		  }
		  
		}
		else
		{
		  
		  if(
			 error_accum_[cnt] + error[cnt] > error_limit_[cnt] &&
			 error_accum_[cnt] + error[cnt] < -error_limit_[cnt]
			)
		  {
			error_accumu_[cnt] += error[cnt];
		  }
		  else
		  {
			/*DO NOTHING  (Anti Windup)*/ 
		  }
		}

	  }

	  /*PID Core : calculating out target force*/
	  for(cnt=0; cnt<demension; cnt++)
	  {
		force_target[cnt] = pid_coef_p_[cnt] * error[cnt] + pid_coef_i_[cnt] * error_accum_[cnt];
	  }

	  /*Calculate thrust*/
	  

	}

  

}

void VelController::publish_()
{
  ros::Rate rate(100);
  
  while(ros::ok())
  {
	mtx_.lock();
	thrustout_pub_.publish(out_force_);
	mtx_.unlock();

  }
  rate.sleep();
}
