// Headers in this package
#include <ros/ros.h>
#include <velocity_controller/vel2force_pid_controller.hpp>

Vel2ForcePidController::Vel2ForcePidController(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  std::string targetvel_topic = "joy";
  std::string currentvel_topic = "joy";
  targetvel_sub_ = nh_.subscribe(targetvel_topic, 100, &Vel2ForcePidController::targetvel_sub_callback, this);
  currentvel_sub_ = nh_.subscribe(currentvel_topic, 100, &Vel2ForcePidController::currentvel_sub_callback, this);
  motor_port_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>("thrust_force", 100);

  pid_coef_p = {3.0, 3.0, 3.0};
  pid_coef_i = {2.0, 2.0, 2.0};
  pid_coef_d = {0.0, 0.0, 0.0};
}

Vel2ForcePidController::~Vel2ForcePidController():
{
  
}

Vel2ForcePidController::run()
{
  boost::thread thread_update(boost::bind(&Vel2ForcePidController::update_force_, this));
  boost::thread thread_publish(boost::bind(&Vel2ForcePidController::publish_force, this));
}

void Vel2ForcePidController::targetvel_sub_callback_(const std_msgs::Float32MultiArray msg)
{
  int i = 0;
  int size = msg.data.size();
  
  mtx_.lock();
  target_vel_.data.clear();
  for(i=0; i<size; i++)
  {
	target_vel_.data.push_back(msg.data[i]);
  }
  mtx_.unlock();
}

void Vel2ForcePidController::currentvel_sub_callback_(const std_msgs::Float32MultiArray msg)
{
  int i = 0;
  int size = msg.data.size();
  
  mtx_.lock();
  current_vel_.data.clear();
  for(i=0; i<size; i++)
  {
	current_vel_.data.push_back(msg.data[i]);
  }
  mtx_.unlock();
}
  
void Vel2ForcePidController::update_force()
{
  int i = 0;
  int size = 0;
  std_msgs::Float32MultiArray target, current, error;

  while(ros::ok())
	{
	  current.data.clear();
	  target.data.clear();
	  error.data.clear();

	  /*Get target velocity and current velocity*/
	  mtx_.lock();
	  size = current_vel_.data.size();;
	  for(i=0; i<size; i++)
	  {
		current.data.push_back(current_vel_);
	  }
	  
	  size = target_vel_.data.size();
	  for(i=0; i<size; i++)
	  {
		target.data.push_back(target_vel_);
	  }
	  mtx.unlock();

	  /*Error Calculation between target and current data*/
	  if(current.data.size() == target.data.size())
	  {
		for(i=0; i<size; i++)
		{
		  error.data.push_back( target.data[i] - current.data[i] );
		}

	  }
	  else
	  {
		/* DO NOTHING (ERROR)*/
	  }

	  /*Update Error Accumulations for "I" element*/
	  for(i=0; i<size; i++)
	  {
		if(
		   error_accumulation_[i] > -error_min_[i] &&
		   error_accumulation_[i] < error_min_[i]
		  )
		{
		  error_accumulation_[i] = 0.0;
		}
		else
		{
		  error_accumulation[i] += error.data[i];
		}

	  }

	  /*Calculate Force*/
	  mtx.lock();
	  for(i=0; i<size; i++)
	  {
		out_force_.data[i] = pid_coef_p_[i] * error.data[i] + pid_coef_i_[i] * error_accumulation_[i];
	  }
	  mtx.unlock();
	  

	}

}

void Vel2ForcePidController::publish_force_()
{
  ros::Rate rate(100);
  
  while(ros::ok())
  {
	mtx_.lock();
	target_force_pub_.publish(out_force_);
	mtx_.unlock();

  }
  rate.sleep();
}
