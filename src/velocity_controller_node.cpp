/* Headers in ROS */
#include <ros/ros.h>

/* Include this package */
#include <velocity_controller/velocity_controller.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "velocity_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  VelocityController controller(nh, pnh);
  controller.run();
  
  return 0;
}
