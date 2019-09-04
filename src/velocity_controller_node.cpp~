/* Headers in ROS */
#include <ros/ros.h>

/* Include this package */
#include <vel_controller/vel_controller.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vel_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  VelController controller(nh, pnh);
  controller.run();
  
  return 0;
}
