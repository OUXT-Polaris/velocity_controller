cmake_minimum_required(VERSION 2.8.3)
project(velocity_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  usv_control_msgs
  dynamic_reconfigure
  )

###Dynamic Reconfigure###
generate_dynamic_reconfigure_options(cfg/velocity_controller_constants.cfg)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES velcontroller
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs usv_control_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(vel_controller_node src/vel_controller.cpp src/vel_controller_node.cpp)
target_link_libraries(vel_controller_node ${catkin_LIBRARIES})
add_dependencies(vel_controller_node usv_control_msgs_generate_messages_cpp velocity_controller_gencfg)
