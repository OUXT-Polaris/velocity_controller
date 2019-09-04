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
generate_dynamic_reconfigure_options(cfg/velocity_controller_configure.cfg)

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

add_executable(velocity_controller_node src/velocity_controller.cpp src/velocity_controller_node.cpp)
target_link_libraries(velocity_controller_node ${catkin_LIBRARIES})
add_dependencies(velocity_controller_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  velocity_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch cfg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
