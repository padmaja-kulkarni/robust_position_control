cmake_minimum_required(VERSION 2.8.3)
project(robust_position_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  brics_actuator
  sensor_msgs
)

find_package(orocos_kdl)


catkin_package(
  CATKIN_DEPENDS 
  	geometry_msgs 
  	roscpp 
  	std_msgs
  	brics_actuator
  	sensor_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}

)

add_executable(robust_position_controller_node 
 	ros/src/robust_position_controller_node.cpp
 	ros/src/robust_position_controller.cpp
  common/src/velocity_profile_generator.cpp
)


target_link_libraries(robust_position_controller_node
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)
