cmake_minimum_required(VERSION 2.8.3)
project(robust_position_control_test_unit)

find_package(catkin REQUIRED COMPONENTS
  brics_actuator
  rospy
  std_msgs

)

catkin_package(

  CATKIN_DEPENDS brics_actuator rospy std_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robust_position_control_test_unit scripts/robust_position_control_test_unit.py)
