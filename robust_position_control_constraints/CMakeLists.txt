cmake_minimum_required(VERSION 2.8.3)
project(robust_position_control_constraints)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(
    catkin_INCLUDE_DIRSS
	)


add_executable(robust_position_control_constraints_node 
	src/robust_position_control_constraints_node.cpp)


target_link_libraries(robust_position_control_constraints_node
   ${catkin_LIBRARIES}
)