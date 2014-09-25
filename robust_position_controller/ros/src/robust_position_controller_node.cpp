#include<ros/ros.h>
#include<robust_position_controller/robust_position_controller.h>

int main(int argc, char *argv[])

{
	ros::init(argc, argv, "robust_position_controller");
	
	ros::NodeHandle nh("~");

	double frequency;

	ros::param::param<double>("~loop_rate", frequency, 5.0);

	ros::Rate loop_rate(frequency);

	ROS_INFO("Node robust_position_controller started");

	RobustPositionController robust_position_control(nh);


	while(ros::ok()) {
		robust_position_control.executeCycle();
		ros::spinOnce();
		loop_rate.sleep();
	}

		return 0;

}

