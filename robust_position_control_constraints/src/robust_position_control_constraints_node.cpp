#include <ros/ros.h>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "robust_position_controller");
	
	ros::NodeHandle nh("~");

	ROS_INFO("Node robust_position_controller started");


	std::vector<std::string> joint_names;//

	joint_names.resize(3);

	char *joint_name[3] = {"arm_joint_1", "arm_joint_3", "arm_joint_5"};

  	std::string custom_str = "_angle_max_limit";
  	std::string private_tilda = "/robust_position_controller/";

  	for (int i = 0; i < 3; i++) {
  		joint_names.at(i) = joint_name[i];
  		ROS_INFO_STREAM(joint_names.at(i));
  	}


  	double joints_max_limits[3];

  	for (int i = 0; i < joint_names.size(); i++) {
  		ros::param::param<double>(private_tilda+joint_names[i]+custom_str, joints_max_limits[i], 0.1);
  	}

  	for (int i = 0; i < joint_names.size(); i++) {
  		ROS_INFO_STREAM(private_tilda+joint_names[i]+custom_str << joints_max_limits[i]);
  	}

	ros::spin();

	return 0;
}