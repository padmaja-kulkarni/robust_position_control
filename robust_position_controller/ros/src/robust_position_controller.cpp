/*
 * robust_position_controller.cpp
 * Padmaja Kukarni
 * Date: September 23,2014
 */

#include<ros/ros.h>
#include<robust_position_controller/robust_position_controller.h>


RobustPositionController::RobustPositionController(const ros::NodeHandle &nh):
        nh_(nh), state_(INIT), is_position_cmd_recieved_(false),
        is_joint_feedback_recieved_(false)
{
    event_in_sub_ = nh_.subscribe("event_in", 10, &RobustPositionController::eventInCB, this);

    event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 10);

    joint_positions_sub_ = nh_.subscribe("component_input", 10, &RobustPositionController::jointPositionsCB, this);

    joints_feedback_sub_ = nh_.subscribe("/joint_states", 10, &RobustPositionController::jointsFeedbackCB, this);

    joint_velocities_pub_ = nh_.advertise<brics_actuator::JointVelocities>("component_output", 10);

    ROS_INFO("State:INIT");

    //vp_mode_sub_ = nh_.subscribe("vp_mode_type",10, &RobustPositionController::vpModeSelectionCB, this);

    initParameters();

    //you need to create an object of the velocity profile class
}


RobustPositionController::~RobustPositionController()
{

}


void RobustPositionController::initParameters()
{
    ros::param::param<std::string>("~vp_default_mode", default_mode_, "rect");
    ros::param::param<double>("~velocity_limit", velocity_limit_, 0.5);
    ros::param::param<double>("~acceleration_limit", acceleration_limit_, 0.0);
    ros::param::param<double>("~threshold_limit", threshold_limit_, 0.1);
}


void RobustPositionController::eventInCB(const std_msgs::String::ConstPtr &msg)
{
    event_in_ = msg->data;
}


void RobustPositionController::jointPositionsCB(const brics_actuator::JointPositions::ConstPtr &msg)
{
    joint_positions_ = *msg;
    is_position_cmd_recieved_ = true;
}


void RobustPositionController::jointsFeedbackCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    joints_feedback_ = *msg;
    is_joint_feedback_recieved_ = true;
}

void RobustPositionController::vpModeSelectionCB(const std_msgs::String::ConstPtr &msg)
{


}

void RobustPositionController::executeCycle()
{
    switch(state_) {
        case INIT:
            initState();
        break;

        case IDLE:
            idleState();
        break;

        case RUNNING:
            runningState();
        break;
    }
}


void RobustPositionController::initState()
{
    if (is_position_cmd_recieved_ && is_joint_feedback_recieved_) {
        ROS_INFO("State:IDLE");
        is_position_cmd_recieved_ = false;
        is_joint_feedback_recieved_ = false;
        state_ = IDLE;
        event_in_ = "";
    }

}


void RobustPositionController::idleState()
{
    if (event_in_ == "e_start") {
        ROS_INFO("State:RUNNING");
        state_ = RUNNING;
        event_in_ = "";
    }
}


void RobustPositionController::runningState()
{
    std_msgs::String event_out_msg;
    //set constraints....joint max positions,joint max velocities and accelerations
    getConstraintsParameters();

    getCurrentJointsState();

    /*
     * Communicating with velocity profiler class
     */

     velocity_profile_generator.setConstraints(
                                    joint_max_limits,
                                    joint_min_limits,
                                    velocity_limit_,
                                    acceleration_limit_,
                                    threshold_limit_
                                    );
   

    bool result = velocity_profile_generator.setGoalPositions(joint_feedbacks_, joint_values_);
    if (!result) {
        ROS_INFO_STREAM("Joint_position are out of bound");
        event_out_msg.data = "e_failed";
    } else {
        double start_time = ros::Time::now().toSec();
        double current_time = ros::Time::now().toSec();
        double time_stamp = 0.0;
        std::vector<double> desired_velocities;

        bool break_loop;
        ros::Rate loop_rate(5.0);
        while (ros::ok()) {
           time_stamp = ros::Time::now().toSec() - start_time;
           getCurrentJointsState();
           break_loop = velocity_profile_generator.getRequiredVelocitiesVPTrap(time_stamp, joint_feedbacks_, desired_velocities);
           publishVelocity(desired_velocities);
           if(break_loop)
                break;
           ros::spinOnce();
           loop_rate.sleep();
           current_time = ros::Time::now().toSec();
        }
        event_out_msg.data = "e_done";
    }

    event_out_pub_.publish(event_out_msg);  
    ROS_INFO("State:INIT");
    is_position_cmd_recieved_ = false;
    is_joint_feedback_recieved_ = false;
    state_ = INIT;
}


void RobustPositionController::publishVelocity(std::vector<double> desired_velocities)
{
    int joint_vel_size = desired_velocities.size();
    joint_velocities_.velocities.resize(joint_vel_size);

    for (int loop_index = 0; loop_index < joint_vel_size;loop_index++) {

        joint_velocities_.velocities[loop_index].joint_uri = joint_names_.at(loop_index);

        joint_velocities_.velocities[loop_index].unit = "s^-1 rad";

        joint_velocities_.velocities[loop_index].value = desired_velocities.at(loop_index);
    }
    joint_velocities_pub_.publish(joint_velocities_);
}

void RobustPositionController::getConstraintsParameters()
{
    /*
     * Reading joint names and commanded joint positions from input command
     * We are using joint names to read configuration parameters from parameter server.
     */
    //ROS_INFO_STREAM("Entering in getConstraintsParameters.");
    double number_of_joint_cmd = joint_positions_.positions.size();

    joint_names_.resize(number_of_joint_cmd);
    joint_values_.resize(number_of_joint_cmd);

    joint_max_limits.resize(number_of_joint_cmd);
    joint_min_limits.resize(number_of_joint_cmd);

    std::string custom_str1 = "_angle_max_limit";
    std::string custom_str2 = "_angle_min_limit";
    std::string private_tilda = "~";
    
    for (int loop_index = 0; loop_index < number_of_joint_cmd;loop_index++) {

        joint_names_.at(loop_index) =  joint_positions_.positions[loop_index].joint_uri;
        joint_values_.at(loop_index) =  joint_positions_.positions[loop_index].value;

        ros::param::param<double>(private_tilda + joint_names_.at(loop_index) + custom_str1, joint_max_limits.at(loop_index), 0.0);
        ros::param::param<double>(private_tilda + joint_names_.at(loop_index) + custom_str2, joint_min_limits.at(loop_index), 0.0);

        //ROS_INFO_STREAM("joint_max_limits: " << joint_max_limits.at(loop_index));
        //ROS_INFO_STREAM("joint_min_limits: " << joint_min_limits.at(loop_index));
    }
}


void RobustPositionController::getCurrentJointsState()
{
    //ROS_INFO_STREAM("Entering in getCurrentJointsState.");
    if (is_position_cmd_recieved_) {
        double number_of_joint_cmd = joint_positions_.positions.size();

        joint_feedbacks_.resize(number_of_joint_cmd);
        for (int loop_index = 0; loop_index < number_of_joint_cmd;loop_index++) {

            int pos = find(joints_feedback_.name.begin(), joints_feedback_.name.end(), joint_names_.at(loop_index)) - joints_feedback_.name.begin();
            
            joint_feedbacks_.at(loop_index) = joints_feedback_.position[pos];

            //ROS_INFO_STREAM("arm_joint_5 index is: " << pos);
        }
   }  
}