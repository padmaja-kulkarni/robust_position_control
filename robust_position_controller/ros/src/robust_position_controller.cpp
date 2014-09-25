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
    event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 10);

    joint_velocities_pub_ = nh_.advertise<brics_actuator::JointVelocities>("component_output", 10);

    event_in_sub_ = nh_.subscribe("event_in", 10, &RobustPositionController::eventInCB, this);

    joint_positions_sub_ = nh_.subscribe("component_input", 10, &RobustPositionController::jointPositionsCB, this);

    joints_feedback_sub_ = nh_.subscribe("joints_feedback", 10, &RobustPositionController::jointsFeedbackCB, this);

    ROS_INFO("State:INIT");

    vp_mode_sub_ = nh_.subscribe("vp_mode_type",10, &RobustPositionController::vpModeSelectionCB, this);

    initParameters();

    velocity_generator.initlizeVelocityProfiler(default_mode_, vp_max_vel_,
                                                vp_max_acc_, set_point_threshold_);
    //you need to create an object of the velocity profile class
}


RobustPositionController::~RobustPositionController()
{

}


void RobustPositionController::initParameters()
{
    ros::param::param<std::string>("~default_vp_mode", default_mode_, "rect");
    ros::param::param<double>("~default_vp_max_vel", vp_max_vel_, 0.5);
    ros::param::param<double>("~default_vp_max_vel", vp_max_acc_, 0.1);
    ros::param::param<double>("~set_point_threshold", set_point_threshold_, 0.01);
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
    ROS_INFO("State:INIT");

    //*call computation class. Computation class should call constraints class.

    //velocity_profile.set(current_pos, goal_pos):

    std_msgs::String event_out_msg;
    event_out_msg.data = "e_done";

    event_out_pub_.publish(event_out_msg);
    state_ = INIT;
}
