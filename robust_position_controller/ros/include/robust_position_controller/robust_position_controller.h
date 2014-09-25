/*
 * robust_position_controller.h
 * Padmaja Kukarni
 * Date: September 23,2014
 */

#ifndef ROBUST_POSITION_CONTROLLER_H_
#define ROBUST_POSITION_CONTROLLER_H_
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<brics_actuator/JointPositions.h>
#include<brics_actuator/JointVelocities.h>
#include<sensor_msgs/JointState.h>
#include<robust_position_controller/velocity_profile_generator.h>
#include<vector>
#include<string>


class RobustPositionController
{
    public:
        RobustPositionController(const ros::NodeHandle &nh);
        ~RobustPositionController();
        void eventInCB(const std_msgs::String::ConstPtr &msg);
        void jointPositionsCB(const brics_actuator::JointPositions::ConstPtr &msg);
        void jointsFeedbackCB(const sensor_msgs::JointState::ConstPtr &msg);
        //to be used 
        void vpModeSelectionCB(const std_msgs::String::ConstPtr &msg);

        void initParameters();
        void executeCycle();
        void initState();
        void idleState();
        void runningState();

        void getConstraintsParameters();
        void getCurrentJointsState();
        void publishVelocity(std::vector<double> desired_velocities);

    private:
        enum States{
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle nh_;

        bool is_position_cmd_recieved_;
        bool is_joint_feedback_recieved_;

        ros::Subscriber event_in_sub_;
        ros::Subscriber joint_positions_sub_;
        ros::Subscriber joints_feedback_sub_;
        ros::Publisher joint_velocities_pub_;
        ros::Publisher event_out_pub_;
        //ros::Subscriber vp_mode_sub_;

        std::string event_in_;
        brics_actuator::JointPositions joint_positions_;
        brics_actuator::JointVelocities joint_velocities_;
        sensor_msgs::JointState joints_feedback_;

        States state_;

        //Constraint parameters
        std::string default_mode_;
        std::vector<std::string> joint_names_;
        std::vector<double> joint_values_;
        std::vector<double> joint_feedbacks_;

        std::vector<double> joint_max_limits;
        std::vector<double> joint_min_limits;
        double velocity_limit_;
        double threshold_limit_;
        double acceleration_limit_;

        VelocityProfileGenerator velocity_profile_generator;
};
#endif