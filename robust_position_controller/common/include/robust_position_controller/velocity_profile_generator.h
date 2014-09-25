/*
 * velocity_profile_generator.h
 * Padmaja Kulkarni
 * September 23, 2014
 */

#ifndef VELOCITY_PROFILE_GENERATOR_H_
#define VELOCITY_PROFILE_GENERATOR_H_

#include<vector>
#include<kdl/velocityprofile_trap.hpp>
#include<kdl/velocityprofile_rect.hpp>

class VelocityProfileGenerator
{
	public:
		VelocityProfileGenerator();
		~VelocityProfileGenerator();
		void setConstraints(std::vector<double> joint_limits_max,
							std::vector<double> joint_limits_min,
							double velocity_max_,
							double acceleration_max_,
							double set_point_threshold_
							);

		bool setGoalPositions(
							std::vector<double> start_positions,
							std::vector<double> goal_positions
							);

		bool checkGoalPositionConstraint(
					std::vector<double> current_positions
					);

		bool getRequiredVelocities(
							double time,
							std::vector<double> current_positions,
							std::vector<double> &goal_positions);

		bool getRequiredVelocitiesVP(
						   double sample_time,
						   std::vector<double> current_positions,
						   std::vector<double> &desired_velocities);

		bool getRequiredVelocitiesVPTrap(
						   double sample_time,
						   std::vector<double> current_positions,
						   std::vector<double> &desired_velocities);

	private:
		std::vector<double> joint_limits_max_;
		std::vector<double> joint_limits_min_;
		double velocity_max_;
		double velocity_min_;
		double acceleration_max_;
		double acceleration_min_;
		double set_point_threshold_;

		std::vector<double> start_positions_;
		std::vector<double> goal_positions_;
};
#endif