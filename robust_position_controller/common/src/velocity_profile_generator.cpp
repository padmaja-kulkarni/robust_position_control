/*
 * velocity_profile_generator.cpp
 * Padmaja Kulkarni
 * September 23, 2014
 */
#include<robust_position_controller/velocity_profile_generator.h>


VelocityProfileGenerator::VelocityProfileGenerator()
{
}


VelocityProfileGenerator::~VelocityProfileGenerator()
{
}


void VelocityProfileGenerator::setConstraints(
							std::vector<double> joint_limits_max,
							std::vector<double> joint_limits_min,
							double velocity_max,
							double acceleration_max,
							double set_point_threshold
							)
{
	joint_limits_max_ = joint_limits_max;
	joint_limits_min_ = joint_limits_min;
	velocity_max_ = velocity_max;
	velocity_min_ = -velocity_max;
	acceleration_max_ = acceleration_max;
	acceleration_min_ = -acceleration_max;
	set_point_threshold_ = set_point_threshold;
}

bool VelocityProfileGenerator::setGoalPositions(
					std::vector<double> start_positions,
					std::vector<double> goal_positions
					)
{
	start_positions_ = start_positions;
	goal_positions_ = goal_positions;

	for (int index = 0;index < goal_positions.size();index++) {
		bool comp_with_min_limit = (goal_positions_.at(index) > joint_limits_min_.at(index));
		bool comp_with_max_limit = (goal_positions_.at(index) < joint_limits_max_.at(index));

		if (!(comp_with_min_limit && comp_with_max_limit)) {
			return false;
		}
	}

	return true;
}

bool VelocityProfileGenerator::checkGoalPositionConstraint(
					std::vector<double> current_positions
					)
{
	for (int index = 0;index < current_positions.size();index++) {
		bool comp_with_min_limit = (current_positions.at(index) > (goal_positions_.at(index) - set_point_threshold_));
		bool comp_with_max_limit = (current_positions.at(index) < (goal_positions_.at(index) + set_point_threshold_));

		if ((comp_with_min_limit && comp_with_max_limit)) {
			return true;
		}
	}
	return false;
}

bool VelocityProfileGenerator::getRequiredVelocities(
						   double time,
						   std::vector<double> current_positions,
						   std::vector<double> &desired_velocities)
{
	double vel = velocity_max_;
	bool result = false;
		if(checkGoalPositionConstraint(current_positions)) {
			vel = 0.0;
			result = true;
		}

		desired_velocities.resize(current_positions.size());
		for (int index = 0;index < current_positions.size();index++) {
				desired_velocities.at(index) = vel;
			}

	return result;
}


bool VelocityProfileGenerator::getRequiredVelocitiesVP(
						   double sample_time,
						   std::vector<double> current_positions,
						   std::vector<double> &desired_velocities)
{
	desired_velocities.resize(current_positions.size());
	double vel = velocity_max_;
	bool result = false;
		//if(checkGoalPositionConstraint(current_positions)) {
		//	vel = 0.0;
		//	result = true;
		//}

		KDL::VelocityProfile_Rectangular vp_rect(velocity_max_);

		for (int index = 0;index < current_positions.size();index++) {
			vp_rect.SetProfile(start_positions_.at(index), goal_positions_.at(index));

			vel = vp_rect.Vel(sample_time);
			desired_velocities.at(index) = vel;
		}

		if(vel == 0.0)
			result = true;

	return result;
}

bool VelocityProfileGenerator::getRequiredVelocitiesVPTrap(
						   double sample_time,
						   std::vector<double> current_positions,
						   std::vector<double> &desired_velocities)
{
	desired_velocities.resize(current_positions.size());
	double vel = velocity_max_;
	bool result = false;
		if(checkGoalPositionConstraint(current_positions)) {
			result = true;
		}

		KDL::VelocityProfile_Trap vp_trap(velocity_max_, acceleration_max_);

		for (int index = 0;index < current_positions.size();index++) {
			vp_trap.SetProfile(start_positions_.at(index), goal_positions_.at(index));

			vel = vp_trap.Vel(sample_time);
			desired_velocities.at(index) = vel;
		}

	return result;
}