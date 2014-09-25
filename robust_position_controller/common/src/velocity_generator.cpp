
/*
 * velocity_generator.cpp
 * Padmaja Kulkarni
 * September 23, 2014
 */
#include<robust_position_controller/velocity_generator.h>


VelocityGenerator::VelocityGenerator()
{
}


VelocityGenerator::~VelocityGenerator()
{
}

void VelocityGenerator::initlizeVelocityProfiler(std::string vp_mode, 
			double max_vel, double max_acc,double threshold)
{
	vp_mode_ = vp_mode;
	vp_max_vel_ = max_vel;
	vp_max_acc_ = max_acc;
	threshold_ = threshold;
}

void VelocityGenerator::setVPProfiler(double current_pos, double goal_pos)
{
	current_pos_ = current_pos;
	goal_pos_ = goal_pos;
}

bool VelocityGenerator::getVelocity(double sample_time, double current_pos, double &vel)
{	
	if(checkConstraints(current_pos))
		vel = 0.0;
		return false;

	if (vp_mode_ == "trap") {
		KDL::VelocityProfile_Trap vp_trap(vp_max_vel_, vp_max_acc_);

		vp_trap.SetProfile(current_pos_, goal_pos_);

		vel = vp_trap.Vel(sample_time);
	}
	else if(vp_mode_ == "rect") {
		KDL::VelocityProfile_Rectangular vp_rect(vp_max_vel_);
	}
}


bool VelocityGenerator::checkConstraints(double current_pos)
{
	if((current_pos > (goal_pos_ - threshold_)) && (current_pos < (goal_pos_ + threshold_)))
		return true;
	else
		return false;
}