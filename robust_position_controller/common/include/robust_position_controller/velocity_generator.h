/*
 * velocity_generator.h
 * Padmaja Kulkarni
 * September 23, 2014
 */

#ifndef VELOCITY_GENERATOR_H_
#define VELOCITY_GENERATOR_H_

#include<vector>
#include<kdl/velocityprofile_trap.hpp>
#include<kdl/velocityprofile_rect.hpp>


class VelocityGenerator
{
	public:
		VelocityGenerator();
		~VelocityGenerator();

		void initlizeVelocityProfiler(std::string vp_mode, 
			double max_vel, double max_acc,double threshold);

		void setVPProfiler(double current_pos, double goal_pos);

		bool getVelocity(double sample_time,double current_pos, double &vel);

		bool checkConstraints(double current_pos);

	private:

		std::string vp_mode_;

		double vp_max_vel_;

		double vp_max_acc_;

		double threshold_;

		double current_pos_;

		double goal_pos_;
};
#endif