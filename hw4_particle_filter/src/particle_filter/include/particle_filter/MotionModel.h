#ifndef __MOTION_MODEL_H__
#define __MOTION_MODEL_H__

#include <geometry_msgs/Pose2D.h>
#include <particle_filter_msgs/particle.h>
#include <random>

struct State
{
	double x, y, theta;

	State();
	State(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {};
};

struct Odometry
{
	State o_prev, o_curr;
};


class MotionUpdater
{
public:

	double a1, a2, a3, a4; // error parameters for motion model

	MotionUpdater();
	MotionUpdater(double _a1, double _a2, double _a3, double _a4);

    void sample_motion_model_odometry(
        particle_filter_msgs::particle prev_state,
        particle_filter_msgs::particle &new_state,
        geometry_msgs::Pose2D odom_prev,
        geometry_msgs::Pose2D odom_cur);
	
	void sample_motion_model_odometry_v2(
        particle_filter_msgs::particle prev_state,
        particle_filter_msgs::particle &new_state,
        geometry_msgs::Pose2D odom_prev,
        geometry_msgs::Pose2D odom_cur);
	
protected:
	double sample(double a1, double odo1, double a2, double odo2); 
    std::random_device rd;
    std::default_random_engine generator;

};


#endif
