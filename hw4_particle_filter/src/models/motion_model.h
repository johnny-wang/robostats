#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_

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


class motion_updater
{
public:

	double a1, a2, a3, a4; // error parameters for motion model

	motion_updater();
	motion_updater(double _a1, double _a2, double _a3, double _a4) : a1(_a1), a2(_a2), a3(_a3), a4(_a4) {};
	
	State sample_motion_model_odometry(State prev_state, Odometry odometer);
	State sample_motion_model_odometry_v2(State prev_state, Odometry odometer);
	
protected:
	double sample(double a1, double odo1, double a2, double odo2); 

};


#endif
