#include "motion_model.h"

#include <math.h>
#include <random>
#include <opencv2\opencv.hpp>


State motion_updater::sample_motion_model_odometry(State prev_state, Odometry odometer) 
{
	// prev_state (particle) in {w} : R_w1, t_w1{w}
	// odometer in {o} : R_o1, t_o1{o}, R_o2, t_o2{o} 
	// updated particle : t_w2{w} = R_w1*R_1o*t_o2{o} + R_w1*t_1o{1} + t_w1{w}
	// === R_w1 * R_o1_T * t_o2{o} - R_w1 * R_o1_T * t_o1{o} + t_w1{w}

	double _R_o1[4] = { cos(odometer.o_prev.theta), -sin(odometer.o_prev.theta), sin(odometer.o_prev.theta), cos(odometer.o_prev.theta) };
	cv::Mat R_o1 = cv::Mat(2,2, CV_64F, _R_o1);
	double _t_o1_o[2] = { odometer.o_prev.x, odometer.o_prev.y };
	cv::Mat t_o1_o = cv::Mat(2,1, CV_64F, _t_o1_o);
	
	double _R_o2[4] = { cos(odometer.o_curr.theta), -sin(odometer.o_curr.theta), sin(odometer.o_curr.theta), cos(odometer.o_curr.theta) };
	cv::Mat R_o2 = cv::Mat(2,2, CV_64F, _R_o2);
	double _t_o2_o[2] = { odometer.o_curr.x, odometer.o_curr.y };
	cv::Mat t_o2_o = cv::Mat(2,1, CV_64F, _t_o2_o);

	double _R_w1[4] = { cos(prev_state.theta), -sin(prev_state.theta), sin(prev_state.theta), cos(prev_state.theta) };
	cv::Mat R_w1 = cv::Mat(2,2, CV_64F, _R_w1);
	double _t_w1_w[2] = { prev_state.x, prev_state.y }; 
	cv::Mat t_w1_w = cv::Mat(2,1, CV_64F, _t_w1_w);

	cv::Mat R_w2 = R_w1 * R_o1.t() * R_o2;
	cv::Mat t_w2_w = R_w1*R_o1.t()*t_o2_o - R_w1*R_o1.t()*t_o1_o + t_w1_w;

	odometer.o_prev.x = prev_state.x;
	odometer.o_prev.y = prev_state.y;
	odometer.o_prev.theta = prev_state.theta;
	odometer.o_curr.x = t_w2_w.at<double>(0,0);
	odometer.o_curr.y = t_w2_w.at<double>(1,0);
	odometer.o_curr.theta = acos(R_w2.at<double>(0,0));

	double d_rot1 = atan2(odometer.o_curr.y-odometer.o_prev.y, odometer.o_curr.x-odometer.o_prev.x) - odometer.o_prev.theta;
	double d_trans = sqrt((odometer.o_curr.y-odometer.o_prev.y)*(odometer.o_curr.y-odometer.o_prev.y)
					- (odometer.o_curr.x-odometer.o_prev.x)*(odometer.o_curr.x-odometer.o_prev.x));
	double d_rot2 = odometer.o_curr.theta - odometer.o_prev.theta - d_rot1; 

//	// use below if {o} == {w}
// 	double d_rot1 = atan2(odometer.o_curr.y-odometer.o_prev.y, odometer.o_curr.x-odometer.o_prev.x) - odometer.o_prev.theta;
// 	double d_trans = sqrt((odometer.o_curr.y-odometer.o_prev.y)*(odometer.o_curr.y-odometer.o_prev.y)
// 					- (odometer.o_curr.x-odometer.o_prev.x)*(odometer.o_curr.x-odometer.o_prev.x));
// 	double d_rot2 = odometer.o_curr.theta - odometer.o_prev.theta - d_rot1;

	double d_rot1_noise = d_rot1 - sample(a1, d_rot1, a2, d_trans);
	double d_trans_noise = d_trans - sample(a3, d_trans, a4, d_rot1+d_rot2);
	double d_rot2_noise = d_rot2 - sample(a1, d_rot2, a2, d_trans);

	return State(prev_state.x + d_trans_noise*cos(prev_state.theta + d_rot1_noise),
				prev_state.y + d_trans_noise*sin(prev_state.theta + d_rot1_noise),
				prev_state.theta + d_rot1_noise + d_rot2_noise);
}

State motion_updater::sample_motion_model_odometry_v2(State prev_state, Odometry odometer) 
{
	// prev_state (particle) in {w} : R_w1, t_w1{w}
	// odometer in {o} : R_o1, t_o1{o}, R_o2, t_o2{o} 
	// updated particle : t_w2{w} = R_w1*R_1o*t_o2{o} + R_w1*t_1o{1} + t_w1{w}
	// === R_w1 * R_o1_T * t_o2{o} - R_w1 * R_o1_T * t_o1{o} + t_w1{w}

	double _cos = cos(prev_state.theta-odometer.o_prev.theta);
	double _sin = sin(prev_state.theta-odometer.o_prev.theta);

	double _R_wo[4] = { _cos, -_sin, _sin, _cos };
	cv::Mat R_wo = cv::Mat(2,2, CV_64F, _R_wo);

	double _t_12_o[2] = { odometer.o_curr.x-odometer.o_prev.x, odometer.o_curr.y-odometer.o_prev.y };
	cv::Mat t_12_o = cv::Mat(2,1, CV_64F, _t_12_o);
	double _t_w1_w[2] = { prev_state.x, prev_state.y }; 
	cv::Mat t_w1_w = cv::Mat(2,1, CV_64F, _t_w1_w);

	cv::Mat t_w2_w = R_wo.t()*t_12_o + t_w1_w;

	double d_rot1 = atan2(t_w2_w.at<double>(1,0)-prev_state.y, t_w2_w.at<double>(0,0)-prev_state.x) - prev_state.theta;
	double d_trans = sqrt((t_w2_w.at<double>(1,0)-prev_state.y)*(t_w2_w.at<double>(1,0)-prev_state.y)
		- (t_w2_w.at<double>(0,0)-prev_state.x)*(t_w2_w.at<double>(0,0)-prev_state.x));
	double d_rot2 = prev_state.theta - odometer.o_prev.theta + odometer.o_curr.theta - odometer.o_prev.theta - d_rot1; 

	double d_rot1_noise = d_rot1 - sample(a1, d_rot1, a2, d_trans);
	double d_trans_noise = d_trans - sample(a3, d_trans, a4, d_rot1+d_rot2);
	double d_rot2_noise = d_rot2 - sample(a1, d_rot2, a2, d_trans);

	return State(prev_state.x + d_trans_noise*cos(prev_state.theta + d_rot1_noise),
		prev_state.y + d_trans_noise*sin(prev_state.theta + d_rot1_noise),
		prev_state.theta + d_rot1_noise + d_rot2_noise);
}

double motion_updater::sample(double a1, double odo1, double a2, double odo2)
{
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0, a1*abs(odo1)+a2*abs(odo2));

	return distribution(generator);
}
