#include "particle_filter/MotionModel.h"

#include <math.h>
#include <random>
#include <opencv2/opencv.hpp>

MotionUpdater::MotionUpdater(double _a1, double _a2, double _a3, double _a4) 
    : a1(_a1), a2(_a2), a3(_a3), a4(_a4) 
{
    generator = std::default_random_engine(rd());
}

void MotionUpdater::sample_motion_model_odometry(
    particle_filter_msgs::particle prev_state,
    particle_filter_msgs::particle &new_state,
    geometry_msgs::Pose2D odom_prev,
    geometry_msgs::Pose2D odom_cur)
{
    // prev_state (particle) in {w} : R_w1, t_w1{w}
    // odometer in {o} : R_o1, t_o1{o}, R_o2, t_o2{o} 
    // updated particle : t_w2{w} = R_w1*R_1o*t_o2{o} + R_w1*t_1o{1} + t_w1{w}
    // === R_w1 * R_o1_T * t_o2{o} - R_w1 * R_o1_T * t_o1{o} + t_w1{w}

    // Build Transformation matrix of previous odometry
    double _R_o1[4] = { cos(odom_prev.theta), -sin(odom_prev.theta), 
                        sin(odom_prev.theta), cos(odom_prev.theta) };
    cv::Mat R_o1 = cv::Mat(2,2, CV_64F, _R_o1);
    double _t_o1_o[2] = { odom_prev.x, odom_prev.y };
    cv::Mat t_o1_o = cv::Mat(2,1, CV_64F, _t_o1_o);

    // Build Transformation matrix of current odometry
    double _R_o2[4] = { cos(odom_cur.theta), -sin(odom_cur.theta), 
                        sin(odom_cur.theta), cos(odom_cur.theta) };
    cv::Mat R_o2 = cv::Mat(2,2, CV_64F, _R_o2);
    double _t_o2_o[2] = { odom_cur.x, odom_cur.y };
    cv::Mat t_o2_o = cv::Mat(2,1, CV_64F, _t_o2_o);

    // Build Transformation matrix of Particle
    double _R_w1[4] = { cos(prev_state.pose.theta), -sin(prev_state.pose.theta), 
                        sin(prev_state.pose.theta), cos(prev_state.pose.theta) };
    cv::Mat R_w1 = cv::Mat(2,2, CV_64F, _R_w1);
    double _t_w1_w[2] = { prev_state.pose.x, prev_state.pose.y };
    cv::Mat t_w1_w = cv::Mat(2,1, CV_64F, _t_w1_w);

    // Calculate new Transformation of Particle in world frame
    cv::Mat R_w2 = R_w1 * R_o1.t() * R_o2;
    cv::Mat t_w2_w = R_w1*R_o1.t()*t_o2_o - R_w1*R_o1.t()*t_o1_o + t_w1_w;

    double d_rot2 = odom_cur.theta - odom_prev.theta;

    odom_prev.x = prev_state.pose.x;
    odom_prev.y = prev_state.pose.y;
    odom_prev.theta = prev_state.pose.theta;
    odom_cur.x = t_w2_w.at<double>(0,0);
    odom_cur.y = t_w2_w.at<double>(1,0);
    odom_cur.theta = acos(R_w2.at<double>(0,0));

    double d_rot1 = atan2(odom_cur.y-odom_prev.y, odom_cur.x-odom_prev.x) - odom_prev.theta;
    double d_trans = sqrt((odom_cur.y-odom_prev.y)*(odom_cur.y-odom_prev.y)
                    + (odom_cur.x-odom_prev.x)*(odom_cur.x-odom_prev.x));
    d_rot2 -= d_rot1;

    double d_rot1_noise = d_rot1 - sample(a1, abs(d_rot1), a2, d_trans);
    double d_trans_noise = d_trans - sample(a3, d_trans, a4, abs(d_rot1)+abs(d_rot2));
    double d_rot2_noise = d_rot2 - sample(a1, abs(d_rot2), a2, d_trans);

    new_state.particle_id = prev_state.particle_id;
    new_state.weight = prev_state.weight;
    new_state.pose.x = prev_state.pose.x + d_trans_noise*cos(prev_state.pose.theta + d_rot1_noise);
    new_state.pose.y = prev_state.pose.y + d_trans_noise*sin(prev_state.pose.theta + d_rot1_noise);
    new_state.pose.theta = prev_state.pose.theta + d_rot1_noise + d_rot2_noise;
}

void MotionUpdater::sample_motion_model_odometry_v2(
    particle_filter_msgs::particle prev_state, 
    particle_filter_msgs::particle &new_state, 
    geometry_msgs::Pose2D odom_prev,
    geometry_msgs::Pose2D odom_cur) 
{
	// prev_state (particle) in {w} : R_w1, t_w1{w}
	// odometer in {o} : R_o1, t_o1{o}, R_o2, t_o2{o} 
	// updated particle : t_w2{w} = R_w1*R_1o*t_o2{o} + R_w1*t_1o{1} + t_w1{w}
	// === R_w1 * R_o1_T * t_o2{o} - R_w1 * R_o1_T * t_o1{o} + t_w1{w}

	double _cos = cos(prev_state.pose.theta-odom_prev.theta);
	double _sin = sin(prev_state.pose.theta-odom_prev.theta);

	double _R_wo[4] = { _cos, -_sin, _sin, _cos };
	cv::Mat R_wo = cv::Mat(2,2, CV_64F, _R_wo);

	double _t_12_o[2] = { odom_cur.x-odom_prev.x, odom_cur.y-odom_prev.y };
	cv::Mat t_12_o = cv::Mat(2,1, CV_64F, _t_12_o);
	double _t_w1_w[2] = { prev_state.pose.x, prev_state.pose.y }; 
	cv::Mat t_w1_w = cv::Mat(2,1, CV_64F, _t_w1_w);

	cv::Mat t_w2_w = R_wo*t_12_o + t_w1_w;

	double d_rot1 = atan2( t_w2_w.at<double>(1,0)-prev_state.pose.y, 
                           t_w2_w.at<double>(0,0)-prev_state.pose.x ) - prev_state.pose.theta;
	double d_trans = sqrt( (t_w2_w.at<double>(1,0)-prev_state.pose.y) 
                            * (t_w2_w.at<double>(1,0)-prev_state.pose.y) 
                            + (t_w2_w.at<double>(0,0)-prev_state.pose.x)
                            * (t_w2_w.at<double>(0,0)-prev_state.pose.x) );
	double d_rot2 = odom_cur.theta - odom_prev.theta - d_rot1; 

	double d_rot1_noise = d_rot1 - sample(a1, abs(d_rot1), a2, d_trans);
	double d_trans_noise = d_trans - sample(a3, d_trans, a4, abs(d_rot1)+abs(d_rot2));
	double d_rot2_noise = d_rot2 - sample(a1, abs(d_rot2), a2, d_trans);

    new_state.particle_id = prev_state.particle_id;
    new_state.weight = prev_state.weight;
    new_state.pose.x = prev_state.pose.x + d_trans_noise*cos(prev_state.pose.theta + d_rot1_noise);
    new_state.pose.y = prev_state.pose.y + d_trans_noise*sin(prev_state.pose.theta + d_rot1_noise);
    new_state.pose.theta = prev_state.pose.theta + d_rot1_noise + d_rot2_noise;
}

double MotionUpdater::sample(double a1, double odo1, double a2, double odo2)
{
	//std::default_random_engine generator;
//	std::normal_distribution<double> distribution(0, a1*abs(odo1)+a2*abs(odo2));
//	printf("=== %lf %lf %lf %lf \n", a1, odo1, a2, odo2);
	std::normal_distribution<double> distribution(0, a1*(odo1)+a2*(odo2));

	return distribution(generator);
}
