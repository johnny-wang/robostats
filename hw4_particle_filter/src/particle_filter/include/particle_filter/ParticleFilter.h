#ifndef __PARTICLE_FILTER_H
#define __PARTICLE_FILTER_H

// ROS stuff
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <random_numbers/random_numbers.h>
#include <random>
// Package stuff
#include "particle_filter/DistanceMap.h"
#include "particle_filter/DataLoader.h"
#include "particle_filter/MotionModel.h"
#include <particle_filter_msgs/laser_odom.h>
#include <particle_filter_msgs/particle.h>
// C++ stuff
#include <random>
#include <chrono>
#include <vector>
//Eigen
#include <Eigen/Geometry>
// Boost stuff
//#include <boost/random/uniform_real.hpp>
//#include <boost/random/normal_distribution.hpp>
//#include <boost/random/variate_generator.hpp>
//#include <boost/random/mersenne_twister.hpp>

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle nh, 
                   std::string laser_topic, 
                   std::string odom_topic, 
                   int msg_queue_size,
                   int num_particles,
                   std::string data_file,
                   std::string occ_file,
                   std::string dist_file,
                   int num_dgrees,
                   float ray_step
                  );

    bool run();
    void closeFile();

private:
    void createMarker(visualization_msgs::Marker &marker, int type);
    bool initialize(std::string occ_map, std::string dist_map, int num_dgrees, float ray_step);
    void initializeParticles();
    Eigen::Matrix3f poseToMatrix(geometry_msgs::Pose2D pose);
    Eigen::Matrix3f poseToMatrix(particle_filter_msgs::particle p);
    geometry_msgs::Pose2D matrixToPose(Eigen::Matrix3f m);
    void printParticle(particle_filter_msgs::particle p);
    void runSensorModel(particle_filter_msgs::laser_odom laser_data);
    void runMotionModel(geometry_msgs::Pose2D odom_data);
    void visualizeParticles();
    void visualizeLaser(geometry_msgs::Pose2D pose, sensor_msgs::LaserScan laser, float ray_angle);
    
    //sensor model prob functions
    double prob_hit(double z_true, double z);
    double prob_short(double z_true, double z);
    double prob_max(double z);
	double prob_rand(double z);
    double sensorSigma;//std dev for sensor model
	double z_max; //max lidar range
	double lambda_short; //measurement model intrinsic
    float weight_max;
	float weight_rand;
	float weight_hit;
	float weight_short;
	
	void resampleParticles(std::vector<float> probability_list);
	std::default_random_engine generator;
    float placement_stdDev; // cm from placement for 1 std dev
    float rotation_stdDev;

    int _num_particles;

    DistanceMap _map;
    DataLoader _data;
    std::vector<particle_filter_msgs::particle> _particles_list;

    bool _initialized_odom;
    geometry_msgs::Pose2D _last_odom;

    // Subscribe to odom topics
    ros::NodeHandle _nh;
    ros::Publisher _particles_pub;
    ros::Publisher _lines_pub;
    ros::Publisher _laser_pub;
    visualization_msgs::Marker _points_marker;
    visualization_msgs::Marker _lines_marker;
    visualization_msgs::Marker _laser_marker;
};

#endif
