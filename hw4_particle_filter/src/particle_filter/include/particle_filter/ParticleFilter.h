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
// Package stuff
#include "particle_filter/DistanceMap.h"
#include "particle_filter/DataLoader.h"
#include <particle_filter_msgs/laser_odom.h>
#include <particle_filter_msgs/particle.h>
// C++ stuff
#include <random>
#include <chrono>
#include <vector>
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

    void laser_odom_callback(const particle_filter_msgs::laser_odom::ConstPtr &msg);
    void odom_callback(const geometry_msgs::Pose2D::ConstPtr &msg);
    bool run();
    void closeFile();

private:
    void createMarker(visualization_msgs::Marker &marker, int type);
    bool initialize(std::string occ_map, std::string dist_map, int num_dgrees, float ray_step);
    void initializeParticles();
    void printParticle(particle_filter_msgs::particle p);
    void visualizeParticles();

    int _num_particles;

    DistanceMap _map;
    DataLoader _data;
    std::vector<particle_filter_msgs::particle> _particles_list;

    bool _initialized_odom;
    geometry_msgs::Pose2D _last_odom;

    // Subscribe to odom topics
    ros::NodeHandle _nh;
    ros::Subscriber _laser_odom;
    ros::Subscriber _odom;
    ros::Publisher _particles_pub;
    ros::Publisher _lines_pub;
    visualization_msgs::Marker _points_marker;
    visualization_msgs::Marker _lines_marker;
};

#endif
