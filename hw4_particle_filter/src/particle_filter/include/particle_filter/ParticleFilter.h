#ifndef __PARTICLE_FILTER_H
#define __PARTICLE_FILTER_H

// ROS stuff
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
// Package stuff
#include "particle_filter/DistanceMap.h"
#include <particle_filter_msgs/laser_odom.h>
// C++ stuff
#include <vector>
// Boost stuff
#include <boost/uuid/uuid.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

struct Particle
{
    Particle(double x, double y, double th) {
        static int pid = 0;
        _pose.x = x;
        _pose.y = y;
        _pose.theta = th;

        _particle_id = pid++;
    }

    geometry_msgs::Pose2D _pose;
    int _particle_id;
    float _weight;

    inline void print() {
        printf("pid: %d x: %f y: %f th: %f\n", _particle_id, _pose.x, _pose.y, _pose.theta);
    }
};

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle nh, 
                   std::string laser_topic, 
                   std::string odom_topic, 
                   int msg_queue_size,
                   int num_particles,
                   std::string occ_file,
                   std::string dist_file,
                   int num_dgrees,
                   float ray_step
                  );

    void laser_odom_callback(const particle_filter_msgs::laser_odom::ConstPtr &msg);
    void odom_callback(const geometry_msgs::Pose2D::ConstPtr &msg);
    void run();

private:
    bool initialize(std::string occ_map, std::string dist_map, int num_dgrees, float ray_step);
    void initializeParticles();

    int _num_particles;

    DistanceMap _map;
    std::vector<Particle> _particles_list;

    // Random sampling
    boost::mt19937 rng1, rng2, rng3;

    // Subscribe to odom topics
    ros::NodeHandle _nh;
    ros::Subscriber _laser_odom;
    ros::Subscriber _odom;
    ros::Publisher _particles_pub;
};

#endif
