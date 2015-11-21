
#include "particle_filter/ParticleFilter.h"

ParticleFilter::ParticleFilter(
        ros::NodeHandle nh, 
        std::string laser_topic, 
        std::string odom_topic,
        int msg_queue_size,
        int num_particles,
        std::string map_file
    ) : // initialization list
    _laser_odom(nh.subscribe(laser_topic, msg_queue_size, &ParticleFilter::laser_odom_callback, this)),
    _odom(nh.subscribe(odom_topic, msg_queue_size, &ParticleFilter::laser_odom_callback, this)),
    _num_particles(num_particles),
    _map_file(map_file)
{
    initialize();
}

bool ParticleFilter::initialize() 
{
    // Load Map

    // create RNG, uniform & normal

    // initialize particles (uniformly)
    initializeParticles();
}

void ParticleFilter::laser_odom_callback(const particle_filter_msgs::laser_odom::ConstPtr &msg)
{
    // Update particles with sensor model
}

void ParticleFilter::odom_callback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    // Update particles with motion model
}

void ParticleFilter::run()
{

}

/***************************************************************************/

void ParticleFilter::initializeParticles()
{

}
