
#include "particle_filter/ParticleFilter.h"

ParticleFilter::ParticleFilter(
        ros::NodeHandle nh, 
        std::string laser_topic, 
        std::string odom_topic,
        int msg_queue_size,
        int num_particles,
        std::string occ_map_file,
        std::string dist_map_file,
        int num_degrees,
        float ray_step_size
    ) : // initialization list
    _laser_odom(nh.subscribe(laser_topic, msg_queue_size, &ParticleFilter::laser_odom_callback, this)),
    _odom(nh.subscribe(odom_topic, msg_queue_size, &ParticleFilter::laser_odom_callback, this)),
    _num_particles(num_particles)
{
    initialize(occ_map_file, dist_map_file, num_degrees, ray_step_size);
}

bool ParticleFilter::initialize(
    std::string occ_map, 
    std::string dist_map, 
    int num_degrees, 
    float ray_step_size) 
{
    // Load Map (occupancy and distance maps)
std::cout << occ_map << " " << dist_map << std::endl;
    _map.loadMaps(occ_map, dist_map, num_degrees, ray_step_size);

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
