
#include "particle_filter/ParticleFilter.h"

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

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
    _nh(nh),
    _laser_odom(nh.subscribe(laser_topic, msg_queue_size, &ParticleFilter::laser_odom_callback, this)),
    _odom(nh.subscribe(odom_topic, msg_queue_size, &ParticleFilter::laser_odom_callback, this)),
    _num_particles(num_particles)
{
    //nh.
    initialize(occ_map_file, dist_map_file, num_degrees, ray_step_size);
}

bool ParticleFilter::initialize(
    std::string occ_map, 
    std::string dist_map, 
    int num_degrees, 
    float ray_step_size) 
{
    // Load Map (occupancy and distance maps)
    _map.loadMaps(_nh, occ_map, dist_map, num_degrees, ray_step_size);

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
    int col_min = 0, row_min = 0;
    int col_max = _map.getColSize();
    int row_max = _map.getRowSize();
    // create RNG, uniform & normal for each of the x,y,theta
    boost::uniform_real<double> uniform_x(col_min, col_max);
    boost::uniform_real<double> uniform_y(row_min, row_max);
    boost::uniform_real<double> uniform_t(-M_PI, M_PI);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double> > unif_x(rng1, uniform_x);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double> > unif_y(rng2, uniform_y);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double> > unif_t(rng3, uniform_t);

    // Resample until we get particle in free space
    for (int i = 0; i < _num_particles; ) {
        Particle p(unif_x(), unif_y(), unif_t());

        if (_map.getMapValue(p._pose.y, p._pose.x) == FREE) {
            p._weight = 1.0 / _num_particles;
            _particles_list.push_back(p);
            std::cout << "Adding particle: " << std::endl;
            p.print();
            i++;
        }
    }

    std::cout << "Num particles: " << _particles_list.size() << std::endl;
}
