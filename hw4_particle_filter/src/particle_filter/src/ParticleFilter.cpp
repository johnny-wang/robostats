
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
    _particles_pub = nh.advertise<visualization_msgs::Marker>("particles", 1, true);
    _lines_pub = nh.advertise<visualization_msgs::Marker>("lines", 1, true);
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

void ParticleFilter::createMarker(visualization_msgs::Marker &marker, int type) {
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;

    // points
    if (type == 1) {
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.ns = "particle_pts";

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    } else {
    // lines
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.ns = "orientation_lines";

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }

    marker.color.a = 1.0;     // alpha = 1 or else won't show
    marker.lifetime = ros::Duration();
}

void ParticleFilter::initializeParticles()
{
    int col_min = 0, row_min = 0;
    int col_max = _map.getColSize();
    int row_max = _map.getRowSize();
    // create RNG, uniform & normal for each of the x,y,theta
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> rng1(col_min, col_max);
    std::uniform_real_distribution<double> rng2(row_min, row_max);
    std::uniform_real_distribution<double> rng3(-M_PI, M_PI);

    // Resample until we get particle in free space
    int id_cnt = 0;
    for (int i = 0; i < _num_particles; ) {
        particle_filter_msgs::particle p;
        p.pose.x = rng1(generator);
        p.pose.y = rng2(generator);
        p.pose.theta = rng3(generator);

        if (_map.getMapValue(p.pose.y, p.pose.x) == FREE) {
            p.particle_id = id_cnt;
            p.weight = 1.0 / _num_particles;
            _particles_list.push_back(p);
            //std::cout << "Adding particle: " << std::endl;
            //printParticle(p);
            i++;
            id_cnt++;
        }
    }

    std::cout << "Num particles: " << _particles_list.size() << std::endl;
    visualizeParticles();
}

void ParticleFilter::printParticle(particle_filter_msgs::particle p) {
    printf("pid: %d x: %f y: %f th: %f wt: %f\n", p.particle_id, p.pose.x, p.pose.y, 
           p.pose.theta, p.weight);
}

void ParticleFilter::visualizeParticles() {

    //_points_marker.action = 3;
    //_lines_marker.action = 3;
    //_particles_pub.publish(_points_marker);
    //_lines_pub.publish(_lines_marker);

    //_points_marker.points.clear();
    //_lines_marker.points.clear();

    createMarker(_points_marker, 1);
    createMarker(_lines_marker, 2);
   
    std::vector<geometry_msgs::Point> p_vec; 
    std::vector<geometry_msgs::Point> l_vec;
    for (int i = 0; i < _particles_list.size(); i++) {
        geometry_msgs::Point p;
        // Set position and orientation of particle
        p.x = _particles_list[i].pose.x;
        p.y = _particles_list[i].pose.y;
        p.z = 0;
        p_vec.push_back(p);

        // Draw line (2 points) to show orientation
        geometry_msgs::Point p1;
        p1.x = 0.1*cos(_particles_list[i].pose.theta) + p.x;
        p1.y = 0.1*sin(_particles_list[i].pose.theta) + p.y;
        p1.z = 0;
        l_vec.push_back(p);
        l_vec.push_back(p1);

        //p_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 
        //    _particles_list[i].pose.theta);
        //
        //double roll, pitch, yaw;
        //tf::Quaternion bt_q;
        //quaternionMsgToTF(p_marker.pose.orientation, bt_q);
        //tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw); 
        //printf("%f %f %f\n", p_marker.pose.position.x, p_marker.pose.position.y, yaw);

        //printf("%f %f \n", p.x, p.y);
        //printf("%f %f %f %f\n", p.x, p.y, p1.x, p1.y);
    }
    _points_marker.points = p_vec;
    _lines_marker.points = l_vec;

    //printf("p size: %lu\n", p_vec.size());
    //printf("l size: %lu\n", l_vec.size());

    _particles_pub.publish(_points_marker);
    _lines_pub.publish(_lines_marker);
}
