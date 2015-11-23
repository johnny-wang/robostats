
#include "particle_filter/ParticleFilter.h"

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

//#define DEBUG_INIT
//#define DEBUG_DATA
#define DEBUG_MOTION
using namespace std; // for debugging for now

ParticleFilter::ParticleFilter(
        ros::NodeHandle nh, 
        std::string laser_topic, 
        std::string odom_topic,
        int msg_queue_size,
        int num_particles,
        std::string data_file,
        std::string occ_map_file,
        std::string dist_map_file,
        int num_degrees,
        float ray_step_size
    ) : // initialization list
    _nh(nh),
    _num_particles(num_particles),
    _particles_pub(nh.advertise<visualization_msgs::Marker>("particles", 1, true)),
    _lines_pub(nh.advertise<visualization_msgs::Marker>("lines", 1, true)),
    _data(data_file)
{
    initialize(occ_map_file, dist_map_file, num_degrees, ray_step_size);

    _initialized_odom = false;
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

/*
 * Return 'true' if everything is good and we should keep running.
 * Return 'false' once we've read all the lines.
 */
bool ParticleFilter::run()
{
    static int laser_cnt = 0;
    static int all_cnt = 0;
    static int odom_cnt = 0;
#ifdef DEBUG_DATA
    cout << "Running" << endl;
#endif

    OdomType data = _data.parseNextData();

    switch(data) {
    case NONE:
    {
#ifdef DEBUG_DATA
        cout << "No more data" << endl;
#endif
        return false;
        break;
    }
    case LASER:
    {
#ifdef DEBUG_DATA
        cout << "Laser " << laser_cnt << " all " << all_cnt << endl;
        laser_cnt++;
        all_cnt++;
#endif
        particle_filter_msgs::laser_odom laser_data = _data.getLaserData();
        // Run sensor model
        break;
    }
    case ODOM:
    {
#ifdef DEBUG_DATA
        cout << "Odom " << odom_cnt << " all " << all_cnt << endl;
        odom_cnt++;
        all_cnt++;
#endif
        geometry_msgs::Pose2D odom_data = _data.getOdomData();
        // Run motion model
        runMotionModel(odom_data);
        break;
    }
    default:
    {
        cout << "Default case in parsing Odom data" << endl;
        cout << "THIS SHOULD NEVER HAPPEN" << endl;
        break;
    }
    }
    return true;
}

void ParticleFilter::closeFile()
{
    _data.closeFile();
}

/***************************************************************************/

void ParticleFilter::createMarker(visualization_msgs::Marker &marker, int type) {
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;

    int res = _map.getResolution();

    // points
    if (type == 1) {
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.ns = "particle_pts";

        marker.scale.x = 0.5 * res;
        marker.scale.y = 0.5 * res;
        marker.scale.z = 0.5 * res;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    } else {
    // lines to show orientation
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.ns = "orientation_lines";

        marker.scale.x = 0.05 * res;
        marker.scale.y = 0.05 * res;
        marker.scale.z = 0.05 * res;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }

    marker.color.a = 1.0;     // alpha = 1 or else won't show
    marker.lifetime = ros::Duration();
}

void ParticleFilter::initializeParticles()
{
    int y_min = 0, x_min = 0;
    int y_max = _map.getYSize();
    int x_max = _map.getXSize();
    // create RNG, uniform & normal for each of the x,y,theta
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    //std::uniform_real_distribution<double> rng1(y_min*res, y_max*res);
    //std::uniform_real_distribution<double> rng2(x_min*res, x_max*res);
    std::uniform_real_distribution<double> rng1(y_min, y_max);
    std::uniform_real_distribution<double> rng2(x_min, x_max);
    std::uniform_real_distribution<double> rng3(-M_PI, M_PI);

#ifdef DEBUG_INIT
    std::cout << "Initializing particles" << std::endl;
#endif

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
#ifdef DEBUG_INIT
            std::cout << "Adding particle: " << std::endl;
            printParticle(p);
#endif
            i++;
            id_cnt++;
        }
    }

    std::cout << "Num particles: " << _particles_list.size() << std::endl;
    visualizeParticles();
}

Eigen::Matrix3f ParticleFilter::poseToMatrix(geometry_msgs::Pose2D pose) {
    return Eigen::Matrix3f();
}

void ParticleFilter::printParticle(particle_filter_msgs::particle p) {
    printf("pid: %d x: %f y: %f th: %f wt: %f\n", p.particle_id, p.pose.x, p.pose.y, 
           p.pose.theta, p.weight);
}

// Update particles with motion model
void ParticleFilter::runMotionModel(geometry_msgs::Pose2D odom_data) {
    if (!_initialized_odom) {
        _last_odom = odom_data;
        _initialized_odom = true;
#ifdef DEBUG_MOTION
        printf("last: %f %f %f\n", _last_odom.x, _last_odom.y, _last_odom.theta);
#endif
        return;
    }
    geometry_msgs::Pose2D delta;
    delta.x = _last_odom.x - odom_data.x;
    delta.y = _last_odom.y - odom_data.y;
    delta.theta = _last_odom.theta - odom_data.theta;
#ifdef DEBUG_MOTION
    printf("last: %f %f %f\n", _last_odom.x, _last_odom.y, _last_odom.theta);
    printf("cur: %f %f %f\n", odom_data.x, odom_data.y, odom_data.theta);
    printf("delta: %f %f %f\n", delta.x, delta.y, delta.theta);
    cin.get();

    

#endif
}

void ParticleFilter::runSensorModel() {

}

void ParticleFilter::visualizeParticles() {
    int res = _map.getResolution();

    createMarker(_points_marker, 1);
    createMarker(_lines_marker, 2);
   
    std::vector<geometry_msgs::Point> p_vec; 
    std::vector<geometry_msgs::Point> l_vec;
    for (int i = 0; i < _particles_list.size(); i++) {
        geometry_msgs::Point p;
        // Set position and orientation of particle
        // Multiply by map resolution to properly represent where they are on the map
        p.x = _particles_list[i].pose.x * res;
        p.y = _particles_list[i].pose.y * res;
        p.z = 0;
        p_vec.push_back(p);

        // Draw line (2 points) to show orientation
        // Multiply by map resolution to properly represent where they are on the map
        geometry_msgs::Point p1;
        p1.x = 0.5*res*cos(_particles_list[i].pose.theta)+ p.x;
        p1.y = 0.5*res*sin(_particles_list[i].pose.theta)+ p.y;
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

#ifdef DEBUG_INIT
    printf("p size: %lu\n", p_vec.size());
    printf("l size: %lu\n", l_vec.size());
#endif

    _particles_pub.publish(_points_marker);
    _lines_pub.publish(_lines_marker);
}
