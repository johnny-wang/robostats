
#include "particle_filter/ParticleFilter.h"

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

//#define DEBUG_INIT
//#define DEBUG_DATA
//#define DEBUG_MOTION
//#define DEBUG_SENSOR
//#define DEBUG_LASER
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
    _laser_pub(nh.advertise<visualization_msgs::Marker>("laser", 1, true)),
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
    _data.setMapScale(_map.getResolution());

    // initialize particles (uniformly)
    initializeParticles();

    //initialize model parameters
    //sensor models
    sensorSigma = 0.025; //cm
    z_max =  8250; //sensor model max range(not validated) cm
    lambda_short = 0.003; //sensor model lambda short (not validated)
    //tuning params for weighting params
    weight_max = 1;
    weight_rand = 0.005;
    weight_hit = 1;
    weight_short = 0.05;

    //Reseeding params
    placement_stdDev = 0.5; //cm from placement for 1 std dev
    rotation_stdDev = 0.1;  // rad std dev for resample noise
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
    cout << "Running particle filter" << endl;
#endif

    OdomType data = _data.parseNextData();

    switch(data) {
    case NONE:
    {
        cout << "No more data" << endl;
        return false;
        break;
    }
    case LASER:
    {
#ifdef DEBUG_DATA
        cout << "Laser " << laser_cnt << " all " << all_cnt << endl;
        laser_cnt++;
#endif
        // Laser has both odom and laser readings
        // Run motion first then sensor model 
        geometry_msgs::Pose2D odom_data = _data.getOdomData();
        runMotionModel(odom_data);

        // Run sensor model
        particle_filter_msgs::laser_odom laser_data = _data.getLaserData();
        runSensorModel(laser_data);

        all_cnt++;
        break;
    }
    case ODOM:
    {
#ifdef DEBUG_DATA
        cout << "Odom " << odom_cnt << " all " << all_cnt << endl;
        odom_cnt++;
#endif
        geometry_msgs::Pose2D odom_data = _data.getOdomData();
        // Run motion model
        runMotionModel(odom_data);

        all_cnt++;
        break;
    }
    default:
    {
        cout << "Default case in parsing Odom data" << endl;
        cout << "THIS SHOULD NEVER HAPPEN" << endl;
        break;
    }
    }
    printf("Processed %4d log data messages\n", all_cnt);
    
    visualizeParticles();
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

        marker.scale.x = 1.5 * res;
        marker.scale.y = 1.5 * res;
        marker.scale.z = 1.5 * res;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    } else if (type == 2) {
    // lines to show orientation
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.ns = "orientation_lines";

        marker.scale.x = 2.5 * res;
        marker.scale.y = 0.25 * res;
        marker.scale.z = 0.25 * res;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    } else {
        marker.id = 2;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.ns = "laser_pts";

        marker.scale.x = 0.5 * res;
        marker.scale.y = 0.5 * res;
        marker.scale.z = 0.5 * res;

        marker.color.r = 1.0f;
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
    std::uniform_real_distribution<double> rng1(x_min, x_max);
    std::uniform_real_distribution<double> rng2(y_min, y_max);
    std::uniform_real_distribution<double> rng3(-M_PI, M_PI);

#ifdef DEBUG_INIT
    std::cout << "Initializing particles" << std::endl;
    std::cout << "x size: " << x_max << endl;
    std::cout << "y size: " << y_max << endl;
#endif

    // Resample until we get particle in free space
    int id_cnt = 0;
    for (int i = 0; i < _num_particles; ) {
        particle_filter_msgs::particle p;
        p.pose.x = rng1(generator);
        p.pose.y = rng2(generator);
        p.pose.theta = rng3(generator);

#ifdef DEBUG_INIT
// This is the 'start' of log file 1
p.pose.x = 418;
p.pose.y = 400;
p.pose.theta = M_PI;
#endif

        if (_map.getMapValue(p.pose.x, p.pose.y) == FREE) {
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
#ifdef DEBUG_INIT
        else {
            cout << "Particle is NOT in free space" << std::endl;
            cout << "x: " << p.pose.x << " y: " << p.pose.y << " th: " << p.pose.theta << endl;
            cout << _map.getMapValue(p.pose.x, p.pose.y) << endl;
        }
#endif
    }

    std::cout << "Num particles: " << _particles_list.size() << std::endl;
    visualizeParticles();
}

Eigen::Matrix3f ParticleFilter::poseToMatrix(particle_filter_msgs::particle p) {
    return poseToMatrix(p.pose);
}

Eigen::Matrix3f ParticleFilter::poseToMatrix(geometry_msgs::Pose2D pose) {
    Eigen::Matrix3f m;
    m << cos(pose.theta), -sin(pose.theta), pose.x,
         sin(pose.theta),  cos(pose.theta), pose.y,
                      0 ,                0,      1;

#ifdef DEBUG_MOTION
    cout << m << endl;
#endif
}

geometry_msgs::Pose2D ParticleFilter::matrixToPose(Eigen::Matrix3f m) {
    geometry_msgs::Pose2D pose;

    pose.x = m(0,2);
    pose.y = m(1,2);
    pose.theta = atan2(m(1,0), m(1,1));
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

    //MotionUpdater motion_updater(0.01, 0.01, 0.01, 0.01);
    MotionUpdater motion_updater(0, 0, 0, 0);

    /* New world coordinate of particle is old world coord * inverse of odom prev * odom new
     * Latex syntax: T_{p2}^{W} = T_{p1}^{W} * (T_{p1}^{O})^{-1} * T_{p2}^{O}
     */
    for (int i = 0; i < _particles_list.size(); i++) {
        // Get particle_id, wt 
        particle_filter_msgs::particle updated_particle = _particles_list[i];

        motion_updater.sample_motion_model_odometry_v2(
            _particles_list[i],     
            updated_particle, 
            _last_odom, 
            odom_data);

        _particles_list[i] = updated_particle;

/*
        Eigen::Matrix3f p_world = poseToMatrix(_particles_list[i]);
        Eigen::Matrix3f odom_last = poseToMatrix(_last_odom);
        Eigen::Matrix3f odom_cur = poseToMatrix(odom_data);

        Eigen::Matrix3f p_world_new = p_world * odom_last.inverse() * odom_cur;

        updated_particle.pose = matrixToPose(p_world_new);
        _particles_list[i] = updated_particle;

        printParticle(updated_particle);
*/
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

    Eigen::Matrix3f last_pose = poseToMatrix(_last_odom);

#endif
    _last_odom = odom_data;
}


/************************************************************************
Sensor Model
*************************************************************************/
void ParticleFilter::runSensorModel(particle_filter_msgs::laser_odom laser_data) {

    //will store the probabiblity of each particle
    //float probability_list[_num_particles];
    std::vector<float> probability_list(_num_particles);

    //for each particle in list
    for (int particleIndex = 0; particleIndex < _num_particles; particleIndex++)
    {
        //get current particle from list
        particle_filter_msgs::particle particle = _particles_list[particleIndex];

        //get right ray angle in radians, [0,2pi)
        float rayAngle = particle.pose.theta - M_PI/2.0;
        if (rayAngle >= 2*M_PI) {rayAngle -= (2*M_PI);}

#ifdef DEBUG_SENSOR
        //_map.checkMaps();
        printf("laser scans at %f %f %f\n", particle.pose.x, particle.pose.y, rayAngle);
//        sensor_msgs::LaserScan laser = _map.getLaserScans(particle.pose.x, particle.pose.y, rayAngle);
        std::vector<float> laser = _map.getLaserVals(particle.pose.x, particle.pose.y, rayAngle);
        
//        pl = _map.getAllLaser(particle.pose.y, particle.pose.x);
//        for (int i=0; i<pl.size(); i++) {
//            cout << pl[i] << " ";
//        }
//        cout << endl;
        
        visualizeLaser(particle.pose, laser, rayAngle);
#endif

        //store the running probability
        float particleScore = 0;
        //for all 180 rays (180 degrees, 1 deg steps, same as input scan)
        //<TODO> possibly make this rely on data in laser in?
        for (int rayIndex = 0; rayIndex < 180; rayIndex ++)
        {
            //get predicted measurement from position and orientation (plus rel. ray angle)
            float predictedMeasure = _map.getDistValue(
                particle.pose.x, particle.pose.y, rayAngle);

            //int min = 1; int max = 8250;
            //float predictedMeasure = (rand()%(max-min))+min;; //temporary for testing
            
            float actualMeasure = laser_data.laser.ranges[rayIndex];
            double phit = prob_hit(predictedMeasure, actualMeasure);
            double pshort = prob_short(predictedMeasure, actualMeasure);
            double pmax = prob_max(actualMeasure);
            double prand = prob_rand(actualMeasure);

            double rayProb = weight_hit*phit + weight_short*pshort + 
                weight_max * pmax + weight_rand * prand;

            //update running probability
            particleScore += rayProb;

#ifdef DEBUG_SENSOR
    //output for each ray

    //printf("pshort = %f\n", pshort);
    //printf("Ind: %d; Angle (abs): %.1f; RayProb: %f; Measurement: %.2f; actual: %.2f\n",
        //rayIndex, rayAngle*180.0/M_PI, rayProb, actualMeasure, predictedMeasure);
    //printf("x: %0.2f y: %0.2f rayAngle: %0.2f",particle.pose.x, particle.pose.y, rayAngle );
#endif           

            //get next ray angle , [0,2pi)
            rayAngle -= (2*M_PI)/360;
            if (rayAngle < 0) {rayAngle += (2*M_PI);}
            
        }
        if (_map.getMapValue(particle.pose.x, particle.pose.y) == FREE) {
            probability_list[particleIndex] = particleScore;
        } else {
            probability_list[particleIndex] = 0;
        }
#ifdef DEBUG_SENSOR
    //output for each particle
    //printf("Particle: %d; Score: %f\n",particleIndex+1, particleScore);
#endif        

    }
#ifdef DEBUG_SENSOR
    //for each laser observation
    cin.get();
#endif

    //resample according to new weights
    resampleParticles(probability_list);
}

double ParticleFilter::prob_hit(double z_true, double z)
{
    //<TODO> finish this function
    double p = 0.0;

    if (z >= 0 && z <= z_max)
    {
        double diff = abs(z_true - z);

        p = (1/(sensorSigma*sqrt(2*M_PI)) *exp(-1.0*pow(diff,2) / (2*sensorSigma)  )   );

    }

    return p;
}

double ParticleFilter::prob_short(double z_true, double z)
{
    double p = 0.0;

    if (z >= 0 && z <= z_true)
    {
        double eta = 1 / (1 - exp(-lambda_short * z_true));
        p = (eta * lambda_short * exp(-lambda_short * z));
    }

    return p;
}

double ParticleFilter::prob_max(double z)
{
    if ( abs(z-z_max) < 0.001 ) 
        return 1.0;
    else
        return 0.0;
}

double ParticleFilter::prob_rand(double z)
{
    if (z >= 0 && z <= z_max)
        return (1.0/z_max);
    else
        return 0.0;
}

/************************************************************************
resample
*************************************************************************/
void ParticleFilter::resampleParticles(std::vector<float> probability_list)
{
    //<TODO> rnasci 

    //keep current locations of particles to select from
    std::vector<particle_filter_msgs::particle> particleLocationList = _particles_list;

    //set distro based on scores
    std::discrete_distribution<int> distribution(probability_list.begin(), probability_list.end());
    //std::discrete_distribution<int> distribution({10,10,40,10});

    for (int particleIndex = 0; particleIndex < _num_particles; particleIndex ++)
    {   
        //draw index according to score distro
        int index = distribution(generator);

        bool invalidPosition = true;
        int limit = 0;
        while (invalidPosition && limit < 25)
        {
            //change particle location w perturbations
            //x
            std::normal_distribution<float> distributionX(
                particleLocationList[index].pose.x,placement_stdDev);
            float x = distributionX(generator);
            //y
            std::normal_distribution<float> distributionY(
                particleLocationList[index].pose.y,placement_stdDev);
            float y = distributionY(generator);
            //th
            std::normal_distribution<float> distributionTH(
                particleLocationList[index].pose.theta,rotation_stdDev);
             float th = distributionTH(generator);

            if (x < _map.getXSize() && x > 0 && y < _map.getYSize() && y > 0 )
            {
                if (_map.getMapValue(x,y) == FREE) 
                {    
                    _particles_list[particleIndex].pose.x = x;
                    _particles_list[particleIndex].pose.y = y;
                    _particles_list[particleIndex].pose.theta = th;
                    invalidPosition = false;
//#define DEBUG_RESAMPLE
#ifdef  DEBUG_RESAMPLE                  
    printf("Relocated Particle: Base %0.2f,%0.2f,%0.2f; Now %0.2f.%0.2f.%0.2f",
        )
#endif
                }
            }
            limit ++;
        }

    }
}

/************************************************************************
Visualize Tools
*************************************************************************/
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

//cout << "cos " << cos(_particles_list[i].pose.theta) << endl;
//cout << "sin " << sin(_particles_list[i].pose.theta) << endl;
        p1.x = 0.25*res*sin(_particles_list[i].pose.theta)+ p.x;
        p1.y = 0.25*res*cos(_particles_list[i].pose.theta)+ p.y;
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
        //printf("%f %f %f %f %f\n", p.x, p.y, p1.x, p1.y, _particles_list[i].pose.theta*180/M_PI);
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

void ParticleFilter::visualizeLaser(
    geometry_msgs::Pose2D pose, 
    std::vector<float> laser,
    float ray_angle_start) 
{
    // 1 degree in radian
    float angle_step = 1 * M_PI / 180;
    std::vector<geometry_msgs::Point> p_vec;
    int res = _map.getResolution();

    createMarker(_laser_marker, 3);

#ifdef DEBUG_LASER
    std::vector<float> vec_ang(180);    // laser scans are 180 degrees
    vec_ang = _map.getLaserVals(pose.x, pose.y, ray_angle_start);
    std::vector<float> pl(360);
    pl = _map.getAllLaser(pose.x, pose.y);
    printf("vec size: %lu\n", vec_ang.size());

    for (int i=0; i<pl.size(); i++) {
        printf("%d: %f %f %f\n", i, pl[i], laser[i], vec_ang[i]);
    }
    cout << endl;
    cout << "visualizing lasers" << endl;
    cout << laser.size() << endl;
#endif

    for (int i = 0; i < laser.size(); i++) {
        // Save distances as points
        // calculate point's location wrt particle location
        geometry_msgs::Point p;
        p.x = (pose.x + laser[i]/res * cos(ray_angle_start + (i*angle_step))) * res;
        p.y = (pose.y + laser[i]/res * sin(ray_angle_start + (i*angle_step))) * res;
        p.z = 0;
        p_vec.push_back(p);
#ifdef DEBUG_LASER
        printf("laser pt: %f %f %f %f\n", p.x, p.y, laser[i], angle_step);
        cin.get();
#endif
    }
    _laser_marker.points = p_vec;
    _laser_pub.publish(_laser_marker);
}
