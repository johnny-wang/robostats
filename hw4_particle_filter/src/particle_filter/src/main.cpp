#include <ros/ros.h>
#include <ros/package.h>

#include "particle_filter/DistanceMap.h"
#include "particle_filter/ParticleFilter.h"

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "main_pf_node");
    ros::NodeHandle nh;
    std::string laser_topic;
    std::string odom_topic;
    std::string data_file;
    int num_particles;
    int msg_queue_size;
    int num_degrees;
    double ray_step;

    // Get values from launch file
    nh.getParam("laser_topic", laser_topic);
    nh.getParam("odom_topic", odom_topic);
    nh.getParam("particles", num_particles);
    nh.getParam("msg_queue_size", msg_queue_size);
    nh.getParam("num_degrees", num_degrees);
    nh.getParam("ray_step_size", ray_step);
    nh.getParam("odom_file", data_file);

//#define DEBUG
#ifdef DEBUG
    std::string occ_map_name = "testmap.dat";
    std::string dist_map_name = "testmap.dat.txt";
#else
    std::string occ_map_name = "wean.dat";
    std::string dist_map_name = "wean.dat.txt";
#endif

    ParticleFilter *pf = new ParticleFilter(
        nh,
        laser_topic,
        odom_topic,
        msg_queue_size,
        num_particles,
        data_file,
        occ_map_name,
        dist_map_name,
        num_degrees,
        (float)ray_step
        );

    while (nh.ok()) {
        // break if done running
        if (!pf->run()) {
            break;
        }
        ros::spinOnce();
    }
    pf->closeFile();

    delete pf;

    return 0;
}
