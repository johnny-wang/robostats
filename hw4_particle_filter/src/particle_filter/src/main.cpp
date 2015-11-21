#include <ros/ros.h>
#include <ros/package.h>

#include "particle_filter/DistanceMap.h"
#include "particle_filter/ParticleFilter.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "main_pf_node");
    ros::NodeHandle nh;

    //nh.getParam("");
    // TODO load these as params
    std::string laser_topic = "laser_odom";
    std::string odom_topic = "odom";
    int msg_queue_size = 1000;
    int num_particles = 4000;
    std::string map_name = "test.dat";

    ParticleFilter *pf = new ParticleFilter(
        nh,
        laser_topic,
        odom_topic,
        msg_queue_size,
        num_particles,
        map_name
        );

    while (nh.ok()) {
        ros::spinOnce();
    }

    delete pf;

    return 0;
}
