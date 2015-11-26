#ifndef __DATA_LOADER_H__
#define __DATA_LOADER_H__

// ROS stuff
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
// Custom message
#include <particle_filter_msgs/laser_odom.h>

// basic file operations
#include <iostream>
#include <fstream>

// Boost for string tokenizing
#include <boost/algorithm/string.hpp>

#define PI 3.14159265359

enum OdomType { NONE, LASER, ODOM };

class DataLoader
{
public:
    DataLoader(std::string fname);

    OdomType parseNextData();
    geometry_msgs::Pose2D getOdomData();
    particle_filter_msgs::laser_odom getLaserData();
    void closeFile();
    void setMapScale(float scale);

private:
    bool initialize();
    float _map_scale;

    std::string _filename;
    std::string _package_path;
    std::ifstream _data_fs;
    geometry_msgs::Pose2D _parsed_odom;
    particle_filter_msgs::laser_odom _parsed_laser;
};

#endif
