#ifndef __DISTANCE_MAP_H__
#define __DISTANCE_MAP_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>

#include <stdio.h>
#include <math.h>
// basic file operations
#include <iostream>
#include <fstream>
// Boost for string tokenizing
#include <boost/algorithm/string.hpp>

enum MapInfo { SIZE_X, SIZE_Y, RESOLUTION, SHIFT_X, SHIFT_Y };

class DistanceMap {

public:

    DistanceMap();
    DistanceMap(nav_msgs::OccupancyGrid map, int num_degree);

    unsigned int getXSize();
    unsigned int getYSize();
    unsigned int getResolution();
    unsigned int getNumMeasurements();
    unsigned int getNumDistX();
    unsigned int getNumDistY();
    unsigned int getMapValue(int x, int y);
    float getDistValue(int x, int y, float theta);

    void loadMaps(ros::NodeHandle nh, 
        std::string occ_map, 
        std::string dist_map, 
        int num_degree, 
        float ray_step);
    void loadDistMap(std::string filename);
    void loadOccMap(std::string filename);
    void saveDistMap(std::string filename);

    void setNumDegrees(int num_degrees);
    void setRayStepSize(float ray_step);

protected:

    void initialize();
    void create_distance_map();
    float calculate_dist(int x, int y, float angle);
    void print_dist_map();
    void print_occ_map();

    std::string _package_path;

    nav_msgs::OccupancyGrid _nav_occ_map;
    std::vector< std::vector<int> > _oc_map;
    std::vector< std::vector< std::vector<float> > > _dist_map;

    bool _map_loaded;

    int _x_max, _x_min;
    int _y_max, _y_min;
    int _resolution;
    int _num_measurements;

    float _ray_step_size;
    float _angle_step_size;

    ros::Publisher _occ_map_pub;    // publish occupancy map once with latch
};

#endif
