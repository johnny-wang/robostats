#ifndef __DISTANCE_MAP_H__
#define __DISTANCE_MAP_H__

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

    unsigned int getRowSize();
    unsigned int getColSize();
    unsigned int getResolution();
    unsigned int getNumMeasurements();
    unsigned int getNumDistRow();
    unsigned int getNumDistCol();
    unsigned int getMapValue(int row, int col);
    float getDistValue(int row, int col, float theta);

    void loadMaps(std::string occ_map, std::string dist_map, int num_degree, float ray_step);
    void loadDistMap(std::string filename);
    void loadOccMap(std::string filename);
    void saveDistMap(std::string filename);

    void setNumDegrees(int num_degrees);
    void setRayStepSize(float ray_step);

protected:

    void initialize();
    void create_distance_map();
    float calculate_dist(int row, int col, float angle);
    void print_dist_map();
    void print_occ_map();

    std::string _package_path;

    nav_msgs::OccupancyGrid _nav_occ_map;
    std::vector< std::vector<int> > _oc_map;
    std::vector< std::vector< std::vector<float> > > _dist_map;

    bool _map_loaded;

    int _row_dim;
    int _col_dim;
    int _resolution;
    int _num_measurements;

    float _ray_step_size;
    float _angle_step_size;
};

#endif
