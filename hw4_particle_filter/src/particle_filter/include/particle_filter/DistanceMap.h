#ifndef __DISTANCE_MAP_H__
#define __DISTANCE_MAP_H__

#include <nav_msgs/OccupancyGrid.h>

#include <stdio.h>
#include <math.h>

class DistanceMap {

public:

    DistanceMap(nav_msgs::OccupancyGrid map, int num_degree);

    unsigned int getRowSize();
    unsigned int getColSize();
    unsigned int getResolution();
    unsigned int getNumMeasurements();
    unsigned int getNumDistRow();
    unsigned int getNumDistCol();
    unsigned int getMapValue(int row, int col);
    float getDistValue(int row, int col, float theta);

protected:

    void initialize(nav_msgs::OccupancyGrid ocmap);
    void create_distance_map();
    float calculate_dist(int row, int col, float angle);

    std::vector< std::vector<int> > _oc_map;
    std::vector< std::vector< std::vector<float> > > _dist_map;

    int _row_dim;
    int _col_dim;
    int _resolution;
    int _num_measurements;

    float _ray_step_size;
    float _angle_step_size;
};

#endif
