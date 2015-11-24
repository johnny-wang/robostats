// load_map.cpp
// Load the given occupancy map

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>

// basic file operations
#include <iostream>
#include <fstream>

#include <ctime>

// Boost for string tokenizing
#include <boost/algorithm/string.hpp>

#include "particle_filter/DistanceMap.h"

#define UNKNOWN -1
#define WALL 100
#define FREE 60

#define PI 3.14159265

using namespace std;

ros::Publisher occ_map_pub;
ros::Publisher dist_map_pub;

std::string g_package_path = ros::package::getPath("particle_filter");
std::string g_map_file;

void load_map(nav_msgs::OccupancyGrid &ocmap) {
    using namespace boost;
    ifstream data_in;
    std::string line;

    /*
    0 = size_x
    1 = size_y
    2 = resolution
    3 = shift_x
    4 = shift_y
    */
    vector<int> map_info;
    ocmap.header.frame_id = "/map";
    ocmap.header.stamp = ros::Time::now();

    int map_data_counter = 0;  // to track where to write for ocmap.data

    //g_map_file = g_package_path + "/data/map/wean.dat";
    std::string map_file = g_package_path + "/data/map/" + g_map_file;
    
    data_in.open(map_file.c_str());

    if (data_in.is_open()) {
        cout << "Opening map file " << map_file << endl;

        int line_cnt = 0;
        while (getline(data_in, line)) {

            /* Parse the first 5 lines:
             * robot_specifications->global_mapsize_x                 8000
             * robot_specifications->global_mapsize_y                 8000
             * robot_specifications->resolution                       10
             * robot_specifications->autoshifted_x                    0
             * robot_specifications->autoshifted_y                    0
             */
            if (line_cnt < 5) {
                cout << line << endl;

                vector<string> strs;
                boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

                string it = strs.back();
                cout << atoi(it.c_str()) << endl;
                map_info.push_back(atoi(it.c_str()));
            } else if (line_cnt == 5) {
            // Set up occupancy map after reading in the map info
            // line 5 has empty line
                ocmap.info.resolution = map_info[RESOLUTION];
                ocmap.info.width = map_info[SIZE_X] / map_info[RESOLUTION];
                ocmap.info.height = map_info[SIZE_Y] / map_info[RESOLUTION];
                ocmap.data.resize( ocmap.info.height * ocmap.info.width );

            } else if (line_cnt == 6) {
            // this line has "global_map[0]: 800 800"
            } else {
            // fill in the occupancy map by row 
                vector<string> strs;
                boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

                // Some lines has extra line element that is empty space
                if (strs.size() == (ocmap.info.width+1)) {
                    strs.pop_back();
                }

                vector<string>::iterator it;

                for (it = strs.begin(); it != strs.end(); it++) {
                    float input = atof(it->c_str());
                    if (input >= 0) { input = 1-input;    // b/c the file has occupancy backwards
                                            // 1 = occupied; 0 = not occupied
                        if (input > 0.9) {   // occupied space
                            ocmap.data[map_data_counter++] = 100;// // map takes 0-100
                        } else if ((input <= 0.9) && (input > 0.1)) { 
                            // probability of 50 that it's occupied
                            ocmap.data[map_data_counter++] = 50;// // map takes 0-100
                        } else {       
                            // free space
                            ocmap.data[map_data_counter++] = 0;// // map takes 0-100
                        }
                        // No tier-ing
                        //ocmap.data[map_data_counter++] = input * 100;// // map takes 0-100
                    } else {
                        ocmap.data[map_data_counter++] = -1;    // -1: unknown
                    }
                }
                //map_data_counter--;  // so each line stays at 800 items read
            }
            line_cnt++;
        }
        cout << "line count: " << line_cnt << endl;

        data_in.close();
    } else {
        cerr << "Unable to open file: " << g_map_file << endl;
    }

    occ_map_pub.publish(ocmap);
}

// Reshape the map data into the appropriate x, y and then calculate the distance map
void calculate_distance_map(const nav_msgs::OccupancyGrid occupancy_map) {
    int x_size = occupancy_map.info.width;
    int y_size = occupancy_map.info.height;
    int ocmap[y_size][x_size];

    // break occupancy map into x by y
    for (int row = 0; row < x_size; row++) {
        for (int col = 0; col < y_size; col++) {
            int tmp = occupancy_map.data[x_size * row + col];
            ocmap[row][col] = tmp;
            //cout << tmp << " ";
            //cout << row << " " << col << " " << tmp << endl;
        }
        //cout << endl;
    }

    int num_deg;
    if (g_map_file.compare("testmap.dat") == 0) {
        num_deg = 4;
    } else {
        num_deg = 360;
    }

    DistanceMap map(occupancy_map, num_deg);

    std::clock_t start;
    double duration;
    start = std::clock();

    std::string save_file = g_package_path + "/data/map/dist_map.txt";
    map.loadDistMap(save_file);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    cout << "run time: " << duration << " sec" << endl;
    

    cout << "x dim: " << map.getXSize() << endl;
    cout << "y dim: " << map.getYSize() << endl;
    cout << "num meas: " << map.getNumMeasurements() << endl;
    cout << "dist row: " << map.getNumDistX() << endl;
    cout << "dist col: " << map.getNumDistY() << endl;
    cout << "val: " << map.getDistValue(1,1,0*PI/180) << endl;
    cout << "val: " << map.getDistValue(1,2,0*PI/180) << endl;
    cout << "val: " << map.getDistValue(1,3,0*PI/180) << endl;
    cout << "val: " << map.getDistValue(1,4,0*PI/180) << endl;

    map.saveDistMap(save_file);
/*
    for (int row = 0; row < x_size; row++) {
        for (int col = 0; col < y_size; col++) {
            cout << (int)map.getMapValue(row, col) << " ";
        }
        cout << endl;
    }
*/
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "load_map");
    ros::NodeHandle nh;

    // latch occupancy map data
    occ_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    dist_map_pub = nh.advertise<particle_filter_msgs::distance_map>("dist_map", 1, true);

    // Get filename for map file
    nh.getParam("map_input", g_map_file);

    nav_msgs::OccupancyGrid ocmap;
    load_map(ocmap);
    calculate_distance_map(ocmap);

    while (nh.ok()) {
        ros::spinOnce();
    }

    return 0;
}
