// load_data.cpp
// Load the laser and odometry data

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include <particle_filter_msgs/laser_odom.h>

// basic file operations
#include <iostream>
#include <fstream>

// Boost for string tokenizing
#include <boost/algorithm/string.hpp>

using namespace std;

ros::Publisher laser_pub;
ros::Publisher odom_pub;

const int NUM_LASER_READINGS = 180;

std::string g_laser_frame = "laser_frame";
std::string g_input_file;

#define PI 3.14159265359

// uncomment for debug output
//#define DEBUG

void load_data() {
    using namespace boost;
    ifstream data_fs;
    std::string line;

    /*
     EXAMPLE INPUT
        L -94.234001 -139.953995 -1.342158 -88.567719 -164.303391 -1.342158 66 66 66 66 66 65 66 66 65 66 66 66 66 66 67 67 67 66 67 66 67 67 67 68 68 68 69 67 530 514 506 508 494 481 470 458 445 420 410 402 393 386 379 371 365 363 363 364 358 353 349 344 339 335 332 328 324 321 304 299 298 294 291 288 287 284 282 281 277 277 276 274 273 271 269 268 267 266 265 265 264 263 263 263 262 261 261 261 261 261 193 190 189 189 192 262 262 264 194 191 190 190 193 269 271 272 274 275 277 279 279 281 283 285 288 289 292 295 298 300 303 306 309 314 318 321 325 329 335 340 360 366 372 378 384 92 92 91 89 88 87 86 85 84 83 82 82 81 81 80 79 78 78 77 76 76 76 75 75 74 74 73 73 72 72 72 71 72 71 71 71 71 71 71 71 71 70 70 70 70 0.025466
        O -94.234001 -139.953995 -1.342158 0.025863
        O -94.234001 -139.953995 -1.342158 0.079745

    */

    std::string package_path = ros::package::getPath("particle_filter");
    //std::string in_data = package_path + "/data/log/robotdata1.log";
    std::string in_data = package_path + "/data/log/";
    in_data = in_data + g_input_file;
#ifdef DEBUG
    cerr << "file: " << in_data << endl;
#endif
    
    data_fs.open(in_data.c_str());

    if (data_fs.is_open()) {
        cerr << "Opening data file " << in_data << endl;

        int line_cnt = 0;
        int data_cnt = 0;
        int laser_data = 0;
        int odom_data = 0;
        while (getline(data_fs, line)) {
            // parse by empty space
            vector<string> strs;
            boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

            // Error out if not Laser or Odom message
            if ((strs[0].compare("L") != 0) && (strs[0].compare("O") != 0)) {
                // Neither odom nor laser
                cerr << "Data is neither laser nor odom!" << endl;
                cerr << line << endl;
                exit(-1);
            }
            /*** parse laser data ***/
            if (strs[0].compare("L") == 0) {
                // Pose2D (pose) for x, y, theta
                // Pose2D (pose_l) for xl, yl, thetal
                // LaserScan (scan) for 180 laser readings
                particle_filter_msgs::laser_odom scan;

                /* 
                 * Laser data should have 188 elements:
                 * 1 for 'L'
                 * 3 for x, y, theta (coordinate of robot in standard odom frame when scan taken)
                 * 3 for xl, yl, thetal (coordinates of the lasers in standard odom frame when scan taken)
                 * 180 for scan readings at each degree of 180 degrees (right to left)
                 * 1 for timestamp
                 */
                // x, y, theta => coordinate of robot in standard odom frame when laser scan taken
                scan.pose.x = atof(strs[1].c_str());
                scan.pose.y = atof(strs[2].c_str());
                scan.pose.theta = atof(strs[3].c_str());
                scan.pose_l.x = atof(strs[4].c_str());
                scan.pose_l.y = atof(strs[5].c_str());
                scan.pose_l.theta = atof(strs[6].c_str());
#ifdef DEBUG
                printf("%f %f %f %f %f %f | %f %f\n", scan.pose.x, scan.pose.y, scan.pose.theta, 
                        scan.pose_l.x, scan.pose_l.y, scan.pose_l.theta, 
                        atof(strs[7].c_str()), atof(strs[strs.size()-2].c_str()));
#endif 
                // store laser scans
                scan.laser.header.frame_id = g_laser_frame;
                // 0 to 179 degrees because there are 180 readings
                scan.laser.angle_min = 0;
                scan.laser.angle_max = 179 * PI/180;  // to radians
                scan.laser.angle_increment = PI / NUM_LASER_READINGS;
                scan.laser.range_min = 0.0;
                scan.laser.range_max = 8500;  // in cm, heuristically 8183 cm
                scan.laser.ranges.resize(NUM_LASER_READINGS);

                float max_val = 0;
                int strs_offset = 7;
                for (int i = 7; i < strs.size()-1; i++) {
                    float input = atof(strs[i].c_str());
                    if (input > max_val) {
                        max_val = input;
                    }
                    scan.laser.ranges[i-strs_offset] = input;
                }
#ifdef DEBUG
                cout << "MAX: " << max_val << endl;
#endif
                laser_data++;
                data_cnt++;
                laser_pub.publish(scan);
            }
            /*** parse odom data ***/
            if (strs[0].compare("O") == 0) {
                geometry_msgs::Pose2D pose;
                //pose.header.frame_id = "pose";
                // TODO ADD TIME HERE??

                // Get the rest of the line (not including "O")
                float x = atof(strs[1].c_str());
                float y = atof(strs[2].c_str());
                float theta = atof(strs[3].c_str());
                pose.x = x;
                pose.y = y;
                pose.theta = theta;
#ifdef DEBUG
                cerr << x << " " << y << " " << theta << endl;
#endif
                odom_data++;
                data_cnt++;
                odom_pub.publish(pose);
            } 
            line_cnt++;
        }
        cout << "line count: " << line_cnt << endl;
        cout << "all data: " << data_cnt << endl;
        cout << "laser: " << laser_data << endl;
        cout << "odom: " << odom_data << endl;

        data_fs.close();
    } else {
        cerr << "Unable to open file" << endl;
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "load_data");
    ros::NodeHandle nh;

    std::string laser_topic;
    std::string odom_topic;

    // Get filename for input file
    nh.getParam("odom_file", g_input_file);
    nh.getParam("laser_topic", laser_topic);
    nh.getParam("odom_topic", odom_topic);

    laser_pub = nh.advertise<particle_filter_msgs::laser_odom>(laser_topic, 3000);
    odom_pub = nh.advertise<geometry_msgs::Pose2D>(odom_topic, 3000);

    load_data();

    //while (nh.ok()) {
    //    ros::spinOnce();
    //}

    return 0;
}
