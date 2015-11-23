#include "particle_filter/DataLoader.h"

using namespace std;

const int NUM_LASER_READINGS = 180;

// uncomment for debug output
//#define DEBUG
//#define DEBUG_LASER
//#define DEBUG_ODOM

DataLoader::DataLoader(std::string fname) :
    _filename(fname) 
{
    if (!initialize()) {
        cerr << "Initialization of DataLoader object FAILED" << endl;
        exit(-1);
    }
}

/*
 * Returns:
 *  0 - no more data
 *  1 - data is laser
 *  2 - data is odom
 */
OdomType DataLoader::parseNextData()
{
    std::string line;
    static int laser_cnt = 0;
    static int odom_cnt = 0;

    if (getline(_data_fs, line)) {
        vector<string> strs;
        boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

        /*** Laser data ***/
        if (strs[0].compare("L") == 0) {
            // Pose2D (pose) for x, y, theta
            // Pose2D (pose_l) for xl, yl, thetal
            // LaserScan (scan) for 180 laser readings
            //particle_filter_msgs::laser_odom _parsed_laser;

            /* 
             * Laser data should have 188 elements:
             * 1 for 'L'
             * 3 for x, y, theta (coordinate of robot in standard odom frame when scan taken)
             * 3 for xl, yl, thetal (coordinates of the lasers in standard odom frame when scan taken)
             * 180 for scan readings at each degree of 180 degrees (right to left)
             * 1 for timestamp
             */
            // x, y, theta => coordinate of robot in standard odom frame when laser scan taken
            _parsed_laser.pose.x = atof(strs[1].c_str());
            _parsed_laser.pose.y = atof(strs[2].c_str());
            _parsed_laser.pose.theta = atof(strs[3].c_str());
            _parsed_laser.pose_l.x = atof(strs[4].c_str());
            _parsed_laser.pose_l.y = atof(strs[5].c_str());
            _parsed_laser.pose_l.theta = atof(strs[6].c_str());
#ifdef DEBUG_LASER
            printf("%f %f %f %f %f %f | %f %f\n", _parsed_laser.pose.x, _parsed_laser.pose.y, 
                   _parsed_laser.pose.theta, _parsed_laser.pose_l.x, _parsed_laser.pose_l.y, 
                   _parsed_laser.pose_l.theta, atof(strs[7].c_str()), atof(strs[strs.size()-2].c_str()));
#endif
            // store laser scans
            _parsed_laser.laser.header.frame_id = "laser_frame";
            _parsed_laser.laser.angle_min = -PI / 2;
            _parsed_laser.laser.angle_max = PI / 2;
            _parsed_laser.laser.angle_increment = PI / NUM_LASER_READINGS;
            _parsed_laser.laser.range_min = 0.0;
            _parsed_laser.laser.range_max = 8500;  // in cm, heuristically 8183 cm
            _parsed_laser.laser.ranges.resize(NUM_LASER_READINGS);

            float max_val = 0;
            int strs_offset = 7;
            for (int i = 7; i < strs.size()-1; i++) {
                float input = atof(strs[i].c_str());
                if (input > max_val) {
                    max_val = input;
                }
                _parsed_laser.laser.ranges[i-strs_offset] = input;
            }
#ifdef DEBUG_LASER
            cerr << "MAX: " << max_val << endl;
            cerr << "laser cnt: " << laser_cnt << endl;
#endif
            laser_cnt++;
            //laser_pub.publish(_parsed_laser);
            return LASER;
        } else if (strs[0].compare("O") == 0) {
        /*** parse odom data ***/
            //geometry_msgs::Pose2D pose;

            // Get the rest of the line (not including "O")
            float x = atof(strs[1].c_str());
            float y = atof(strs[2].c_str());
            float theta = atof(strs[3].c_str());
            _parsed_odom.x = x;
            _parsed_odom.y = y;
            _parsed_odom.theta = theta;
#ifdef DEBUG_ODOM
            cerr << x << " " << y << " " << theta << endl;
            cerr << "odom cnt: " << odom_cnt << endl;
#endif
            odom_cnt++;
            //odom_pub.publish(pose);
            return ODOM;
        } else {
        // Error out if not Laser or Odom message (should never happen)
        // ((strs[0].compare("L") != 0) && (strs[0].compare("O") != 0))
            cerr << "Data is neither laser nor odom!" << endl;
            cerr << line << endl;
            exit(-1);
        }
    } else {
        return NONE;   // no more lines
    }
}

/* 
 * Return the Odom message that was parsed
 */
geometry_msgs::Pose2D DataLoader::getOdomData()
{
    return _parsed_odom;
}

/*
 * Return the laser data that was parsed
 */
particle_filter_msgs::laser_odom DataLoader::getLaserData()
{
    return _parsed_laser;
}

void DataLoader::closeFile()
{
    _data_fs.close();
}

/***********************************************************************************/

/*
 * Return false if fail to initialize
 */
bool DataLoader::initialize()
{
    _package_path = ros::package::getPath("particle_filter");
    std::string in_data = _package_path + "/data/log/";
    in_data = in_data + _filename;
#ifdef DEBUG
    cerr << "file: " << in_data << endl;
#endif

    _data_fs.open(in_data.c_str());

    if (_data_fs.is_open()) {
#ifdef DEBUG
        cerr << "filestream open SUCCESS" << endl;
#endif
        return true;
    } else {
#ifdef DEBUG
        cerr << "filestream open FAILED" << endl;
#endif
        return false;
    }
}
