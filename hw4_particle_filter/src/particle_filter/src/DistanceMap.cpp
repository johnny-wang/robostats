#include "particle_filter/DistanceMap.h"

#define PI 3.14159265

using namespace std;

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

//#define DEBUG

DistanceMap::DistanceMap() :
    _map_loaded(false),
    _package_path(ros::package::getPath("particle_filter")),
    _ray_step_size(0.25f),  // Half the resolution. Resolution is 10cm meaning each map square is 
                            // 10cm x 10cm. Our step size will be 5cm or 0.5 of map square.
    _row_min(0),
    _col_min(0)
{
}

/*
 * (0, 0) row/col starts at the UPPER LEFT corner of the map.
 */
DistanceMap::DistanceMap(nav_msgs::OccupancyGrid ocmap, int num_degree)
    : _col_dim(ocmap.info.width),
      _row_dim(ocmap.info.height),
      _resolution(ocmap.info.resolution),
      _num_measurements(num_degree),     // one measurement per degree
      _ray_step_size(0.25f),  // Half the resolution. Resolution is 10cm meaning each map square is 
                              // 10cm x 10cm. Our step size will be 5cm or 0.5 of map square.
      _angle_step_size(2*PI / num_degree),
      _row_min(0),
      _col_min(0)
{
    _nav_occ_map = ocmap;
    _map_loaded = true;
    initialize();
}

unsigned int DistanceMap::getRowSize() {
    return _row_dim;
}

unsigned int DistanceMap::getColSize() {
    return _col_dim;
}

unsigned int DistanceMap::getNumMeasurements() {
    return _num_measurements;
}

unsigned int DistanceMap::getMapValue(int row, int col) {
    return _oc_map[row][col];
}

/*
 * Theta is in RADIANS!!!
 */
float DistanceMap::getDistValue(int row, int col, float theta) {
    // Addition part is to help round to nearest angle
    int multiplier = (theta + (_angle_step_size/2)) / _angle_step_size;
#ifdef DEBUG
    printf("mod of %f %f: %d\n", theta, _angle_step_size, multiplier);
#endif
    return _dist_map[row][col][multiplier];
}

unsigned int DistanceMap::getNumDistRow() {
    return _dist_map.size();
}

unsigned int DistanceMap::getNumDistCol() {
    return _dist_map[0].size();
}

/* 
 * Load the saved map according to the format specified in saveDistMap() comment.
 */
void DistanceMap::loadDistMap(std::string filename) {
    ifstream data_in;
    std::string line;

    // Load occupancy map if not already loaded
    if (!_map_loaded) {
        cerr << "ERROR: need to load occupancy map first!" << endl;
        exit(-1);
    }

    std::string fname = _package_path + "/data/map/" + filename;

    data_in.open(fname.c_str());

    if (data_in.is_open()) {
        cout << "Loading distance map data from " << fname << endl;

        std::vector< std::vector<float> > col_data;
        // Loop through each line of the file
        int count = 0;
        while(getline(data_in, line)) {

            vector<string> strs;
            boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

            int row = atoi(strs[0].c_str());
            int col = atoi(strs[1].c_str());

            std::vector<float> dist_val;
            for (int j = 2; j < strs.size(); j++) {
                dist_val.push_back(atof(strs[j].c_str()));
            }

            col_data.push_back(dist_val);

            if (col == _col_dim-1) {
                _dist_map.push_back(col_data);
                col_data.clear();
            }
            count++;
            // Just so we know the program didn't hang.
            if ((count % 2500) == 0) {
                cout << "Reading file line: " << count << endl;
            }
        }

    } else {
    // file doesn't exist, create the distance map then save it.
        cout << "File doesn't exist: " << fname << endl;
        cout << "Creating new distance map" << endl;
        create_distance_map();
        saveDistMap(filename);
    }
    cout << "Finished loading" << endl;
    data_in.close();
}

void DistanceMap::loadOccMap(std::string filename) {
    ifstream data_in;
    std::string line;
    int map_data_counter = 0;  // to track where to write for _nav_occ_map.data

    /* First few rows of the map data:
    0 = size_x
    1 = size_y
    2 = resolution
    3 = shift_x
    4 = shift_y
    */
    vector<int> map_info;
    _nav_occ_map.header.frame_id = "/map";
    _nav_occ_map.header.stamp = ros::Time::now();

    std::string map_file = _package_path + "/data/map/" + filename;

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
                _nav_occ_map.info.resolution = map_info[RESOLUTION];
                _nav_occ_map.info.width = map_info[SIZE_X] / map_info[RESOLUTION];
                _nav_occ_map.info.height = map_info[SIZE_Y] / map_info[RESOLUTION];
                _nav_occ_map.data.resize( _nav_occ_map.info.height * _nav_occ_map.info.width );
                _col_dim = _nav_occ_map.info.width;
                _row_dim = _nav_occ_map.info.height;
                _resolution = _nav_occ_map.info.resolution;

            } else if (line_cnt == 6) {
            // this line has "global_map[0]: 800 800"
            } else {
            // fill in the occupancy map by row 
                vector<string> strs;
                boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

                // Some lines has extra line element that is empty space
                if (strs.size() == (_nav_occ_map.info.width+1)) {
                    strs.pop_back();
                }

                vector<string>::iterator it;

                for (it = strs.begin(); it != strs.end(); it++) {
                    float input = atof(it->c_str());
                    if (input >= 0) { input = 1-input;    // b/c the file has occupancy backwards
                                            // 1 = occupied; 0 = not occupied
                        if (input > 0.9) {   // occupied space
                            _nav_occ_map.data[map_data_counter++] = 100;// // map takes 0-100
                        } else if ((input <= 0.9) && (input > 0.1)) {
                            // probability of 50 that it's occupied
                            _nav_occ_map.data[map_data_counter++] = 50;// // map takes 0-100
                        } else {
                            // free space
                            _nav_occ_map.data[map_data_counter++] = 0;// // map takes 0-100
                        }
                        // No tier-ing
                        //_nav_occ_map.data[map_data_counter++] = input * 100;// // map takes 0-100
                    } else {
                        _nav_occ_map.data[map_data_counter++] = -1;    // -1: unknown
                    }
                }  // end for loop of each tokens
            }  // end line
            line_cnt++;
        }
        cout << "line count: " << line_cnt << endl;

        data_in.close();
    } else {
        cerr << "Unable to open file: " << map_file << endl;
        exit(-1);
    }
    _map_loaded = true;
    _occ_map_pub.publish(_nav_occ_map);
}

void DistanceMap::loadMaps(
        ros::NodeHandle nh, 
        std::string occ_map, 
        std::string dist_map, 
        int num_degrees, 
        float ray_step) {
    _occ_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    setNumDegrees(num_degrees);
    setRayStepSize(ray_step);
    loadOccMap(occ_map);
    initialize();
    loadDistMap(dist_map);
}

/* 
 * Save the distance map to file so we don't have to calculate it everytime.
 * Each row of the file will be saved in this format:
 *   [row num] [col num] [_dist_map[0]] [_dist_map[1]] [_dist_map[2]] ...
 */
void DistanceMap::saveDistMap(std::string filename) {
    ofstream data_out;
    filename = _package_path + "/data/map/" + filename;
    data_out.open(filename.c_str());

    if (!data_out.is_open()) {
        cerr << "ERROR: Unable to open file: " << filename << endl;
        return;
    }

    // Write to file in this format:
    for (int row = 0; row < _row_dim; row++) {
        for (int col = 0; col < _col_dim; col++) {
            data_out << row << " " << col;
            for (int dist = 0; dist < _num_measurements; dist++) {
                data_out << " " << _dist_map[row][col][dist];
            }
            data_out << endl;
        }
    }

    cout << "Distance map saved to " << filename << endl;
    data_out.close();
}

void DistanceMap::setNumDegrees(int num_degrees) {
    _num_measurements = num_degrees;
    _angle_step_size = 2*PI / num_degrees;
}

void DistanceMap::setRayStepSize(float ray_step) {
    _ray_step_size = ray_step;
}

/*********************************************************************/

/*
 * Reshape the occupancy map into 2D matrix of vectors so we can create a
 * distance map with it later. 
 */
void DistanceMap::initialize() {

    if (!_map_loaded) {
        cerr << "ERROR: Need to load occupancy map first!" << endl;
        exit(-1);
    }

    for (int row = 0; row < _row_dim; row++) {
        std::vector<int> col_data;
        for (int col = 0; col < _row_dim; col++) {
            int tmp = _nav_occ_map.data[_row_dim * row + col];
            col_data.push_back(tmp);
        }
        _oc_map.push_back(col_data);
    }
}

/*
 * Create distance map of each cell to the closes obstacles.
 * Each map cell will have _num_measurements of distances representing the ray casts from
 * _num_measurements / 360 degrees.
 * If the map cell is "unknown" (-1) or an obstacle (100), then the vector of measurements will
 * be all 0s.
 *
 * Distances will start with 0 degree pointing directly to the right of the map.
 * This is because cos(0) is 1, which is our 'x' direction.
 */
void DistanceMap::create_distance_map() {
//#define DEBUG

    for (int row = 0; row < _row_dim; row++) {
        std::vector< std::vector<float> > col_data;
        for (int col = 0; col < _row_dim; col++) {

            std::vector<float> dist(_num_measurements);

            // If cell is wall or unknown then all distance values are 0
            if ((getMapValue(row, col) == UNKNOWN) || (getMapValue(row, col) == WALL)) {
                std::fill(dist.begin(), dist.begin()+_num_measurements, 0.0f);
            } else {
            // ray cast
                float angle = 0.0f;    // angles are in radians
                //float angle_step = PI / float(_num_measurements);
#ifdef DEBUG
                cout << "ang step: " << _angle_step_size << endl;
#endif
                // Cycle through all the angles
                for (int ray = 0; ray < _num_measurements; ray++) {
                    float ray_dist = calculate_dist(row, col, angle);
                    ray_dist *= _resolution;    // multiply by resolution to get values in cm
                    dist[ray] = ray_dist;
                    angle += _angle_step_size;
#ifdef DEBUG
                    cout << "r: " << ray << " ray_dist: " << ray_dist << endl;
#endif 
                }
#ifdef DEBUG
                cout << "final angle: " << angle << endl;
#endif 

            }
            col_data.push_back(dist);
        }
        _dist_map.push_back(col_data);
    }
/* DEBUG 
    std::cout << "HERE" << std::endl;
    std::cout << _dist_map.size() << std::endl;
    std::cout << _dist_map[0].size() << std::endl;
    std::cout << _dist_map[0][0].size() << std::endl;
*/
}

/*
 */
float DistanceMap::calculate_dist(int row, int col, float angle) {
    float cur_row = (float)row;
    float cur_col = (float)col;

    float step_col = _ray_step_size * cos(angle);
    float step_row = _ray_step_size * sin(angle);

#ifdef DEBUG_1
    cout << "r: " << row << " c: " << col << " a: " << angle << endl;
    cout << "   " << _col_dim << " " << _row_dim << endl;
#endif
    cur_col += step_col;
    cur_row += step_row;

    // Make sure within map boundaries
    while ((cur_col >= 0) && (cur_col < _col_dim) && (cur_row >= 0) && (cur_row < _row_dim)) {
        int map_val = getMapValue(int(cur_row), int(cur_col));

        if ((map_val == WALL)) {
            break;
        }
        cur_col += step_col;
        cur_row += step_row;
    }

#ifdef DEBUG
    cout << "cur_col: " << cur_col << " cur_row: " << cur_row << endl;
    cout << "col: " << col << " row: " << row << endl;
#endif

    float x_squared = (cur_col - float(col)) * (cur_col - float(col));
    float y_squared = (cur_row - float(row)) * (cur_row - float(row));

    return sqrt(x_squared + y_squared);
}

void DistanceMap::print_dist_map() {
    cout << "Distance Map size" << endl;
    cout << _dist_map.size() << endl;
    cout << _dist_map[0].size() << endl;

    for (int i = 0; i < _dist_map.size(); i++) {
        for (int j = 0; j < _dist_map[0].size(); j++) {

        }
    }
}

void DistanceMap::print_occ_map() {
    cout << "Occupancy Map size" << endl;
    cout << _nav_occ_map.data.size() << endl;
}
