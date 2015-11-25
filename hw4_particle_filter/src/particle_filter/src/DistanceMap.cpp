#include "particle_filter/DistanceMap.h"

#define PI 3.14159265

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

//#define DEBUG
using namespace std;  // for debugging

DistanceMap::DistanceMap() :
    _map_loaded(false),
    _package_path(ros::package::getPath("particle_filter")),
    _ray_step_size(0.25f),  // Half the resolution. Resolution is 10cm meaning each map square is 
                            // 10cm x 10cm. Our step size will be 5cm or 0.5 of map square.
    _x_min(0),
    _y_min(0)
{
}

/*
 * (0, 0) x/y starts at the UPPER LEFT corner of the map.
 */
DistanceMap::DistanceMap(nav_msgs::OccupancyGrid ocmap, int num_degree)
    : _y_max(ocmap.info.width),
      _x_max(ocmap.info.height),
      _resolution(ocmap.info.resolution),
      _num_measurements(num_degree),     // one measurement per degree
      _ray_step_size(0.25f),  // Half the resolution. Resolution is 10cm meaning each map square is 
                              // 10cm x 10cm. Our step size will be 5cm or 0.5 of map square.
      _angle_step_size(2*PI / num_degree),
      _x_min(0),
      _y_min(0)
{
    _nav_occ_map = ocmap;
    _map_loaded = true;
    initialize();
}

unsigned int DistanceMap::getResolution() {
    return _resolution;
}

unsigned int DistanceMap::getXSize() {
    return _x_max;
}

unsigned int DistanceMap::getYSize() {
    return _y_max;
}

unsigned int DistanceMap::getNumMeasurements() {
    return _num_measurements;
}

unsigned int DistanceMap::getMapValue(int x, int y) {
    return _oc_map[x][y];
}

/*
 * INPUT:
 *  x, y = the positions of the particle.
 *  theta = the orientation of the particle (in RADIANS!!!)
 */
float DistanceMap::getDistValue(float x, float y, float theta) {
//#define DEBUG_DIST
    // Addition part is to help round to nearest angle
    int multiplier = (theta + (_angle_step_size/2)) / _angle_step_size;
#ifdef DEBUG_DIST
    printf("mod of %f %f: %d\n", theta, _angle_step_size, multiplier);
#endif
    // Need to convert to map's coordinate which is 10cm x 10cm per cell
    x /= _resolution;
    y /= _resolution;
    return _dist_map[x][y][multiplier];
}

unsigned int DistanceMap::getNumDistX() {
    return _dist_map.size();
}

unsigned int DistanceMap::getNumDistY() {
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

/*
cout << fname << endl;
    FILE* file = fopen(fname.c_str(), "r");

    if (NULL == file) {
        printf("Failed to open '%s'\n", fname.c_str());
        exit(-1);
    }
    int row, col;
    while(!feof(file)) {
        while(fscanf(file, "%d %d ", &row, &col)) {
            float tmp;
//            cout << "r: " << row << " c: " << col << endl;
            for (int i = 0; i < _num_measurements; i++) {
                fscanf(file, "%f ", &tmp);
//                cout << " " << tmp;
            }
//            cout << endl;
            fscanf(file, "%[^\n]\n", NULL); // skip the newline
//cin.get();
//break;
        }
    }
fclose(file);
*/
    boost::timer t;

    data_in.open(fname.c_str());

    if (data_in.is_open()) {
        cout << "Loading distance map data from " << fname << endl;

        _dist_map.resize(_x_max);
        // allocate memory
        for (int i = 0; i < _x_max; i++) {
            _dist_map[i].resize(_y_max);
            for (int j = 0; j < _y_max; j++) {
                _dist_map[i][j].resize(_num_measurements,0);
            }
        }

//print_dist_map();

        //std::vector< std::vector<float> > y_data(_y_max, std::vector<float>(_x_max));
        // Loop through each line of the file
        int count = 0;
        while(getline(data_in, line)) {

            vector<string> strs;
            boost::split(strs, line, boost::is_any_of(" "), boost::token_compress_on);

            int x = atoi(strs[0].c_str());
            int y = atoi(strs[1].c_str());

            //std::vector<float> dist_val;
            //dist_val.reserve(_num_measurements + 2); // 2 for row, col
            for (int j = 2; j < strs.size(); j++) {
                //dist_val.push_back(atof(strs[j].c_str()));
                _dist_map[x][y][j-2] = atof(strs[j].c_str());
            }

            //y_data.push_back(dist_val);

            //if (y == _y_max-1) {
            //    _dist_map.push_back(y_data);
            //    y_data.clear();
            //}
            count++;
            // Just so we know the program didn't hang.
            if ((count % 5000) == 0) {
                cout << "Reading file line: " << count << endl;
            }
        }

    } else {
    // file doesn't exist, create the distance map then save it.
        cout << "File doesn't exist: " << fname << endl;
        create_distance_map();
        saveDistMap(filename);
    }

//print_dist_map();

    cout << "Finished loading" << endl;
    cout << "time: " << t.elapsed() << endl;

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
                _y_max = _nav_occ_map.info.width;
                _x_max = _nav_occ_map.info.height;
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
 *   [x num] [y num] [_dist_map[0]] [_dist_map[1]] [_dist_map[2]] ...
 *
 * Only save the calculated values at free cells
 */
void DistanceMap::saveDistMap(std::string filename) {
    ofstream data_out;
    filename = _package_path + "/data/map/" + filename;
    data_out.open(filename.c_str());

    cout << "Saving distance map" << endl;

    if (!data_out.is_open()) {
        cerr << "ERROR: Unable to open file: " << filename << endl;
        return;
    }
    // Write to file in this format:
    for (int x = 0; x < _x_max; x++) {
        for (int y = 0; y < _y_max; y++) {
            // Don't write to file if it's unknown or occupied cell
            if ((getMapValue(x, y) == UNKNOWN) || (getMapValue(x, y) == WALL)) {
                // do nothing
            } else {
                data_out << x << " " << y << " ";
                for (int dist = 0; dist < _num_measurements; dist++) {
                    data_out << _dist_map[x][y][dist] << " ";
                }
                data_out << endl;
            }
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

    for (int x = 0; x < _x_max; x++) {
        std::vector<int> y_data;
        for (int y = 0; y < _x_max; y++) {
            int tmp = _nav_occ_map.data[_x_max * x + y];
            y_data.push_back(tmp);
        }
        _oc_map.push_back(y_data);
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

    cout << "Creating new distance map" << endl;

#ifdef DEBUG
    boost::timer t;
#endif

    int lines_written = 0;
    for (int x = 0; x < _x_max; x++) {
        std::vector< std::vector<float> > y_data;
        y_data.reserve(_x_max * _y_max);   // pre-allocate

        for (int y = 0; y < _x_max; y++) {

            std::vector<float> dist(_num_measurements);

            // If cell is wall or unknown then skip storing distance
            if ((getMapValue(x, y) == UNKNOWN) || (getMapValue(x, y) == WALL)) {
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
                    float ray_dist = calculate_dist(x, y, angle);
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
                lines_written++;
            }
            y_data.push_back(dist);
        }
        _dist_map.push_back(y_data);
    }
#ifdef DEBUG
    std::cout << "time: " << t.elapsed() << std::endl;
    std::cout << "lines: " << lines_written << std::endl;
#endif
/*
    std::cout << "HERE" << std::endl;
    std::cout << _dist_map.size() << std::endl;
    std::cout << _dist_map[0].size() << std::endl;
    std::cout << _dist_map[0][0].size() << std::endl;
*/
}

/*
 */
float DistanceMap::calculate_dist(int x, int y, float angle) {
    float cur_x = (float)x;
    float cur_y = (float)y;

    float step_y = _ray_step_size * cos(angle);
    float step_x = _ray_step_size * sin(angle);

#ifdef DEBUG_1
    cout << "r: " << x << " c: " << y << " a: " << angle << endl;
    cout << "   " << _y_max << " " << _x_max << endl;
#endif
    cur_y += step_y;
    cur_x += step_x;

    // Make sure within map boundaries
    while ((cur_y >= 0) && (cur_y < _y_max) && (cur_x >= 0) && (cur_x < _x_max)) {
        int map_val = getMapValue(int(cur_x), int(cur_y));

        if ((map_val == WALL)) {
            break;
        }
        cur_y += step_y;
        cur_x += step_x;
    }

#ifdef DEBUG
    cout << "cur_y: " << cur_y << " cur_x: " << cur_x << endl;
    cout << "y: " << y << " x: " << x << endl;
#endif

    float x_squared = (cur_x - float(x)) * (cur_x - float(x));
    float y_squared = (cur_y - float(y)) * (cur_y - float(y));

    return sqrt(x_squared + y_squared);
}

void DistanceMap::print_dist_map() {
    cout << "Distance Map size" << endl;
    cout << _dist_map.size() << endl;
    cout << _dist_map[0].size() << endl;
    cout << _dist_map[0][0].size() << endl;

    for (int i = 0; i < _x_max; i++) {
        cout << "*************************" << endl;
        for (int j = 0; j < _y_max; j++) {
            printf("\n%d %d\n", i, j);
            for (int k = 0; k < _num_measurements; k++) {
                cout << _dist_map[i][j][k] << " ";
            }
            cout << endl;
        }
    }

}

void DistanceMap::print_occ_map() {
    cout << "Occupancy Map size" << endl;
    cout << _nav_occ_map.data.size() << endl;
}
