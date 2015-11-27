#include "particle_filter/DistanceMap.h"

#define PI 3.14159265

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

//#define DEBUG
//#define DEBUG_INIT
//#define DEBUG_OCMAP
//#define DEBUG_DATA
//#define DEBUG_SAVE
//#define DEBUG_DIST
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
    : _x_max(ocmap.info.width),
      _y_max(ocmap.info.height),
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

int DistanceMap::getResolution() {
    return _resolution;
}

int DistanceMap::getXSize() {
    return _x_max;
}

int DistanceMap::getYSize() {
    return _y_max;
}

int DistanceMap::getNumMeasurements() {
    return _num_measurements;
}

/*
 * Input is map min to max. i.e. 0 to 800.
 */
int DistanceMap::getMapValue(int x, int y) {
    return _oc_map[y][x];
}

/*
 * INPUT:
 *  x, y = the positions of the particle.
 *  theta = the orientation of the particle (in RADIANS!!!)
 */
float DistanceMap::getDistValue(float x, float y, float theta) {
    // Addition part is to help round to nearest angle
    int multiplier = (theta + (_angle_step_size/2)) / _angle_step_size;
#ifdef DEBUG_DIST
    printf("mod of %f %f: %d\n", theta, _angle_step_size, multiplier);
#endif
    // Need to convert to map's coordinate which is 10cm x 10cm per cell
    //x /= _resolution;
    //y /= _resolution;
    return _dist_map[y][x][multiplier];
}

sensor_msgs::LaserScan DistanceMap::getLaserScans(float x, float y, float theta) {
    // store laser scans
    sensor_msgs::LaserScan my_laser;
    my_laser.header.frame_id = "laser_frame";
    // 0 to 179 because there are 180 readings
    my_laser.angle_min = 0;
    my_laser.angle_max = 179 * PI / 180; // radians
    my_laser.angle_increment = PI / 180;
    my_laser.range_min = 0.0;
    my_laser.range_max = 8250;  // in cm, heuristically 8183 cm
    my_laser.ranges.resize(180);

    for (int i=0; i<180; i++) {
        my_laser.ranges[i] = getDistValue(x, y, theta);
    }

    return my_laser;
}

std::vector<float> DistanceMap::getDistVal(float x, float y) {
    return _dist_map[y][x];
}

int DistanceMap::getNumDistX() {
    return _dist_map.size();
}

int DistanceMap::getNumDistY() {
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

    boost::timer t;

    // Load binary file
    data_in.open(fname.c_str(), std::ios::binary);

    if (data_in.is_open()) {
        std::vector<float> all_in;
        data_in.unsetf(std::ios::skipws);
        std::streampos file_size;
        data_in.seekg(0, std::ios::end);
        file_size = data_in.tellg();
        data_in.seekg(0, std::ios::beg);
        
        size_t size_of_buffer = file_size / sizeof(float);
        float* file_buffer = new float[size_of_buffer];

#ifdef DEBUG_DATA
        cout << "file size: " << file_size << endl;
        cout << "buffer: " << size_of_buffer << endl;
#endif
        
        data_in.read(reinterpret_cast<char*>(file_buffer), file_size);
        
        all_in = std::vector<float>(file_buffer, file_buffer + size_of_buffer);
        
        free(file_buffer);

        _dist_map.clear();
        _dist_map.resize(_y_max);

        // Convert all data to _dist_map format
        for (int i = 0; i < _y_max; i++) {
            std::vector<std::vector<float>> tmp(_x_max);
            for (int j = 0; j < _x_max; j++) {
                int row_val = i * _num_measurements * _x_max;
                int col_val = j * _num_measurements;
                int last_col = ((j+1) * _num_measurements);

                std::vector<float>::const_iterator first = all_in.begin() + row_val + col_val;
                std::vector<float>::const_iterator last  = all_in.begin() + row_val + last_col;
                std::vector<float> dist(first, last);

#ifdef DEBUG_DATA
                int start = row_val + col_val;
                int end = row_val + last_col;
                int diff = end - start;
                printf("%d %d %d %d\n", start, end, diff, last_col);
                bool zeros = std::all_of(dist.begin(), dist.end(), [](int i) { return i==0; });
                //if (!zeros) {
                    cout << "---------------------------------------------------------" << endl;
                    for (int z = 0; z < dist.size(); z++) {
                        cout << dist[z] << " ";
                    }
                    cout << dist.size() << endl;
                    cin.get();
                //}
#endif
                tmp[j] = dist;
            }
#ifdef DEBUG_DATA
            cout << "tmp size: " << tmp.size() << endl;
#endif
            _dist_map[i] = tmp;
        }
    } else {
    // file doesn't exist, create the distance map then save it.
        cout << "File doesn't exist: " << fname << endl;
        create_distance_map();
        saveDistMap(filename);
    }

/*
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
*/

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
                _x_max = _nav_occ_map.info.width;
                _y_max = _nav_occ_map.info.height;
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
                    if (input >= 0) { 
                        input = 1-input;    // b/c the file has occupancy backwards
                        // 1 = occupied; 0 = not occupied
                        if (input > 0.95) {   // occupied space
                            _nav_occ_map.data[map_data_counter++] = 100;// // map takes 0-100
                        } else if ((input <= 0.95) && (input > 0.05)) {
                            // probability of 50 that it's occupied
                            _nav_occ_map.data[map_data_counter++] = input * 100;// // map takes 0-100
                        } else {
                            // free space
                            _nav_occ_map.data[map_data_counter++] = 0;// // map takes 0-100
                        }
                        // No tier-ing
                        //_nav_occ_map.data[map_data_counter++] = input * 100;// // map takes 0-100
                    } else {
                        _nav_occ_map.data[map_data_counter++] = -1;    // -1: unknown
                    }
#ifdef DEBUG_OCMAP
                    if ((map_data_counter % _x_max) == 0) {
                        printf("%3d\n", (int)_nav_occ_map.data[map_data_counter-1]); 
                    } else {
                        printf("%3d ", (int)_nav_occ_map.data[map_data_counter-1]); 
                    }
#endif
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
#ifdef DEBUG_SAVE
    boost::timer t;
#endif

//    ofstream data_out;
    filename = _package_path + "/data/map/" + filename;
//    data_out.open(filename.c_str());
    std::ofstream data_out(filename.c_str(), std::ios::binary | std::ios::out);

    cout << "Saving distance map" << endl;

    if (!data_out.is_open()) {
        cerr << "ERROR: Unable to open file: " << filename << endl;
        return;
    }

    // Write binary to file
    for (int y = 0; y < _dist_map.size(); y++) {
        for (int x = 0; x < _dist_map[y].size(); x++) {
            const char* buffer = reinterpret_cast<const char*>(&_dist_map[y][x][0]);
            data_out.write(buffer, _dist_map[y][x].size() * sizeof(float));
        }
    }
/*
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
*/

#ifdef DEBUG_SAVE
    std::cout << "time: " << t.elapsed() << std::endl;
//    std::cout << "lines: " << lines_written << std::endl;
#endif
    std::cout << "Distance map saved to " << filename << std::endl;
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
#ifdef DEBUG_INIT
    cout << "Initializing internal occupancy map" << endl;
    printf("x: %d y: %d\n", _x_max, _y_max);
#endif

    for (int y = 0; y < _y_max; y++) {
        std::vector<int> x_data;
        for (int x = 0; x < _x_max; x++) {
            int tmp = _nav_occ_map.data[_x_max * y + x];
//cout << tmp << " " << _x_max * y + x << " " << x << " " << y;
//cin.get();
            x_data.push_back(tmp);
        }
        _oc_map.push_back(x_data);
    }

#ifdef DEBUG_INIT
    for (int i = 0; i < _y_max; i++) {
        for (int j = 0; j < _x_max; j++) {
            printf("%3d ", _oc_map[i][j]);
        }
        cout << endl;
    }
#endif
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

    cout << "Creating new distance map" << endl;

#ifdef DEBUG_DIST
    boost::timer t;
#endif

    _dist_map.clear();
    int lines_written = 0;

    for (int y = 0; y < _y_max; y++) {
        std::vector< std::vector<float> > x_data;
        //y_data.reserve(_y_max * _x_max);   // pre-allocate

        for (int x = 0; y < _x_max; x++) {

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
            x_data.push_back(dist);
        }
        _dist_map.push_back(x_data);
    }
#ifdef DEBUG_DIST
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

    float step_x = _ray_step_size * cos(angle);
    float step_y = _ray_step_size * sin(angle);

#ifdef DEBUG_1
    cout << "r: " << y << " c: " << x << " a: " << angle << endl;
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

    for (int i = 0; i < _y_max; i++) {
        cout << "*************************" << endl;
        for (int j = 0; j < _x_max; j++) {
            printf("\n%d %d\n", i, j);
            for (int k = 0; k < _num_measurements; k++) {
                cout << _dist_map[j][i][k] << " ";
            }
            cout << endl;
        }
    }

}

void DistanceMap::print_occ_map() {
    cout << "Occupancy Map size" << endl;
    cout << _nav_occ_map.data.size() << endl;
}

void DistanceMap::checkMaps() {
    int free_space = 0;
    std::vector<std::string> bad_coords;

    for (int y = 0; y < _y_max; y++) {
        for (int x = 0; x < _x_max; x++) {
            std::vector<float> values = getDistVal(x, y);

            if ((getMapValue(x, y) >= FREE) && (getMapValue(x, y) < WALL)) {
                free_space++;
            }

            bool zeros = std::all_of(values.begin(), values.end(), [](float i) { return i==0; });
            if (((getMapValue(x, y) == UNKNOWN) || (getMapValue(x, y) == WALL)) && (zeros)) {
                printf("GOOD x: %d y: %d\n", x, y);
            } else if (((getMapValue(x, y) >= FREE) && (getMapValue(x, y) < WALL)) && (!zeros)) {
                printf("GOOD x: %d y: %d\n", x, y);
            } else {
                printf("BAD!!! x: %d y: %d %d\n", x, y, zeros);
                cout << getMapValue(x,y) << endl;
                std::ostringstream oss;
                oss << y << " " << x << " " << getMapValue(x,y);
                bad_coords.push_back(oss.str());
            }
            cout << (int)getMapValue(x,y) << endl;
            //for (int i=0; i<360; i++) {
            //    cout << values[i] << " ";
            //}
            //cout << endl;
            //cout << zeros << endl;
            //cin.get();
            cout << "**********************************************" << endl;
        }
    }

    cout << "--------------------------------------------------" << endl;
    for (int i = 0; i < bad_coords.size(); i++) {
        cout << bad_coords[i] << endl;
    }
    cout << "--------------------------------------------------" << endl;
    cout << "num bad: " << bad_coords.size() << endl;
    cout << "free space: " << free_space << endl;

}
