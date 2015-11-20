#include "particle_filter/DistanceMap.h"

#define PI 3.14159265

using namespace std;

static const int UNKNOWN = -1;
static const int WALL = 100;
static const int FREE = 0;

//#define DEBUG

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
      _angle_step_size(2*PI / num_degree)
{
    initialize(ocmap);
    create_distance_map();
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
    return _oc_map.size();
}

unsigned int DistanceMap::getNumDistCol() {
    return _oc_map[0].size();
}

/*********************************************************************/

/*
 * Reshape the occupancy map into 2D matrix of vectors so we can create a
 * distance map with it later. 
 */
void DistanceMap::initialize(nav_msgs::OccupancyGrid ocmap) {
    for (int row = 0; row < _row_dim; row++) {
        std::vector<int> col_data;
        for (int col = 0; col < _row_dim; col++) {
            int tmp = ocmap.data[_row_dim * row + col];
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
                    cout << "ray_dist: " << ray_dist << endl;
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

#ifdef DEBUG
    cout << "r: " << row << " c: " << col << " a: " << angle << endl;
#endif
    cur_col += step_col;
    cur_row += step_row;

    // Make sure within map boundaries
    while ((cur_col >= 0) && (cur_col < _row_dim) && (cur_row >= 0) && (cur_row < _row_dim)) {
        int map_val = getMapValue(int(cur_row), int(cur_col));

        if ((map_val == WALL)) {
            break;
        }
        cur_col += step_col;
        cur_row += step_row;
    }

#ifdef DEBUG
    cout << "cur_col: " << cur_col << " cur_row: " << cur_row << endl;
#endif

    float x_squared = (cur_col - float(col)) * (cur_col - float(col));
    float y_squared = (cur_row - float(row)) * (cur_row - float(row));

    return sqrt(x_squared + y_squared);
}
