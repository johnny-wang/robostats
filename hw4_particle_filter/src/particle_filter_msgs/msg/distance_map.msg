# Message definition for distance map.
# This map shows the different distances to obstacles of each angle of each x,y location

std_msgs/Header header
uint32 num_angles   # should be multiple of 360
map_x[] x           # stacked with map_x and ray_distance to get 3D array
