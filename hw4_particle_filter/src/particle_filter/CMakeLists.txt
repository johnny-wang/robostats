cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  roslib
  particle_filter_msgs
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES particle_filter
  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs roslib message_runtime
  DEPENDS system_lib message_runtime 
  DEPENDS Eigen
)

###########
## Build ##
###########

set( PF_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include" )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${PF_INCLUDE_DIRECTORIES}
)

## Declare a C++ library
# add_library(particle_filter
#   src/${PROJECT_NAME}/particle_filter.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(load_map_node src/load_map.cpp src/DistanceMap.cpp)
add_executable(load_data_node src/load_data.cpp)
add_executable(main_node src/main.cpp src/DistanceMap.cpp src/ParticleFilter.cpp src/DataLoader.cpp)
#add_executable(test_node src/test_cb.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(load_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(load_map_node
  ${catkin_LIBRARIES} ${PF_INCLUDE_DIRECTORIES}
)
target_link_libraries(load_data_node
  ${catkin_LIBRARIES}
)
target_link_libraries(main_node
  ${catkin_LIBRARIES} ${PF_INCLUDE_DIRECTORIES} ${Eigen_LIBRARIES}
)
#target_link_libraries(test_node
#  ${catkin_LIBRARIES} ${PF_INCLUDE_DIRECTORIES} 
#)

## Compile with RELEASE flag
set_target_properties(load_map_node PROPERTIES COMPILE_FLAGS "-std=c++11 -DCMAKE_BUILD_TYPE=Release")
set_target_properties(load_data_node PROPERTIES COMPILE_FLAGS "-std=c++11 -DCMAKE_BUILD_TYPE=Release")
set_target_properties(main_node PROPERTIES COMPILE_FLAGS "-std=c++11 -DCMAKE_BUILD_TYPE=Release")
#set_target_properties(test_node PROPERTIES COMPILE_FLAGS "-std=c++11 -DCMAKE_BUILD_TYPE=Release")

