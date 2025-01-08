cmake_minimum_required(VERSION 3.5)
project(mpc_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

# options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CasADi example" OFF)

## Find catkin macros and libraries
find_package(CasADi REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pluginlib
  message_generation
  move_base
  base_local_planner
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

## Find CasADi
find_package(CasADi REQUIRED)

include_directories(/usr/include)
link_directories(/usr/local/bin)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${CasADi_INCLUDE_DIRS})
link_directories(${CasADi_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MPCPlanner.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mpc_ros
   CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs move_base roscpp rospy std_msgs tf visualization_msgs pluginlib 
)

###########
## Build ##
###########

# MPC Local planner plugin
add_library(mpc_ros src/mpc_planner_ros.cpp src/mpc_planner.cpp)
target_link_libraries(mpc_ros ${CasADi_LIBRARIES} ${catkin_LIBRARIES})

#############
## Example ##
#############
if(BUILD_EXAMPLE)
    add_executable(casadi_example example/casadi_example.cpp)
    target_link_libraries(casadi_example ${CasADi_LIBRARIES} ${catkin_LIBRARIES})
endif(BUILD_EXAMPLE)

