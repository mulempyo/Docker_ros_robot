cmake_minimum_required(VERSION 3.5)
project(mpc_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

# options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CasADi example" OFF)

## Find catkin macros and libraries
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

## Include CasADi headers and libraries manually
set(CasADi_INCLUDE_DIRS /usr/local/include)
set(CasADi_LIBRARIES /usr/local/lib/libcasadi.so)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${catkin_INCLUDE_DIRS} ${CasADi_INCLUDE_DIRS})
link_directories(/usr/local/lib)

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
