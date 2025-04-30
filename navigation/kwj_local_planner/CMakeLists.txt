cmake_minimum_required(VERSION 3.5)
project(kwj_local_planner)

add_compile_options(-std=c++11 -O3)

option(BUILD_EXAMPLE "Whether or not building the CasADi example" OFF)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  move_base
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

set(CasADi_INCLUDE_DIRS /usr/local/include)
set(CasADi_LIBRARIES /usr/local/lib/libcasadi.so)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${catkin_INCLUDE_DIRS} ${CasADi_INCLUDE_DIRS})
link_directories(/usr/local/lib)

generate_dynamic_reconfigure_options(
    cfg/KWJPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kwj_local_planner
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs move_base nav_msgs pluginlib roscpp rospy std_msgs tf visualization_msgs
)


add_library(kwj_local_planner src/kwj_local_ros.cpp src/kwj.cpp)
target_link_libraries(kwj_local_planner ${CasADi_LIBRARIES} ${catkin_LIBRARIES})

if(BUILD_EXAMPLE)
    add_executable(casadi_example example/casadi_example.cpp)
    target_link_libraries(casadi_example ${CasADi_LIBRARIES} ${catkin_LIBRARIES})
endif(BUILD_EXAMPLE)


