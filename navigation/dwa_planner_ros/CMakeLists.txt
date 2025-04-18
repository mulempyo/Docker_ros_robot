cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner_ros)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  tf2
  move_base
  astar_global_planner
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_planner_ros
  CATKIN_DEPENDS 
    base_local_planner 
    costmap_2d
    nav_core 
    nav_msgs 
    pluginlib 
    roscpp 
    tf 
    tf2
    move_base
    astar_global_planner
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  
)
link_directories(${TORCH_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/dwa_planner_ros.cpp src/dwa_planner.cpp)

# Torch를 포함한 라이브러리 연결
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

