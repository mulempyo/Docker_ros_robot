cmake_minimum_required(VERSION 3.18)
project(dwa_planner_ros)

add_compile_options(-std=c++17)

set(Torch_DIR "/home/user/libtorch/share/cmake/Torch/")

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
)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} src/dwa_planner_ros.cpp src/dwa_planner.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TORCH_LIBRARIES} ${Boost_LIBRARIES})

