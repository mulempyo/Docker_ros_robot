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

# Torch 설정
set(Torch_DIR "/home/user/libtorch/share/cmake/Torch")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
find_package(Torch REQUIRED)

# Eigen3 설정
find_package(Eigen3 REQUIRED)

# Catkin 패키지 설정
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
  ${EIGEN3_INCLUDE_DIRS}  # 오류 수정
  ${TORCH_INCLUDE_DIRS}   # Torch 경로 추가
)
link_directories(${TORCH_LIBRARY_DIRS})
# 라이브러리 추가
add_library(${PROJECT_NAME} src/dwa_planner_ros.cpp src/dwa_planner.cpp)

# Torch를 포함한 라이브러리 연결
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TORCH_LIBRARIES})

