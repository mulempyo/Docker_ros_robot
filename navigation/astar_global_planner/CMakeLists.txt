cmake_minimum_required(VERSION 3.0.2)
project(astar_global_planner)

add_compile_options(-std=c++11)

find_package(CUDA REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_core
  pluginlib
  roscpp
  rospy
  std_msgs
  tf
  base_local_planner
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES astar_global_planner
   CATKIN_DEPENDS nav_core pluginlib roscpp rospy std_msgs tf base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/cuda-10.2/include
  ${CUDA_INCLUDE_DIRS}
)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC" CACHE STRING "nvcc flags" FORCE)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
cuda_add_library(${PROJECT_NAME}_kernel SHARED src/astar.cu)
set_target_properties(${PROJECT_NAME}_kernel PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${PROJECT_NAME}_kernel ${CUDA_LIBRARIES})

 add_library(astar_global_planner
   src/astar.cpp
 )

target_link_libraries(astar_global_planner
  ${catkin_LIBRARIES} ${PROJECT_NAME}_kernel ${CUDA_LIBRARIES}
)

