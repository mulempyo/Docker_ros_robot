cmake_minimum_required(VERSION 3.0.2)
project(graph_slam)

add_definitions(-std=c++14 -O3)

set(G2O_INCLUDE_DIRS /usr/local/include)
set(G2O_CORE_LIBRARY /usr/local/lib/libg2o_core.so)
set(G2O_STUFF_LIBRARY /usr/local/lib/libg2o_stuff.so)
set(G2O_SOLVER_PCG /usr/local/lib/libg2o_solver_pcg.so)
set(G2O_SOLVER_CSPARSE /usr/local/lib/libg2o_solver_csparse.so)
set(G2O_SOLVER_CHOLMOD /usr/local/lib/libg2o_solver_cholmod.so)
set(G2O_TYPES_SLAM2D /usr/local/lib/libg2o_types_slam2d.so)
set(G2O_TYPES_SLAM2D_ADDONS /usr/local/lib/libg2o_types_slam2d_addons.so)

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf
  nodelet
  pluginlib
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_slam
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp sensor_msgs tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  /usr/local/cuda-10.2/include
  ${CUDA_INCLUDE_DIRS}
)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC" CACHE STRING "nvcc flags" FORCE)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
cuda_add_library(${PROJECT_NAME}_kernel SHARED src/icp_cuda.cu)
set_target_properties(${PROJECT_NAME}_kernel PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${PROJECT_NAME}_kernel ${CUDA_LIBRARIES})

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_library(graph_slam_nodelet
  src/slam_algorithm.cpp 
  src/robust_kernel_io.cpp 
  src/slam.cpp 
  src/nodelet.cpp
  src/icp_wrapper.cpp
)
target_link_libraries(graph_slam_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME}_kernel ${CUDA_LIBRARIES})

add_executable(graph_slam_node
  src/slam_algorithm.cpp 
  src/robust_kernel_io.cpp 
  src/slam.cpp 
  src/main.cpp
  src/icp_wrapper.cpp
)

target_link_libraries(graph_slam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM2D_ADDONS}
  ${CUDA_LIBRARIES}
  ${PROJECT_NAME}_kernel
)
add_dependencies(graph_slam_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_kernel)

install(TARGETS graph_slam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
