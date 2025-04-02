cmake_minimum_required(VERSION 3.0.2)
project(detect_object)

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)  
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  gb_visual_detection_3d_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS costmap_2d gb_visual_detection_3d_msgs geometry_msgs roscpp std_msgs tf2 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/cuda-11.1/include
  ${CUDA_INCLUDE_DIRS}
)

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_75 -Xcompiler -pthread -std=c++14")

cuda_add_library(${PROJECT_NAME}_kernel SHARED src/detect_object_kernel.cu)
set_target_properties(${PROJECT_NAME}_kernel PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${PROJECT_NAME}_kernel ${CUDA_LIBRARIES} -lpthread)

add_library(${PROJECT_NAME} src/detect_object.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(detect_node src/detect_object.cpp)
target_link_libraries(detect_node 
    ${PROJECT_NAME} 
    ${PROJECT_NAME}_kernel 
    ${catkin_LIBRARIES} 
    ${CUDA_LIBRARIES}  
    Threads::Threads 
    -lpthread
)

add_executable(bounding_boxes_3d src/detect_object.cpp)
target_link_libraries(bounding_boxes_3d 
    ${PROJECT_NAME} 
    ${PROJECT_NAME}_kernel 
    ${catkin_LIBRARIES} 
    ${CUDA_LIBRARIES}  
    Threads::Threads 
    -lpthread
)

