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
  /usr/local/cuda-10.2/include
  ${CUDA_INCLUDE_DIRS}
)

# CUDA 컴파일러 플래그 수정
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_75 -Xcompiler -pthread -std=c++14")

# CUDA 라이브러리 추가
cuda_add_library(${PROJECT_NAME}_kernel SHARED src/detect_object_kernel.cu)
set_target_properties(${PROJECT_NAME}_kernel PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${PROJECT_NAME}_kernel ${CUDA_LIBRARIES} -lpthread)

# 메인 라이브러리
add_library(${PROJECT_NAME} src/detect_object.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# 실행 파일: detect_node
add_executable(detect_node src/detect_object.cpp)
target_link_libraries(detect_node 
    ${PROJECT_NAME} 
    ${PROJECT_NAME}_kernel 
    ${catkin_LIBRARIES} 
    ${CUDA_LIBRARIES}  
    Threads::Threads 
    -lpthread
)

# 실행 파일: bounding_boxes_3d (필요하면 유지)
add_executable(bounding_boxes_3d src/detect_object.cpp)
target_link_libraries(bounding_boxes_3d 
    ${PROJECT_NAME} 
    ${PROJECT_NAME}_kernel 
    ${catkin_LIBRARIES} 
    ${CUDA_LIBRARIES}  
    Threads::Threads 
    -lpthread
)

