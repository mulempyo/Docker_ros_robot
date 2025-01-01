cmake_minimum_required(VERSION 3.0.2)
project(costmap_update)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
)

catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES costmap_update
   CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/costmap_update.cpp
)

add_executable(update_map src/costmap_update.cpp)

target_link_libraries(update_map
  ${catkin_LIBRARIES}
)

add_dependencies(update_map ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})

