cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(convex_hull)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)
find_package(PCL 1.8 REQUIRED)
find_package(base)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(convex_hull src/convex_hull.cpp)

add_dependencies(convex_hull ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})


target_link_libraries(convex_hull
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
  ${PCL_LIBRARIES}
)