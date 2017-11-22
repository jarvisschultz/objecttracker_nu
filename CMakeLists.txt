cmake_minimum_required(VERSION 2.8.3)
project(nodelet_pcl_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  tf
  tf_conversions
  puppeteer_msgs
  geometry_msgs)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_extractor src/cluster_extractor.cpp)
add_dependencies(cluster_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cluster_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
