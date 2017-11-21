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

add_executable(multi_robot_tracker src/multi_robot_tracker.cpp)
add_dependencies(multi_robot_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
