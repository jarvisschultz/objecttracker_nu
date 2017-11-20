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
add_executable(nu_objecttracker src/nu_objecttracker.cpp)
add_executable(object_tracker_3d src/object_tracker_3d.cpp)
add_executable(orientation_calibration src/orientation_calibration.cpp)
add_executable(planar_sys_mass_tracker src/planar_sys_mass_tracker.cpp)
add_executable(planar_sys_robot_tracker src/planar_sys_robot_tracker.cpp)
add_executable(robot_tracker src/robot_tracker.cpp)


add_dependencies(multi_robot_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nu_objecttracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_tracker_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(orientation_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(planar_sys_mass_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(planar_sys_robot_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(multi_robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(nu_objecttracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(object_tracker_3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(orientation_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(planar_sys_mass_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(planar_sys_robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
