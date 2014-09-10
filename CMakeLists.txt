# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(objecttracker_nu)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros tf tf_conversions puppeteer_msgs geometry_msgs)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp pcl pcl_ros tf tf_conversions puppeteer_msgs geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
    )

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


add_dependencies(multi_robot_tracker objecttracker_nu_gencpp)
add_dependencies(nu_objecttracker objecttracker_nu_gencpp)
add_dependencies(object_tracker_3d objecttracker_nu_gencpp)
add_dependencies(orientation_calibration objecttracker_nu_gencpp)
add_dependencies(planar_sys_mass_tracker objecttracker_nu_gencpp)
add_dependencies(planar_sys_robot_tracker objecttracker_nu_gencpp)
add_dependencies(robot_tracker objecttracker_nu_gencpp)


target_link_libraries(multi_robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(nu_objecttracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(object_tracker_3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(orientation_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(planar_sys_mass_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(planar_sys_robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(robot_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
