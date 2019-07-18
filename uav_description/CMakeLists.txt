cmake_minimum_required(VERSION 2.8.3)
project(uav_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs)

catkin_package(
    CATKIN_DEPENDS
      gazebo_msgs
      geometry_msgs
      mav_msgs
      roscpp
      sensor_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(pyramid_hovering_example src/pyramid_hovering_example.cpp)
target_link_libraries(pyramid_hovering_example ${catkin_LIBRARIES})
add_dependencies(pyramid_hovering_example ${catkin_EXPORTED_TARGETS})
