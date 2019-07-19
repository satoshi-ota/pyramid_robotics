cmake_minimum_required(VERSION 2.8.3)
project(ugv_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(pyramid_ugv_teleop_key src/pyramid_ugv_teleop_key.cpp)
target_link_libraries(pyramid_ugv_teleop_key ${catkin_LIBRARIES})
add_dependencies(pyramid_ugv_teleop_key ${catkin_EXPORTED_TARGETS})
