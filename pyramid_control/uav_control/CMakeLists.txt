cmake_minimum_required(VERSION 2.8.3)
project(uav_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES uav_control
    CATKIN_DEPENDS geometry_msgs roscpp
    DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
