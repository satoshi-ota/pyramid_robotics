cmake_minimum_required(VERSION 2.8.3)
project(tether_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    mav_msgs
    rosbag
    roscpp
    sensor_msgs
    cmake_modules
    roslib
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
    LIBRARIES tether_control
    CATKIN_DEPENDS geometry_msgs rosbag roscpp sensor_msgs roslib
    DEPENDS Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(simple_tensions_controller
    src/library/simple_tensions_controller.cpp
)

target_link_libraries(simple_tensions_controller ${catkin_LIBRARIES})
add_dependencies(simple_tensions_controller ${catkin_EXPORTED_TARGETS})

add_executable(simple_tensions_controller_node src/nodes/simple_tensions_controller_node.cpp)
add_dependencies(simple_tensions_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_tensions_controller_node
    simple_tensions_controller ${catkin_LIBRARIES})
