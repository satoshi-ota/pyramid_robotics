cmake_minimum_required(VERSION 2.8.3)
project(uav_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    pyramid_msgs
    mav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
    LIBRARIES uav_control
    CATKIN_DEPENDS geometry_msgs roscpp pyramid_msgs mav_msgs
    DEPENDS Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(motor_speed_controller src/library/motor_speed_controller.cpp)

target_link_libraries(motor_speed_controller ${catkin_LIBRARIES})
add_dependencies(motor_speed_controller ${catkin_EXPORTED_TARGETS})

add_executable(motor_speed_controller_node src/nodes/motor_speed_controller_node.cpp)
add_dependencies(motor_speed_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_speed_controller_node
    motor_speed_controller ${catkin_LIBRARIES})
