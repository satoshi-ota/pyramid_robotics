cmake_minimum_required(VERSION 2.8.3)
project(pyramid_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    trajectory_msgs
    pyramid_msgs
    mav_msgs
    ifopt
    dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SystemParameters.cfg
  cfg/TrajectoryGenerator.cfg
)

catkin_package(
    INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
    LIBRARIES pyramid_control
    CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs roscpp sensor_msgs trajectory_msgs pyramid_msgs mav_msgs
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(pyramid_control
    src/library/sliding_mode_controller.cpp
    src/library/actuator_controller.cpp
    src/library/observer.cpp)

target_link_libraries(pyramid_control ${catkin_LIBRARIES})
add_dependencies(pyramid_control ${catkin_EXPORTED_TARGETS})

add_executable(commander_node src/nodes/commander_node.cpp)
add_dependencies(commander_node pyramid_control_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(commander_node pyramid_control ${catkin_LIBRARIES})

add_executable(trajectory_commander_node src/nodes/trajectory_commander_node.cpp)
add_dependencies(trajectory_commander_node pyramid_control_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_commander_node pyramid_control ${catkin_LIBRARIES})
