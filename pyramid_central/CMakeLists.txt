cmake_minimum_required(VERSION 2.8.3)
project(pyramid_central)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    trajectory_msgs
    pyramid_msgs
    dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SystemCommander.cfg
)

catkin_package(
    INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
    LIBRARIES pyramid_central
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs trajectory_msgs pyramid_msgs
    DEPENDS Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(system_commander
        src/library/system_commander.cpp
        src/library/system_reconfiguration.cpp)

target_link_libraries(system_commander ${catkin_LIBRARIES})
add_dependencies(system_commander ${catkin_EXPORTED_TARGETS})

add_executable(system_commander_node src/nodes/system_commander_node.cpp)
add_dependencies(system_commander_node ${catkin_EXPORTED_TARGETS})
add_dependencies(system_commander_node pyramid_central_gencfg)
target_link_libraries(system_commander_node system_commander ${catkin_LIBRARIES})
