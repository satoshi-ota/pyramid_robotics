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
    mav_msgs
    dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SystemCommander.cfg
)

catkin_package(
    INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
    LIBRARIES pyramid_central
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs trajectory_msgs pyramid_msgs mav_msgs
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
        src/library/system_reconfiguration.cpp
        src/library/sliding_mode_controller.cpp)

target_link_libraries(system_commander ${catkin_LIBRARIES})
add_dependencies(system_commander ${catkin_EXPORTED_TARGETS})

add_executable(system_commander_node src/nodes/system_commander_node.cpp)
add_dependencies(system_commander_node pyramid_central_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_commander_node system_commander ${catkin_LIBRARIES})

add_executable(trajectory_commander_node src/nodes/trajectory_commander_node.cpp)
add_dependencies(trajectory_commander_node pyramid_central_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_commander_node system_commander ${catkin_LIBRARIES})

add_executable(sliding_mode_control_node src/nodes/sliding_mode_control_node.cpp)
add_dependencies(sliding_mode_control_node pyramid_central_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(sliding_mode_control_node system_commander ${catkin_LIBRARIES})
