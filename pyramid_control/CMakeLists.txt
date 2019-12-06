cmake_minimum_required(VERSION 2.8.3)
project(pyramid_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
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
  cfg/SlidingModeController.cfg
  cfg/TrajectoryGenerator.cfg
)

catkin_package(
    INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
    LIBRARIES uav_control
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs trajectory_msgs pyramid_msgs mav_msgs
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(pyramid_control
    src/library/sliding_mode_controller.cpp)

target_link_libraries(pyramid_control ${catkin_LIBRARIES})
add_dependencies(pyramid_control ${catkin_EXPORTED_TARGETS})

# add_executable(motor_speed_controller_node src/nodes/motor_speed_controller_node.cpp)
# add_dependencies(motor_speed_controller_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(motor_speed_controller_node
#     motor_speed_controller ${catkin_LIBRARIES})
add_executable(sliding_mode_control_node src/nodes/sliding_mode_control_node.cpp)
add_dependencies(sliding_mode_control_node pyramid_control_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(sliding_mode_control_node pyramid_control ${catkin_LIBRARIES})

add_executable(trajectory_commander_node src/nodes/trajectory_commander_node.cpp)
add_dependencies(trajectory_commander_node pyramid_control_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_commander_node pyramid_control ${catkin_LIBRARIES})
