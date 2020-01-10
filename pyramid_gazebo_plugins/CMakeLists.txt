cmake_minimum_required(VERSION 2.8.3)
project(pyramid_gazebo_plugins)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  gazebo_ros
  roslib
  pyramid_msgs
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
INCLUDE_DIRS include ${GAZEBO_LIBRARY_DIRS}
LIBRARIES pyramid_gazebo_plugins
CATKIN_DEPENDS roscpp roslib gazebo_ros sensor_msgs geometry_msgs pyramid_msgs std_msgs
DEPENDS
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# add_library(gazebo_tensions_plugin
#     src/gazebo_tensions_plugin.cpp
#     include/pyramid_gazebo_plugins/gazebo_tensions_plugin.h)
# target_link_libraries(gazebo_tensions_plugin
#     ${catkin_LIBRARIES}
#     ${GAZEBO_LIBRARIES}
#     ${roscpp_LIBRARIES})
# add_dependencies(gazebo_tensions_plugin
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})

add_library(gazebo_external_force_plugin
    src/gazebo_external_force_plugin.cpp
    include/pyramid_gazebo_plugins/gazebo_external_force_plugin.h)
target_link_libraries(gazebo_external_force_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES})
add_dependencies(gazebo_external_force_plugin
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

add_library(gazebo_disturbance_plugin
    src/gazebo_disturbance_plugin.cpp
    include/pyramid_gazebo_plugins/gazebo_disturbance_plugin.h)
target_link_libraries(gazebo_disturbance_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES})
add_dependencies(gazebo_disturbance_plugin
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

add_library(gazebo_psuedo_tether_plugin
    src/gazebo_psuedo_tether_plugin.cpp
    include/pyramid_gazebo_plugins/gazebo_psuedo_tether_plugin.h)
target_link_libraries(gazebo_psuedo_tether_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES})
add_dependencies(gazebo_psuedo_tether_plugin
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

add_library(gazebo_winch_plugin
    src/gazebo_winch_plugin.cpp
include/pyramid_gazebo_plugins/gazebo_winch_plugin.h)
target_link_libraries(gazebo_winch_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES})
add_dependencies(gazebo_winch_plugin
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
