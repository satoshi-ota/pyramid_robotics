cmake_minimum_required(VERSION 2.8.3)
project(pyramid_evaluation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  pyramid_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/DisturbanceGenerator.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pyramid_evaluation
    CATKIN_DEPENDS roscpp geometry_msgs pyramid_msgs
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

# add_library(pyramid_evaluation)
#
# target_link_libraries(pyramid_evaluation ${catkin_LIBRARIES})
# add_dependencies(pyramid_evaluation ${catkin_EXPORTED_TARGETS})

add_executable(disturbance_gen_node src/nodes/disturbance_gen_node.cpp)
add_dependencies(disturbance_gen_node pyramid_evaluation_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(disturbance_gen_node ${catkin_LIBRARIES})
