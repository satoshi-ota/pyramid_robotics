cmake_minimum_required(VERSION 2.8.3)
project(pyramid_msgs)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
    FILES
    Tensions.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs message_runtime
)
