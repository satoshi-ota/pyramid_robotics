cmake_minimum_required(VERSION 2.8.3)
project(pyramid_msgs)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Tensions.msg
  Positions.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs std_msgs sensor_msgs
)
