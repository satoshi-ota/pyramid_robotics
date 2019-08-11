cmake_minimum_required(VERSION 2.8.3)
project(pyramid_gazebo_plugins)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(gazebo REQUIRED)

catkin_package(
    FILES
    Tensions.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
