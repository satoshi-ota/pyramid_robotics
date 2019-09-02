# Set up the **pyramid robotics**

## Install

This package depends on third-party package.

- Rotors Simulation https://github.com/ethz-asl/rotors_simulator
- neo Simulation https://github.com/neobotix/neo_simulation
- ifopt https://github.com/ethz-adrl/ifopt

1. Install and initialize ROS melodic desktop full, additional ROS packages, catkin-tools, and wstool:
```
$ sudo apt-get update
$ sudo apt-get install ros-melodic-joy ros-melodic-octomap-ros python-wstool python-catkin-tools protobuf-compiler
$ sudo apt-get install libgeographic-dev ros-melodic-geographic-msgs  # Required for mavros.
# For melodic: sudo apt-get install libgeographiclib-dev
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/melodic/setup.bash
```

2. If you don't have ROS workspace yet you can do so by
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ wstool init
```

3. Get the **rotors_simulator** and additional dependencies
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ethz-asl/rotors_simulator.git
$ git clone https://github.com/ethz-asl/mav_comm.git
$ git clone https://github.com/ethz-asl/glog_catkin.git
$ git clone https://github.com/catkin/catkin_simple.git
$ sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
```

4. Get the **neo_simulator** and additional dependencies
```
$ git clone https://github.com/neobotix/neo_simulation.git
```

5. Get the **ifopt** and additional dependencies
```
$ git clone https://github.com/ethz-adrl/ifopt.git
$ sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev
```

6. Build your workspace with python_catkin_tools (therefore you need python_catkin_tools)
```
$ cd ~/catkin_ws/
$ catkin init  # If you haven't done this before.
$ catkin build
```

## Modified package **ifopt**

1. Add member function
~/catkin_ws/src/ifopt/ifopt_core/src
```cpp:problem.cc
void
Problem::Clear()
{
    constraints_.ClearComponents();
}
```
~/catkin_ws/src/ifopt/ifopt_core/src
```
void Clear();
```

2. Build
```
$ cd ~/catkin_ws/
$ catkin build
```

## Build the **pyramid_robotics**
FInally, we build pyramid_robotics!
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/satoshi-ota/pyramid_robotics.git
$ cd ..
$ catkin build
```
