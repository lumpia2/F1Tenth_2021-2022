# Pitt RAS F1Tenth
Official Repository for Pitt RAS F1Tenth 2021-2022 Team

To get "up to speed", take a look at our [Getting Started](https://docs.google.com/document/d/1MPcZuDyK1n8SSpPkCYEtRkjRHpZaNV0-Qj5Cd3Ra4kE/edit?usp=sharing) Guide

This repository follows the resources found at https://f1tenth.org

## Contributing

1. Fork and clone the repository.
2. Make a branch relative to what you're working on.
3. Once you've developed and tested your changes create PR with a description of your changes and any relevant tests.

NOTE: You will need to add any IDE config files to git exclude before landing any changes.

Try and keep your coding style consistent.

## Setup

Update your local packages if you haven't alraedy by running ```$ sudo apt-get update```
You will need ROS Noetic installed and sourced before using this repo. Installation guide can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

1. Install ROS dependencies
```
$ sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server
```
2. Clone the repository
```
$ git clone https://github.com/Pitt-RAS/ras-f1tenth
```
3. Initialize submodules
```
$ cd ras-f1tenth
$ git submodule update --init --recursive
```
4. Install Catkin Tools if you haven't already installed them
```
$ sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python3-catkin-tools
```
5. Build the modules with catkin
```
$ cd catkin_ws ## This should be the workspace located in the cloned repository
$ catkin build
```
You may need to source the workspace and rebuild the project
```
$ source devel/setup.bash
$ catkin build
```
### Running the Simulator
Run any of the launch files found in [this](https://github.com/Pitt-RAS/F1Tenth_2021-2022/tree/main/catkin_ws/src/f1tenth_modules/launch/sims) directory.
```
$ roslaunch f1tenth_modules <launch file>
```


## Introduction to ROS - Part 1

This project uses the ROS 1 framework to connect several nodes and states together. Fundamental knowledge of
ROS is necessary to contribute.

Intro [slides](https://docs.google.com/presentation/d/10-BS7uOYaSVuBZqPPlhUMrAvJ8rqE7bGuX3cjUwzc6Q/edit?usp=sharing).

1. Configuring a ROS environment
2. Creating ROS packages
3. Building a ROS package
4. Using ROS Nodes
5. Using ROS Topics

Online tutortials can be found [here](http://wiki.ros.org/ROS/Tutorials).

## Introduction to ROS - Part 2

ROS Tutorials used:

12. Writing a Simple Publisher and Subscriber
13. Examining the Publisher and Subscriber

Online tutortials can be found [here](http://wiki.ros.org/ROS/Tutorials).


## Resources

- [Intel Realsense d435](https://www.intelrealsense.com/depth-camera-d435/).
- Adam Johnson's perception [slides](https://docs.google.com/presentation/d/1OpZpCFbR4MlRBM9_tBVKmbXkI15ABNUqcTTLkb2kKVQ/edit?usp=sharing).
- F1tenth car build [link](https://f1tenth.org/build.html)
- F1tenth software stack modules [link](https://f1tenth.org/learn.html)


