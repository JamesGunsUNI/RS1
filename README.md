#RS1

 ros2 launch 41068_ignition_bringup 41068_ignition.launch.py     slam:=false     nav2:=true     rviz:=true     world:=large_demo     map:=/home/ben/ros_ws/src/RS1/maps/my_map.yaml


# 41068 Ignition Bringup

Bringup for *41068 Robotics Studio I*. Launches a Husky robot in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

Worlds are build from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).

## Installation

Dependencies:

* Install Gazebo
  ```bash
  sudo apt-get update && sudo apt-get install wget
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update && sudo apt-get install ignition-fortress
  ```
* Install development tools and robot localisation
  ```bash
  sudo apt install ros-dev-tools ros-humble-robot-localization
  sudo apt install ros-humble-ros-ign ros-humble-ros-ign-interfaces
  sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
  ```
* Make sure that your installation is up to date.
  sudo apt upgrade
  sudo apt update
  ```  

Now install this package:
* Copy this package to the `src` directory in this workspace
  ```bash

  colcon build --packages-select ignition_bringup_41068 --symlink-install

  source ~/.bashrc

  ros2 launch ignition_bringup_41068 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
  ```
