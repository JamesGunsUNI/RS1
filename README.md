# 41068 Ignition Bringup

Bringup for *41068 Robotics Studio I*. Launches a Husky robot in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

Worlds are build from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).

## Installation

First install some dependencies:

* If you haven't already, install ROS2 Humble. Follow the instructions here: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
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
* Make sure that your installation is up to date. This is particularly important if you installed ROS a long time ago, such as in another subject. If you get errors here, make sure to resolve these before continuing.
  ```bash
  sudo apt upgrade
  sudo apt update
  ```  

Now install this package:

  ```bash
  source /opt/ros/humble/setup.bash
  cd ros_ws
  colcon build --packages-select 41068_ignition_bringup --symlink-install
  
  source ~/ros_ws/install/setup.bash

* And similarly, the larger world, and with SLAM and navigation:
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo #map:=/maps/my_map.yaml
  ```