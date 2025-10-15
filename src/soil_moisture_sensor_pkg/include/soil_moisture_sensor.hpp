#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <yaml-cpp/yaml.h>

#include <random>
#include <string>
#include <vector>
#include <limits>

struct Tree {
    std::string name;
    double x;
    double y;
    double z;
    double moisture;
};

class SoilMoistureSensor : public rclcpp::Node {
public:
    SoilMoistureSensor();

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateSensor();

    // Utility
    void loadTreeData(const std::string &filename);

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr moisture_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    std::vector<Tree> trees_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    // Random noise generator
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;
};
