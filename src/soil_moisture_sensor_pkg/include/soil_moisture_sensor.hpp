#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <yaml-cpp/yaml.h>

#include <random>
#include <string>
#include <vector>
#include <cmath>

class PerlinNoise {
public:
    PerlinNoise(unsigned int seed = 0);
    double noise(double x, double y) const;
    
private:
    std::vector<int> p;
    double fade(double t) const;
    double lerp(double t, double a, double b) const;
    double grad(int hash, double x, double y) const;
};

struct Tree {
    int id;
    std::string name;
    double x;
    double y;
    double z;
    double moisture;
    double ph;
};

class SoilMoistureSensor : public rclcpp::Node {
public:
    SoilMoistureSensor();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateSensor();
    void loadTreeData(const std::string &filename);
    double getMoistureAtPosition(double x, double y);
    double getPHAtPosition(double x, double y);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr moisture_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ph_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr location_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Tree> trees_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool odom_received_ = false;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;
    
    double sensing_radius_;
    PerlinNoise perlin_;
    double perlin_scale_;
    double perlin_offset_x_;
    double perlin_offset_y_;
    bool use_perlin_noise_;
};