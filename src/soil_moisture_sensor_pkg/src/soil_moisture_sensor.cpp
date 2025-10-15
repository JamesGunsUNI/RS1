#include "soil_moisture_sensor.hpp"

#include <cmath>

SoilMoistureSensor::SoilMoistureSensor()
    : Node("soil_moisture_sensor"),
      gen_(rd_()),
      noise_dist_(0.0, 0.02)  // mean = 0, stddev = 0.02
{
    // Load tree data
    loadTreeData("trees.yaml");

    // Subscriber to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SoilMoistureSensor::odomCallback, this, std::placeholders::_1));

    // Publisher for soil moisture readings
    moisture_pub_ = this->create_publisher<std_msgs::msg::Float32>("/soil_moisture", 10);

    // Timer for periodic updates (5 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SoilMoistureSensor::updateSensor, this));

    RCLCPP_INFO(this->get_logger(), "SoilMoistureSensor node started.");
}

void SoilMoistureSensor::loadTreeData(const std::string &filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        for (const auto &t : config["trees"]) {
            Tree tree;
            tree.name = t["name"].as<std::string>();
            tree.x = t["position"][0].as<double>();
            tree.y = t["position"][1].as<double>();
            tree.z = t["position"][2].as<double>();
            tree.moisture = t["moisture"].as<double>();
            trees_.push_back(tree);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu trees from %s", trees_.size(), filename.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load tree data: %s", e.what());
    }
}

void SoilMoistureSensor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
}

void SoilMoistureSensor::updateSensor() {
    float reading = std::numeric_limits<float>::quiet_NaN();

    for (const auto &tree : trees_) {
        double dx = robot_x_ - tree.x;
        double dy = robot_y_ - tree.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 0.5) { // within 0.5 m radius
            double noisy_val = tree.moisture + noise_dist_(gen_);
            noisy_val = std::max(0.0, std::min(1.0, noisy_val)); // clamp [0,1]
            reading = static_cast<float>(noisy_val);
            break; // stop at first matching tree
        }
    }

    std_msgs::msg::Float32 msg;
    msg.data = reading;
    moisture_pub_->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoilMoistureSensor>());
    rclcpp::shutdown();
    return 0;
}
