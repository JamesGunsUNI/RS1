#ifndef SOIL_MOISTURE_VISUALIZER_HPP
#define SOIL_MOISTURE_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

struct TreePosition
{
  int id;
  double x, y, z;
};

class SoilMoistureVisualizer : public rclcpp::Node
{
public:
  SoilMoistureVisualizer();

private:
  void soilMoistureCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void loadTreePositions(const std::string &yaml_file);
  int findClosestTree(double x, double y, double &dist);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr soil_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::vector<TreePosition> trees_;
  double robot_x_, robot_y_;
  bool pose_received_;

  double sampling_radius_;  // NEW: max distance for sampling
};

#endif
