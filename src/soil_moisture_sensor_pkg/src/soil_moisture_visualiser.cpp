#include "soil_moisture_visualiser.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <limits>

SoilMoistureVisualizer::SoilMoistureVisualizer()
: Node("soil_moisture_visualizer"), robot_x_(0.0), robot_y_(0.0), pose_received_(false)
{
  // Declare and get sampling radius parameter (default = 1.0 m)
  this->declare_parameter("sampling_radius", 1.0);
  this->get_parameter("sampling_radius", sampling_radius_);

  // Subscription to soil moisture topic
  soil_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/soil_moisture", 10,
    std::bind(&SoilMoistureVisualizer::soilMoistureCallback, this, std::placeholders::_1));

  // Subscription to robot pose (could be /amcl_pose or /robot_pose)
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/amcl_pose", 10,
    std::bind(&SoilMoistureVisualizer::poseCallback, this, std::placeholders::_1));

  // Publisher for RViz markers
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/soil_moisture_markers", 10);

  // Load tree positions from YAML
  std::string pkg_share = ament_index_cpp::get_package_share_directory("soil_moisture_sensor_pkg");
  std::string yaml_file = pkg_share + "/config/trees.yaml";
  loadTreePositions(yaml_file);

  RCLCPP_INFO(this->get_logger(),
              "Loaded %zu trees from %s (sampling radius = %.2f m)",
              trees_.size(), yaml_file.c_str(), sampling_radius_);
}

void SoilMoistureVisualizer::loadTreePositions(const std::string &yaml_file)
{
  YAML::Node config = YAML::LoadFile(yaml_file);
  for (const auto &tree : config["trees"])
  {
    TreePosition t;
    t.id = tree["id"].as<int>();
    t.x  = tree["x"].as<double>();
    t.y  = tree["y"].as<double>();
    t.z  = tree["z"].as<double>();
    trees_.push_back(t);
  }
}

void SoilMoistureVisualizer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  robot_x_ = msg->pose.position.x;
  robot_y_ = msg->pose.position.y;
  pose_received_ = true;
}

int SoilMoistureVisualizer::findClosestTree(double x, double y, double &dist)
{
  int closest_id = -1;
  dist = std::numeric_limits<double>::max();

  for (const auto &tree : trees_)
  {
    double dx = x - tree.x;
    double dy = y - tree.y;
    double d = std::sqrt(dx*dx + dy*dy);
    if (d < dist)
    {
      dist = d;
      closest_id = tree.id;
    }
  }
  return closest_id;
}

void SoilMoistureVisualizer::soilMoistureCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!pose_received_ || trees_.empty()) return;

  double dist;
  int tree_id = findClosestTree(robot_x_, robot_y_, dist);

  if (tree_id == -1 || dist > sampling_radius_)
  {
    RCLCPP_WARN(this->get_logger(),
                "No tree within %.2f m (closest = %.2f m), ignoring moisture reading.",
                sampling_radius_, dist);
    return;
  }

  // Get tree info
  auto it = std::find_if(trees_.begin(), trees_.end(),
                         [tree_id](const TreePosition &t){ return t.id == tree_id; });
  if (it == trees_.end()) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";  // Adjust if you use odom
  marker.header.stamp = this->now();
  marker.ns = "soil_moisture";
  marker.id = it->id;

  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = it->x;
  marker.pose.position.y = it->y;
  marker.pose.position.z = it->z + 1.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.z = 0.5;  // text height
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.text = "Tree " + std::to_string(it->id) +
                " Moisture: " + std::to_string(msg->data);

  marker_pub_->publish(marker);

  RCLCPP_INFO(this->get_logger(),
              "Updated Tree %d (dist=%.2f m) with moisture %.2f",
              it->id, dist, msg->data);
}