#ifndef SOIL_MOISTURE_VISUALIZER_HPP
#define SOIL_MOISTURE_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <map>

struct TreePosition {
    int id;
    std::string name;
    double x, y, z;
    double moisture_reading;
    bool has_reading;
    rclcpp::Time last_update;
};

struct HeatmapCell {
    double x, y;
    double moisture_sum;
    int sample_count;
    rclcpp::Time last_update;
};

class SoilMoistureVisualizer : public rclcpp::Node {
public:
    SoilMoistureVisualizer();

private:
    void soilMoistureCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void sampleLocationCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishVisualization();
    void loadTreePositions(const std::string &yaml_file);
    int findClosestTree(double x, double y, double &dist);
    void updateHeatmap(double x, double y, double moisture);
    void publishHeatmapMarkers();
    std_msgs::msg::ColorRGBA moistureToColor(double moisture);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr soil_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr location_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr heatmap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heatmap_cloud_pub_;
    
    rclcpp::TimerBase::SharedPtr viz_timer_;

    std::vector<TreePosition> trees_;
    std::map<std::pair<int, int>, HeatmapCell> heatmap_grid_;
    
    double robot_x_, robot_y_;
    double last_moisture_;
    bool pose_received_;
    bool moisture_received_;

    double sampling_radius_;
    double grid_resolution_;
    double map_min_x_, map_max_x_;
    double map_min_y_, map_max_y_;
};

#endif