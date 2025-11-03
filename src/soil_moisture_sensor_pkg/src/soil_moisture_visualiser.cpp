/**
 * @file soil_moisture_visualiser.cpp
 * @brief Implementation of RViz visualization for soil moisture data
 * 
 * This node creates visual representations of soil moisture readings:
 * 1. Color-coded tree markers showing moisture at tree locations
 * 2. Grid-based heatmap showing moisture distribution across the map
 * 
 * The heatmap accumulates data as the robot explores, building up a
 * complete picture of moisture distribution over time.
 */

#include "soil_moisture_visualiser.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <limits>

// ============================================================================
// CONSTRUCTOR - Initialize visualizer with parameters and setup
// ============================================================================

/**
 * @brief Constructor - Sets up visualization node
 * 
 * Initializes:
 * - ROS2 parameters for heatmap configuration
 * - Subscribers for moisture data and robot position
 * - Publishers for RViz markers
 * - Timer for periodic visualization updates
 * - Tree position data from YAML file
 */
SoilMoistureVisualizer::SoilMoistureVisualizer()
: Node("soil_moisture_visualizer"), 
  robot_x_(0.0), robot_y_(0.0), 
  last_moisture_(0.0),
  pose_received_(false),
  moisture_received_(false)
{
    // ===== DECLARE ROS2 PARAMETERS =====
    // These control how the visualization behaves
    this->declare_parameter("sampling_radius", 1.0);
    this->declare_parameter("grid_resolution", 0.5);
    this->declare_parameter("map_min_x", -10.0);
    this->declare_parameter("map_max_x", 10.0);
    this->declare_parameter("map_min_y", -10.0);
    this->declare_parameter("map_max_y", 10.0);
    this->declare_parameter("yaml_file", "trees.yaml");
    
    // ===== GET PARAMETER VALUES =====
    this->get_parameter("sampling_radius", sampling_radius_);
    this->get_parameter("grid_resolution", grid_resolution_);
    this->get_parameter("map_min_x", map_min_x_);
    this->get_parameter("map_max_x", map_max_x_);
    this->get_parameter("map_min_y", map_min_y_);
    this->get_parameter("map_max_y", map_max_y_);

    // ===== CREATE SUBSCRIBERS =====
    
    // Subscribe to moisture readings from sensor node
    soil_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/soil_moisture", 10,
        std::bind(&SoilMoistureVisualizer::soilMoistureCallback, this, std::placeholders::_1));

    // Subscribe to sample locations (where readings were taken)
    // This allows us to build the heatmap grid
    location_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/soil_sample_location", 10,
        std::bind(&SoilMoistureVisualizer::sampleLocationCallback, this, std::placeholders::_1));

    // Subscribe to robot odometry for position tracking
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SoilMoistureVisualizer::odomCallback, this, std::placeholders::_1));

    // ===== CREATE PUBLISHERS =====
    
    // Publish tree markers and text labels as MarkerArray
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/soil_moisture_markers", 10);
    
    // Publish heatmap grid as single CUBE_LIST marker
    heatmap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/soil_moisture_heatmap", 10);

    // ===== CREATE TIMER =====
    // Update visualization at 2 Hz (every 500ms)
    viz_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SoilMoistureVisualizer::publishVisualization, this));

    // ===== LOAD TREE POSITIONS =====
    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("soil_moisture_sensor_pkg");
        RCLCPP_INFO(this->get_logger(), "Found package at: %s", pkg_share.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find package: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "Trying to use local directory");
        pkg_share = ".";
    }
    
    std::string yaml_file;
    this->get_parameter("yaml_file", yaml_file);
    std::string full_path = pkg_share + "/config/" + yaml_file;
    loadTreePositions(full_path);

    RCLCPP_INFO(this->get_logger(),
                "Visualizer started: %zu trees, grid=%.2fm, sampling=%.2fm",
                trees_.size(), grid_resolution_, sampling_radius_);
}

// ============================================================================
// DATA LOADING
// ============================================================================

/**
 * @brief Load tree positions from YAML configuration file
 * 
 * Trees are used for visualization purposes - they are rendered as
 * colored cylinders in RViz, with color indicating moisture level.
 * 
 * Expected YAML format:
 * trees:
 *   - id: 1
 *     name: tree_1
 *     x: 0.0
 *     y: 3.0
 *     z: 0.0
 */
void SoilMoistureVisualizer::loadTreePositions(const std::string &yaml_file) {
    try {
        RCLCPP_INFO(this->get_logger(), "Loading trees from: %s", yaml_file.c_str());
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        // Parse each tree entry
        for (const auto &tree : config["trees"]) {
            TreePosition t;
            t.id = tree["id"].as<int>();
            t.name = tree["name"].as<std::string>();
            t.x = tree["x"].as<double>();
            t.y = tree["y"].as<double>();
            t.z = tree["z"].as<double>();
            t.has_reading = false;  // No moisture data yet
            t.moisture_reading = 0.0;
            trees_.push_back(t);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu tree positions", trees_.size());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load trees: %s", e.what());
    }
}

// ============================================================================
// CALLBACK FUNCTIONS - Handle incoming ROS2 messages
// ============================================================================

/**
 * @brief Callback for odometry messages - updates robot position
 * 
 * We track robot position to potentially use in future features
 * (e.g., showing robot location on heatmap).
 */
void SoilMoistureVisualizer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    pose_received_ = true;
}

/**
 * @brief Callback for moisture readings - stores latest value
 * 
 * The moisture value is stored temporarily until we receive the
 * corresponding sample location message. This allows us to pair
 * moisture readings with their spatial locations.
 */
void SoilMoistureVisualizer::soilMoistureCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    // Only store valid readings (ignore NaN which means "no data")
    if (!std::isnan(msg->data)) {
        last_moisture_ = msg->data;
        moisture_received_ = true;
    }
}

/**
 * @brief Callback for sample location - updates heatmap and tree data
 * 
 * When a sample location arrives, we:
 * 1. Add the moisture reading to the heatmap grid
 * 2. Find the nearest tree and update its moisture reading
 * 
 * This builds up both the continuous heatmap and discrete tree markers.
 */
void SoilMoistureVisualizer::sampleLocationCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    
    // Can't process without a moisture value
    if (!moisture_received_) return;

    double x = msg->point.x;
    double y = msg->point.y;
    
    // Update the heatmap grid with this reading
    updateHeatmap(x, y, last_moisture_);
    
    // Find closest tree to this sample location
    double dist;
    int tree_id = findClosestTree(x, y, dist);
    
    // Update tree's moisture reading if it's close enough
    if (tree_id >= 0 && tree_id < static_cast<int>(trees_.size())) {
        trees_[tree_id].moisture_reading = last_moisture_;
        trees_[tree_id].has_reading = true;
        trees_[tree_id].last_update = this->now();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Updated %s with moisture %.3f (dist=%.2fm)",
                   trees_[tree_id].name.c_str(), last_moisture_, dist);
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Find the closest tree to a given position
 * 
 * Uses Euclidean distance to find which tree is nearest to the
 * sample location. This associates moisture readings with trees
 * for visualization purposes.
 * 
 * @param x X coordinate of sample
 * @param y Y coordinate of sample
 * @param dist Output parameter - distance to closest tree
 * @return Index of closest tree in trees_ vector, or -1 if no trees
 */
int SoilMoistureVisualizer::findClosestTree(double x, double y, double &dist) {
    int closest_idx = -1;
    dist = std::numeric_limits<double>::max();

    // Linear search through all trees (fine for small number of trees)
    for (size_t i = 0; i < trees_.size(); ++i) {
        double dx = x - trees_[i].x;
        double dy = y - trees_[i].y;
        double d = std::sqrt(dx*dx + dy*dy);
        
        if (d < dist) {
            dist = d;
            closest_idx = static_cast<int>(i);
        }
    }
    return closest_idx;
}

/**
 * @brief Add a moisture reading to the heatmap grid
 * 
 * The heatmap uses a grid structure where each cell accumulates multiple
 * readings. This allows us to average out noise and build a smoother
 * visualization as the robot explores.
 * 
 * Grid cells are identified by integer coordinates (grid_x, grid_y).
 * Multiple readings in the same cell are averaged together.
 * 
 * @param x X coordinate of sample (world coordinates)
 * @param y Y coordinate of sample (world coordinates)
 * @param moisture Moisture value [0, 1]
 */
void SoilMoistureVisualizer::updateHeatmap(double x, double y, double moisture) {
    // Convert world coordinates to grid cell indices
    // Floor division ensures consistent cell assignment
    int grid_x = static_cast<int>(std::floor(x / grid_resolution_));
    int grid_y = static_cast<int>(std::floor(y / grid_resolution_));
    
    // Use (grid_x, grid_y) pair as unique key for the cell
    auto key = std::make_pair(grid_x, grid_y);
    
    // Check if this grid cell exists yet
    if (heatmap_grid_.find(key) == heatmap_grid_.end()) {
        // New cell - create it
        HeatmapCell cell;
        // Position at center of grid cell
        cell.x = grid_x * grid_resolution_ + grid_resolution_ / 2.0;
        cell.y = grid_y * grid_resolution_ + grid_resolution_ / 2.0;
        cell.moisture_sum = moisture;
        cell.sample_count = 1;
        cell.last_update = this->now();
        heatmap_grid_[key] = cell;
    } else {
        // Existing cell - accumulate new reading
        heatmap_grid_[key].moisture_sum += moisture;
        heatmap_grid_[key].sample_count++;
        heatmap_grid_[key].last_update = this->now();
    }
}

/**
 * @brief Convert moisture value to color using a gradient
 * 
 * Color scheme (mimics soil moisture appearance):
 * - 0.0 - 0.3 (Dry):    Brown to Orange (R high, G low)
 * - 0.3 - 0.6 (Medium): Orange to Yellow to Green
 * - 0.6 - 1.0 (Wet):    Green to Blue-Green
 * 
 * This creates an intuitive visualization where dry areas look
 * brown/red and wet areas look blue/green.
 * 
 * @param moisture Moisture value [0, 1]
 * @return RGBA color for visualization
 */
std_msgs::msg::ColorRGBA SoilMoistureVisualizer::moistureToColor(double moisture) {
    std_msgs::msg::ColorRGBA color;
    color.a = 0.8;  // 80% opacity for better blending
    
    if (moisture < 0.3) {
        // DRY: Brown (high red, low green) transitioning to orange
        // Linear interpolation from brown (0.8, 0.0, 0.0) to orange (1.0, 0.4, 0.0)
        double t = moisture / 0.3;  // Normalize to [0, 1] within this range
        color.r = 0.8 + t * 0.2;    // 0.8 -> 1.0
        color.g = t * 0.4;           // 0.0 -> 0.4
        color.b = 0.0;
    } else if (moisture < 0.6) {
        // MEDIUM: Orange through yellow to green
        // Transition from orange (1.0, 0.4, 0.0) to green (0.0, 1.0, 0.0)
        double t = (moisture - 0.3) / 0.3;  // Normalize to [0, 1]
        color.r = 1.0 - t;           // 1.0 -> 0.0 (reduce red)
        color.g = 0.4 + t * 0.6;     // 0.4 -> 1.0 (increase green)
        color.b = 0.0;
    } else {
        // WET: Green to blue-green
        // Transition from green (0.0, 1.0, 0.0) to teal (0.0, 0.7, 0.7)
        double t = (moisture - 0.6) / 0.4;  // Normalize to [0, 1]
        color.r = 0.0;
        color.g = 1.0 - t * 0.3;     // 1.0 -> 0.7 (slightly reduce green)
        color.b = t * 0.7;           // 0.0 -> 0.7 (add blue)
    }
    
    return color;
}

// ============================================================================
// VISUALIZATION PUBLISHING
// ============================================================================

/**
 * @brief Main visualization function - publishes all markers
 * 
 * Called by timer at 2 Hz. Publishes:
 * 1. Tree markers (cylinders with colors based on moisture)
 * 2. Text labels showing moisture percentage at trees
 * 3. Heatmap grid showing moisture distribution
 * 
 * This function is called periodically even if no new data arrived,
 * ensuring RViz always has up-to-date markers.
 */
void SoilMoistureVisualizer::publishVisualization() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // ===== CREATE TREE MARKERS =====
    for (size_t i = 0; i < trees_.size(); ++i) {
        const auto &tree = trees_[i];
        
        // ----- Create cylinder marker for tree -----
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "tree_markers";  // Namespace for grouping
        marker.id = tree.id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at tree location
        marker.pose.position.x = tree.x;
        marker.pose.position.y = tree.y;
        marker.pose.position.z = 1.0;  // Elevate to ground level
        marker.pose.orientation.w = 1.0;  // No rotation
        
        // Size: 0.3m diameter, 2m tall cylinder
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 2.0;
        
        // Color based on moisture reading
        if (tree.has_reading) {
            // Use moisture-based color gradient
            marker.color = moistureToColor(tree.moisture_reading);
        } else {
            // Gray for trees without data yet
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 0.5;
        }
        
        marker_array.markers.push_back(marker);
        
        // ----- Create text label if we have moisture data -----
        if (tree.has_reading) {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = marker.header;
            text_marker.ns = "tree_text";  // Different namespace
            text_marker.id = tree.id + 1000;  // Offset ID to avoid collision
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position above the tree cylinder
            text_marker.pose.position.x = tree.x;
            text_marker.pose.position.y = tree.y;
            text_marker.pose.position.z = 2.5;  // Above tree
            text_marker.pose.orientation.w = 1.0;
            
            // Text size
            text_marker.scale.z = 0.3;  // 30cm tall text
            
            // White text for visibility
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // Format text: "tree_1\n35.2%"
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "%s\n%.1f%%", 
                    tree.name.c_str(), tree.moisture_reading * 100.0);
            text_marker.text = buffer;
            
            marker_array.markers.push_back(text_marker);
        }
    }
    
    // Publish all tree markers and labels
    marker_pub_->publish(marker_array);
    
    // Publish heatmap grid
    publishHeatmapMarkers();
}

/**
 * @brief Publish the heatmap grid as a CUBE_LIST marker
 * 
 * CUBE_LIST is an efficient marker type that renders many small cubes
 * with individual colors. Perfect for grid-based heatmaps.
 * 
 * Each grid cell is rendered as a thin square at ground level,
 * colored according to the average moisture in that cell.
 * 
 * The heatmap accumulates over time as the robot explores, creating
 * a persistent visualization of moisture distribution.
 */
void SoilMoistureVisualizer::publishHeatmapMarkers() {
    // Don't publish empty heatmap
    if (heatmap_grid_.empty()) return;
    
    // Create CUBE_LIST marker
    visualization_msgs::msg::Marker heatmap;
    heatmap.header.frame_id = "odom";
    heatmap.header.stamp = this->now();
    heatmap.ns = "moisture_heatmap";
    heatmap.id = 0;  // Single marker containing all cubes
    heatmap.type = visualization_msgs::msg::Marker::CUBE_LIST;
    heatmap.action = visualization_msgs::msg::Marker::ADD;
    
    // Size of each cube (matches grid resolution)
    heatmap.scale.x = grid_resolution_;
    heatmap.scale.y = grid_resolution_;
    heatmap.scale.z = 0.01;  // Very thin (1cm) so it doesn't obscure ground
    
    heatmap.pose.orientation.w = 1.0;  // No rotation
    
    // Add each grid cell to the CUBE_LIST
    for (const auto &entry : heatmap_grid_) {
        const HeatmapCell &cell = entry.second;
        
        // Calculate average moisture for this cell
        // (cells accumulate multiple readings over time)
        double avg_moisture = cell.moisture_sum / cell.sample_count;
        
        // Position of cube (center of grid cell, slightly above ground)
        geometry_msgs::msg::Point p;
        p.x = cell.x;
        p.y = cell.y;
        p.z = 0.05;  // 5cm above ground to avoid z-fighting
        heatmap.points.push_back(p);
        
        // Color based on average moisture
        heatmap.colors.push_back(moistureToColor(avg_moisture));
    }
    
    // Publish the complete heatmap
    heatmap_pub_->publish(heatmap);
}