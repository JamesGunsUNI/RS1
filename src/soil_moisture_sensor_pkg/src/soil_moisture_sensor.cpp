#include "soil_moisture_sensor.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <algorithm>

// ============================================================================
// PERLIN NOISE IMPLEMENTATION
// ============================================================================
// Perlin noise is a gradient noise function that creates natural-looking
// random patterns. It works by:
// 1. Dividing space into a grid
// 2. Assigning random gradient vectors to grid corners
// 3. Interpolating smoothly between these gradients
// This creates patterns that look organic rather than purely random.
// ============================================================================

/**
 * @brief Initialize Perlin noise with a permutation table
 * 
 * The permutation table is a shuffled array of integers [0-255] that
 * determines the pseudo-random gradients at each grid point. Using the
 * same seed produces identical noise patterns (deterministic behavior).
 */
PerlinNoise::PerlinNoise(unsigned int seed) {
    // Create array of integers 0-255
    p.resize(256);
    for (int i = 0; i < 256; i++) {
        p[i] = i;
    }
    
    // Shuffle array using the provided seed for reproducibility
    std::default_random_engine engine(seed);
    std::shuffle(p.begin(), p.end(), engine);
    
    // Duplicate the permutation vector to avoid overflow when indexing
    // This allows us to use p[X+1] without checking bounds
    p.insert(p.end(), p.begin(), p.end());
}

/**
 * @brief Fade function for smooth interpolation (6t^5 - 15t^4 + 10t^3)
 * 
 * This is Ken Perlin's improved fade function. It has zero first and
 * second derivatives at t=0 and t=1, creating smoother transitions
 * than linear interpolation.
 */
double PerlinNoise::fade(double t) const {
    return t * t * t * (t * (t * 6 - 15) + 10);
}

/**
 * @brief Linear interpolation between two values
 */
double PerlinNoise::lerp(double t, double a, double b) const {
    return a + t * (b - a);
}

/**
 * @brief Calculate dot product of gradient vector and distance vector
 * 
 * The hash value determines which of 16 possible gradient directions to use.
 * This creates the pseudo-random behavior that makes Perlin noise work.
 * 
 * @param hash Determines gradient direction (0-15)
 * @param x X distance from grid corner
 * @param y Y distance from grid corner
 * @return Dot product of gradient and distance vectors
 */
double PerlinNoise::grad(int hash, double x, double y) const {
    int h = hash & 15;  // Use lower 4 bits (16 gradients)
    
    // Select gradient vector based on hash
    double u = h < 8 ? x : y;
    double v = h < 4 ? y : h == 12 || h == 14 ? x : 0;
    
    // Randomly negate components based on hash bits
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

/**
 * @brief Generate 2D Perlin noise value at given coordinates
 * 
 * Algorithm:
 * 1. Find the grid square containing the point
 * 2. Get distances from point to all 4 corners
 * 3. Calculate dot products of gradients at corners with distance vectors
 * 4. Interpolate results smoothly using fade function
 * 
 * @param x X coordinate in noise space
 * @param y Y coordinate in noise space
 * @return Noise value in range [0, 1]
 */
double PerlinNoise::noise(double x, double y) const {
    // Find grid cell containing point (integer coordinates)
    int X = static_cast<int>(std::floor(x)) & 255;
    int Y = static_cast<int>(std::floor(y)) & 255;
    
    // Get fractional part of coordinates (position within cell)
    x -= std::floor(x);
    y -= std::floor(y);
    
    // Apply fade function for smooth interpolation
    double u = fade(x);
    double v = fade(y);
    
    // Hash coordinates of the 4 square corners
    int A = p[X] + Y;      // Bottom-left
    int AA = p[A];
    int AB = p[A + 1];     // Top-left
    int B = p[X + 1] + Y;  // Bottom-right
    int BA = p[B];
    int BB = p[B + 1];     // Top-right
    
    // Bilinear interpolation of the 4 corner gradients
    // First interpolate along x at bottom (y=0) and top (y=1)
    // Then interpolate those results along y
    double res = lerp(v, 
                     lerp(u, grad(p[AA], x, y), grad(p[BA], x - 1, y)),
                     lerp(u, grad(p[AB], x, y - 1), grad(p[BB], x - 1, y - 1)));
    
    // Normalize from [-1, 1] to [0, 1]
    return (res + 1.0) / 2.0;
}

// ============================================================================
// SOIL MOISTURE SENSOR IMPLEMENTATION
// ============================================================================

/**
 * @brief Constructor - Initialize sensor node with parameters and setup
 */
SoilMoistureSensor::SoilMoistureSensor()
    : Node("soil_moisture_sensor"),
      gen_(rd_()),
      noise_dist_(0.0, 0.01),  // Reduced noise since Perlin provides smoothness
      perlin_(std::random_device{}())
{
    this->declare_parameter("sensing_radius", 0.5);
    this->declare_parameter("yaml_file", "trees_moisture.yaml");
    this->declare_parameter("use_perlin_noise", true);
    this->declare_parameter("perlin_scale", 0.1);
    this->declare_parameter("perlin_seed", 12345);
    
    this->get_parameter("sensing_radius", sensing_radius_);
    
    bool custom_seed = false;
    int perlin_seed;
    if (this->get_parameter("perlin_seed", perlin_seed)) {
        perlin_ = PerlinNoise(perlin_seed);
        custom_seed = true;
    }
    
    this->get_parameter("use_perlin_noise", use_perlin_noise_);
    this->get_parameter("perlin_scale", perlin_scale_);
    
    // Random offsets to make different runs unique
    std::uniform_real_distribution<> offset_dist(0.0, 1000.0);
    perlin_offset_x_ = offset_dist(gen_);
    perlin_offset_y_ = offset_dist(gen_);
    
    std::string yaml_file;
    this->get_parameter("yaml_file", yaml_file);

    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("ignition_bringup");
        RCLCPP_INFO(this->get_logger(), "Found package at: %s", pkg_share.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find package: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "Trying to use local directory");
        pkg_share = ".";
    }
    
    std::string full_path = pkg_share + "/config/" + yaml_file;
    loadTreeData(full_path);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SoilMoistureSensor::odomCallback, this, std::placeholders::_1));

    moisture_pub_ = this->create_publisher<std_msgs::msg::Float32>("/soil_moisture", 10);
    
    location_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/soil_sample_location", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SoilMoistureSensor::updateSensor, this));

    RCLCPP_INFO(this->get_logger(), 
                "SoilMoistureSensor started: %zu trees, sensing=%.2fm, perlin=%s (scale=%.3f, seed=%s)", 
                trees_.size(), sensing_radius_, 
                use_perlin_noise_ ? "enabled" : "disabled",
                perlin_scale_,
                custom_seed ? std::to_string(perlin_seed).c_str() : "random");
}

void SoilMoistureSensor::loadTreeData(const std::string &filename) {
    try {
        RCLCPP_INFO(this->get_logger(), "Loading tree data from: %s", filename.c_str());
        YAML::Node config = YAML::LoadFile(filename);
        
        if (!config["trees"]) {
            RCLCPP_ERROR(this->get_logger(), "No 'trees' key found in YAML file");
            return;
        }
        
        for (const auto &t : config["trees"]) {
            Tree tree;
            tree.id = t["id"].as<int>();
            tree.name = t["name"].as<std::string>();
            tree.x = t["x"].as<double>();
            tree.y = t["y"].as<double>();
            tree.z = t["z"].as<double>();
            tree.moisture = t["moisture"].as<double>();
            trees_.push_back(tree);
            
            RCLCPP_DEBUG(this->get_logger(), "Loaded %s at (%.2f, %.2f) with moisture %.2f", 
                        tree.name.c_str(), tree.x, tree.y, tree.moisture);
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu trees", trees_.size());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load tree data: %s", e.what());
    }
}

void SoilMoistureSensor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    odom_received_ = true;
}

double SoilMoistureSensor::getMoistureAtPosition(double x, double y) {
    if (!use_perlin_noise_) {
        // Fallback to tree-based moisture with noise
        for (const auto &tree : trees_) {
            double dx = x - tree.x;
            double dy = y - tree.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < sensing_radius_) {
                double noisy_val = tree.moisture + noise_dist_(gen_);
                return std::max(0.0, std::min(1.0, noisy_val));
            }
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Use Perlin noise for continuous moisture field
    double perlin_x = (x + perlin_offset_x_) * perlin_scale_;
    double perlin_y = (y + perlin_offset_y_) * perlin_scale_;
    
    // Get base Perlin noise value [0, 1]
    double base_moisture = perlin_.noise(perlin_x, perlin_y);
    
    // Add octaves for more detail
    double octave1 = perlin_.noise(perlin_x * 2.0, perlin_y * 2.0) * 0.5;
    double octave2 = perlin_.noise(perlin_x * 4.0, perlin_y * 4.0) * 0.25;
    
    // Combine octaves
    double moisture = (base_moisture + octave1 * 0.3 + octave2 * 0.15);
    
    // Normalize to reasonable range (0.2 to 0.8 for realistic soil moisture)
    moisture = 0.2 + moisture * 0.6;
    
    // Add tiny bit of sensor noise
    moisture += noise_dist_(gen_);
    
    // Clamp to [0, 1]
    return std::max(0.0, std::min(1.0, moisture));
}

void SoilMoistureSensor::updateSensor() {
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Waiting for odometry data...");
        return;
    }

    float reading = getMoistureAtPosition(robot_x_, robot_y_);
    
    if (!std::isnan(reading)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                   "Position (%.2f, %.2f): moisture = %.3f", 
                   robot_x_, robot_y_, reading);
        
        geometry_msgs::msg::PointStamped location_msg;
        location_msg.header.stamp = this->now();
        location_msg.header.frame_id = "odom";
        location_msg.point.x = robot_x_;
        location_msg.point.y = robot_y_;
        location_msg.point.z = 0.0;
        location_pub_->publish(location_msg);
    }

    std_msgs::msg::Float32 msg;
    msg.data = reading;
    moisture_pub_->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SoilMoistureSensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}