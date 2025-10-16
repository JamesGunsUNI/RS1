#include "soil_moisture_visualiser.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SoilMoistureVisualizer>();
    
    RCLCPP_INFO(node->get_logger(), "Soil Moisture Visualizer node is running...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}