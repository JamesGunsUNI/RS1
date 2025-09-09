#include "rclcpp/rclcpp.hpp"
#include "soil_moisture_visualiser.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<SoilMoistureVisualizer>();

    // Spin node
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
