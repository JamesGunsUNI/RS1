from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time if true"
    )
    ld.add_action(use_sim_time_arg)

    sampling_radius_arg = DeclareLaunchArgument(
        "sampling_radius",
        default_value="1.0",
        description="Maximum distance (m) to a tree for taking a moisture reading"
    )
    ld.add_action(sampling_radius_arg)

    # Paths
    pkg_share = FindPackageShare("soil_moisture_sensor_pkg")
    trees_yaml = PathJoinSubstitution([pkg_share, "config", "trees.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "soil_sensor.rviz"])

    # Soil moisture sensor node
    sensor_node = Node(
        package="soil_moisture_sensor_pkg",
        executable="soil_moisture_sensor",
        name="soil_moisture_sensor",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            trees_yaml
        ]
    )
    ld.add_action(sensor_node)

    # Visualizer node
    visualizer_node = Node(
        package="soil_moisture_sensor_pkg",
        executable="soil_moisture_visualizer",
        name="soil_moisture_visualizer",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"sampling_radius": LaunchConfiguration("sampling_radius")}
        ]
    )
    ld.add_action(visualizer_node)

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )
    ld.add_action(rviz_node)

    return ld
