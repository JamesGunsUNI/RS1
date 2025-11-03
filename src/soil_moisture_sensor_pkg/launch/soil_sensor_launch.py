from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for soil moisture sensor system."""
    
    try:
        pkg_share = get_package_share_directory('soil_moisture_sensor_pkg')
    except Exception:
        pkg_share = ''
    
    # Declare launch arguments
    sensing_radius_arg = DeclareLaunchArgument(
        'sensing_radius',
        default_value='0.5',
        description='Radius in meters for soil moisture sensing around trees'
    )
    
    sampling_radius_arg = DeclareLaunchArgument(
        'sampling_radius',
        default_value='1.0',
        description='Maximum distance for associating readings with trees'
    )
    
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.5',
        description='Grid cell size for heatmap in meters'
    )
    
    yaml_file_arg = DeclareLaunchArgument(
        'yaml_file',
        default_value='trees_moisture.yaml',
        description='Name of YAML file with tree data (must be in config/ directory)'
    )
    
    use_perlin_arg = DeclareLaunchArgument(
        'use_perlin_noise',
        default_value='true',
        description='Use Perlin noise for moisture field (true) or tree-based (false)'
    )
    
    perlin_scale_arg = DeclareLaunchArgument(
        'perlin_scale',
        default_value='0.1',
        description='Scale factor for Perlin noise (smaller = larger features)'
    )
    
    perlin_seed_arg = DeclareLaunchArgument(
        'perlin_seed',
        default_value='12345',
        description='Seed for Perlin noise generation (for reproducibility)'
    )
    
    map_min_x_arg = DeclareLaunchArgument(
        'map_min_x',
        default_value='-10.0',
        description='Minimum X coordinate for heatmap bounds'
    )
    
    map_max_x_arg = DeclareLaunchArgument(
        'map_max_x',
        default_value='10.0',
        description='Maximum X coordinate for heatmap bounds'
    )
    
    map_min_y_arg = DeclareLaunchArgument(
        'map_min_y',
        default_value='-10.0',
        description='Minimum Y coordinate for heatmap bounds'
    )
    
    map_max_y_arg = DeclareLaunchArgument(
        'map_max_y',
        default_value='10.0',
        description='Maximum Y coordinate for heatmap bounds'
    )
    
    # Soil Moisture Sensor Node
    sensor_node = Node(
        package='soil_moisture_sensor_pkg',
        executable='soil_moisture_sensor',
        name='soil_moisture_sensor',
        output='screen',
        parameters=[{
            'sensing_radius': LaunchConfiguration('sensing_radius'),
            'yaml_file': LaunchConfiguration('yaml_file'),
            'use_perlin_noise': LaunchConfiguration('use_perlin_noise'),
            'perlin_scale': LaunchConfiguration('perlin_scale'),
            'perlin_seed': LaunchConfiguration('perlin_seed'),
        }],
        remappings=[
            ('odom', 'odometry/filtered'),
        ],
        emulate_tty=True,
    )
    
    # Soil Moisture Visualizer Node
    visualizer_node = Node(
        package='soil_moisture_sensor_pkg',
        executable='soil_moisture_visualizer',
        name='soil_moisture_visualizer',
        output='screen',
        parameters=[{
            'sampling_radius': LaunchConfiguration('sampling_radius'),
            'grid_resolution': LaunchConfiguration('grid_resolution'),
            'yaml_file': LaunchConfiguration('yaml_file'),
            'map_min_x': LaunchConfiguration('map_min_x'),
            'map_max_x': LaunchConfiguration('map_max_x'),
            'map_min_y': LaunchConfiguration('map_min_y'),
            'map_max_y': LaunchConfiguration('map_max_y'),
        }],
        remappings=[
            ('/odom', '/odom'),
        ],
        emulate_tty=True,
    )
    
    # Create and return launch description
    return LaunchDescription([
        sensing_radius_arg,
        sampling_radius_arg,
        grid_resolution_arg,
        yaml_file_arg,
        use_perlin_arg,
        perlin_scale_arg,
        perlin_seed_arg,
        map_min_x_arg,
        map_max_x_arg,
        map_min_y_arg,
        map_max_y_arg,
        sensor_node,
        visualizer_node,
    ])