cmake_minimum_required(VERSION 3.8)
project(soil_moisture_sensor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories MUST come before add_executable
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

# Soil Moisture Sensor executable
add_executable(soil_moisture_sensor src/soil_moisture_sensor.cpp
)

ament_target_dependencies(soil_moisture_sensor
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  ament_index_cpp
)

target_link_libraries(soil_moisture_sensor
  yaml-cpp
)

# Soil Moisture Visualizer executable (only ONE declaration)
add_executable(soil_moisture_visualizer src/soil_moisture_visualiser.cpp src/soil_moisture_visualiser_node.cpp)

ament_target_dependencies(soil_moisture_visualizer
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(soil_moisture_visualizer
  yaml-cpp
)

# Install executables
install(TARGETS
  soil_moisture_sensor
  soil_moisture_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()