cmake_minimum_required(VERSION 3.8)
project(soil_moisture_sensor_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ----------------------------
# Library: soil_moisture_visualiser
# ----------------------------
add_library(soil_moisture_visualiser
  src/soil_moisture_visualiser.cpp
)

target_include_directories(soil_moisture_visualiser PUBLIC include)

ament_target_dependencies(soil_moisture_visualiser
  rclcpp
  std_msgs
  visualization_msgs
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(soil_moisture_visualiser yaml-cpp)

# ----------------------------
# Executable: soil_moisture_sensor
# ----------------------------
add_executable(soil_moisture_sensor
  src/soil_moisture_sensor.cpp
)

target_include_directories(soil_moisture_sensor PRIVATE include)

ament_target_dependencies(soil_moisture_sensor
  rclcpp
  nav_msgs
  std_msgs
)

target_link_libraries(soil_moisture_sensor
  soil_moisture_visualiser  # link the library
  yaml-cpp
)

# ----------------------------
# Install targets
# ----------------------------
install(TARGETS
  soil_moisture_sensor
  soil_moisture_visualiser
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install YAML config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
