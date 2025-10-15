cmake_minimum_required(VERSION 3.10)
project(tb4_env_ui)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ament (ROS 2)
find_package(ament_cmake REQUIRED)

# Qt5 Widgets
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Qt MOC helpers
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# libcurl (required for GDAL / NetCDF linking)
find_package(CURL REQUIRED)

# Executable
add_executable(tb4_env_ui
  src/main.cpp
  src/mainwindow.cpp
  src/process_launcher.cpp
  src/ros_image_bridge.cpp
  src/image_widget.cpp
  include/mainwindow.h
  include/process_launcher.h
  include/ros_image_bridge.h
  include/image_widget.h
)

# Include directories
target_include_directories(tb4_env_ui PUBLIC include)

# Link libraries
target_link_libraries(tb4_env_ui
  Qt5::Widgets
  ${OpenCV_LIBS}
  CURL::libcurl
)

# Declare ROS 2 dependencies
ament_target_dependencies(tb4_env_ui
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

# Install the executable
install(TARGETS tb4_env_ui
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Export as ament package
ament_package()
