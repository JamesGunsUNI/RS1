cmake_minimum_required(VERSION 3.10)
project(tb4_env_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Qt MOC helpers
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ROS 2 + image stack
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(tb4_env_ui
  src/main.cpp
  src/mainwindow.cpp
  src/process_launcher.cpp
  src/ros_image_bridge.cpp         # NEW
  src/image_widget.cpp             # NEW
  include/mainwindow.h
  include/process_launcher.h
  include/ros_image_bridge.h       # NEW
  include/image_widget.h           # NEW
)

target_include_directories(tb4_env_ui PUBLIC include)
target_link_libraries(tb4_env_ui Qt5::Widgets ${OpenCV_LIBS})
ament_target_dependencies(tb4_env_ui rclcpp sensor_msgs image_transport cv_bridge)

install(TARGETS tb4_env_ui
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
