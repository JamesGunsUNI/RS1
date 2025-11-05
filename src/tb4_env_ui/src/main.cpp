/*
File: main.cpp
Role:
  - Entry point. Initializes ROS 2 first (so any Qt‑constructed objects can create ROS nodes later),
    then starts the Qt application and shows MainWindow.
Shutdown order:
  - Qt event loop exits → we call rclcpp::shutdown() to cleanly tear down any remaining nodes/executors.
*/
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.h"

int main(int argc, char** argv)
/** Program entry: initialize ROS first, then Qt; tear down ROS after the event loop. */ {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  MainWindow w;
  w.show();
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
