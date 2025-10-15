// src/main.cpp
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);        // allow ROS in the same process
  QApplication app(argc, argv);
  MainWindow w; w.show();
  int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
