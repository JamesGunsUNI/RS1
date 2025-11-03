#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  MainWindow w;
  w.show();
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
