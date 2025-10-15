#pragma once
#include <QObject>
#include <QImage>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class RosImageBridge : public QObject {
  Q_OBJECT
public:
  explicit RosImageBridge(QObject* parent = nullptr);
  ~RosImageBridge();

  void start(const std::string& topic);  // subscribe to a camera topic
  void stop();

signals:
  void frameReady(const QImage& img);

private:
  void ensureNode();

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_;
};
