#pragma once
#include <QObject>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class RosBridge : public QObject {
  Q_OBJECT
public:
  explicit RosBridge(QObject* parent = nullptr)
  : QObject(parent),
    node_(std::make_shared<rclcpp::Node>("ui_bridge")),
    exec_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
  {
    moist_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/soil_moisture", 10,
      [this](std_msgs::msg::Float32::SharedPtr m){ emit moisture(m->data); });

    loc_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/soil_sample_location", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr m){
        emit sampleXY(m->point.x, m->point.y);
      });

    exec_->add_node(node_);
    worker_ = std::thread([this](){ exec_->spin(); });
  }

  ~RosBridge() override {
    if (exec_) exec_->cancel();
    if (worker_.joinable()) worker_.join();
  }

signals:
  void moisture(float value);          // 0..1
  void sampleXY(double x, double y);   // map coords

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr loc_sub_;
  std::shared_ptr<rclcpp::Executor> exec_;
  std::thread worker_;
};
