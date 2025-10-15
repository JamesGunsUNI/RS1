#pragma once
#include <QObject>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class RosClient : public QObject {
  Q_OBJECT
public:
  explicit RosClient(QObject* parent = nullptr);
  ~RosClient();

  Q_INVOKABLE void start();   // calls /env/start
  Q_INVOKABLE void stop();    // calls /env/stop
  bool servicesAvailable() const; // true when both services are ready

signals:
  void startResult(bool ok, QString msg);
  void stopResult(bool ok, QString msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
};
