/*
File: ros_client.h
Role & context:
  - Declares RosClient, a Qt‑friendly wrapper around two ROS 2 Trigger services used to start/stop
    an 'environment' (e.g., simulator bringup). Methods are Q_INVOKABLE for QML/Qt use.
Services:
  - /env/start and /env/stop (std_srvs/Trigger). `servicesAvailable()` reports discovery readiness.
Concurrency:
  - Owns a simple executor thread. Responses are marshalled back to the GUI thread via Qt's invokeMethod.
*/
#pragma once
#include <QObject>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class RosClient : public QObject {

/**
 * @class RosClient
 * @brief Thin wrapper around two std_srvs/Trigger services for start/stop.
 */

  Q_OBJECT
public:
  explicit RosClient(QObject* parent = nullptr);
  ~RosClient();

  Q_INVOKABLE void start();
  /// Asynchronously call /env/start. Emits startResult when done.
   // calls /env/start
  Q_INVOKABLE void stop();
  /// Asynchronously call /env/stop. Emits stopResult when done.
    // calls /env/stop
  bool servicesAvailable() const;
  /// Returns true once both services are discovered (ready).
 // true when both services are ready

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
