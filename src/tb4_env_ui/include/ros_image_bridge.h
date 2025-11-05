/*
File: ros_image_bridge.h
Role & context:
  - Declares RosImageBridge, a small Qt-friendly adapter that subscribes to a ROS 2 image topic
    and emits decoded QImage frames to the UI thread.
  - Bridges three ecosystems: ROS 2 (rclcpp, image_transport), OpenCV (cv_bridge), and Qt (QObject/QImage).
Why this exists:
  - Typical Qt widgets need QImage/QPixmap, while ROS cameras publish sensor_msgs/Image.
    This class hides transport, color encoding, and threading details and exposes a single
    Qt signal `frameReady(QImage)` that can be connected to any display widget.
Concurrency model:
  - Internally spins a dedicated ROS executor thread so the Qt event loop remains responsive.
  - Uses Qt's QueuedConnection (via QMetaObject::invokeMethod in the .cpp) to marshal frames
    from the ROS thread back to the GUI thread safely.
How to use:
  - Call start("/camera/image") to begin subscribing; connect frameReady to your viewer.
  - Call stop() to unsubscribe and tear down the executor thread.
Key design notes:
  - SingleThreadedExecutor is sufficient because the bridge only owns one node and one subscriber.
  - image_transport is used so this will also work with compressed transports if available.
*/
#pragma once
#include <QObject>
#include <QImage>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class RosImageBridge : public QObject {

/**
 * @class RosImageBridge
 * @brief Subscribes to a ROS 2 image topic and emits Qt QImage frames via `frameReady`.
 * @details
 *   - Owns its rclcpp::Node and a SingleThreadedExecutor spun in a std::thread.
 *   - Uses image_transport for transport flexibility and cv_bridge for decoding.
 */

  Q_OBJECT
public:
  explicit RosImageBridge(QObject* parent = nullptr);
  ~RosImageBridge();

  void start(
  /// Begin subscribing on the given ROS image topic (e.g., "/camera/image").
const std::string& topic);  // subscribe to a camera topic
  void stop();
  /// Unsubscribe and tear down the executor/thread (idempotent).


signals:
  void frameReady(
  /// Emitted on the GUI thread when a new frame is available.
const QImage& img);

private:
  void ensureNode();

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_;
};
