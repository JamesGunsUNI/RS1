/*
File: ros_image_bridge.cpp
High‑level behavior:
  - Creates a ROS 2 node and SingleThreadedExecutor on demand, subscribes to an image_transport topic,
    decodes incoming sensor_msgs/Image into QImage, and emits a Qt signal on the GUI thread.
Image decoding details:
  - Prefers BGR8 (common in OpenCV), falls back to the message's native encoding.
  - Handles grayscale and RGB/BGRA cases; uses OpenCV color conversions where necessary.
Threading:
  - The ROS executor spins in `spin_thread_`. Frames are posted to the GUI using a QueuedConnection.
Failure modes:
  - Any per‑frame conversion errors are caught and dropped to avoid log spam or crashes.
*/
#include "ros_image_bridge.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <QMetaObject>

RosImageBridge::RosImageBridge(QObject* parent) : QObject(parent) {}
/** Constructor: initializes QObject base; node/executor are created lazily in ensureNode(). */

RosImageBridge::~RosImageBridge() { stop(); }
/** Destructor: ensures subscription/executor are stopped and thread is joined. */

void RosImageBridge::ensureNode() {

/** @brief Lazily allocate the ROS node, executor, spinning thread, and image transport.
 *  Rationale: Defers ROS creation cost until the first call to start(), keeping startup light.
 */

  if (node_) return;
  node_ = rclcpp::Node::make_shared("tb4_env_ui_img");
  exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([this]{ exec_->spin(); });
  it_ = std::make_unique<image_transport::ImageTransport>(node_);
}

void RosImageBridge::start(

/** @brief (Re)subscribe to a given image topic and forward frames to Qt as QImage.
 *  Notes:
 *   - Tries BGR8 first to minimize copying with OpenCV; falls back to native encodings.
 *   - Uses QMetaObject::invokeMethod with QueuedConnection to hop back to the GUI thread.
 */
const std::string& topic) {
  ensureNode();

  // unsubscribe any previous
  sub_.shutdown();

  sub_ = it_->subscribe(topic, 1,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
      try {
        // Try BGR first (works well with OpenCV + Qt)
        cv_bridge::CvImageConstPtr cvp;
        try { cvp = cv_bridge::toCvShare(msg, "bgr8"); }
        catch (...) { cvp = cv_bridge::toCvShare(msg, msg->encoding); }

        cv::Mat mat = cvp->image;
        QImage img;

        if (mat.type() == CV_8UC3) {
          img = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888).copy();
        } else if (mat.type() == CV_8UC1) {
          img = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8).copy();
        } else {
          // Fallback: convert to RGB888
          cv::Mat tmp;
          if (mat.channels() == 4) cv::cvtColor(mat, tmp, cv::COLOR_BGRA2BGR);
          else if (mat.channels() == 3) cv::cvtColor(mat, tmp, cv::COLOR_BGR2RGB);
          else cv::cvtColor(mat, tmp, cv::COLOR_GRAY2RGB);
          img = QImage(tmp.data, tmp.cols, tmp.rows, tmp.step, QImage::Format_RGB888).copy();
        }

        QMetaObject::invokeMethod(this, [this, img]() {
          emit frameReady(img);
        }, Qt::QueuedConnection);
      } catch (const std::exception& e) {
        // swallow conversion errors to avoid spamming
      }
    });
}

void RosImageBridge::stop() {
/** @brief Stop subscription and tear down executor/thread safely (idempotent). */
  sub_.shutdown();
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  it_.reset();
  exec_.reset();
  node_.reset();
}
