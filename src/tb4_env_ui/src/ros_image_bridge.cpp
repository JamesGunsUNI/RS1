#include "ros_image_bridge.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <QMetaObject>

RosImageBridge::RosImageBridge(QObject* parent) : QObject(parent) {}

RosImageBridge::~RosImageBridge() { stop(); }

void RosImageBridge::ensureNode() {
  if (node_) return;
  node_ = rclcpp::Node::make_shared("tb4_env_ui_img");
  exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([this]{ exec_->spin(); });
  it_ = std::make_unique<image_transport::ImageTransport>(node_);
}

void RosImageBridge::start(const std::string& topic) {
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
  sub_.shutdown();
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  it_.reset();
  exec_.reset();
  node_.reset();
}
