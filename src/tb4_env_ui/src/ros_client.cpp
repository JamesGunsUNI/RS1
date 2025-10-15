#include "ros_client.h"
#include <QMetaObject>
#include <QString>

using namespace std::chrono_literals;

static constexpr const char* START_SRV = "/env/start"; // change if needed
static constexpr const char* STOP_SRV  = "/env/stop";  // change if needed

RosClient::RosClient(QObject* parent) : QObject(parent) {
  node_ = rclcpp::Node::make_shared("tb4_env_ui_node");
  start_client_ = node_->create_client<std_srvs::srv::Trigger>(START_SRV);
  stop_client_  = node_->create_client<std_srvs::srv::Trigger>(STOP_SRV);

  exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([this]{ exec_->spin(); });
}

RosClient::~RosClient() {
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

bool RosClient::servicesAvailable() const {
  // service_is_ready() becomes true once a server is discovered
  return start_client_ && stop_client_
      && start_client_->service_is_ready()
      && stop_client_->service_is_ready();
}

void RosClient::start() {
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Wait up to ~5s for the service (discovery can be slow on some setups)
  bool ready = false;
  for (int i = 0; i < 20; ++i) {
    if (start_client_->wait_for_service(250ms)) { ready = true; break; }
    if (!rclcpp::ok()) { emit startResult(false, "ROS shutdown"); return; }
  }
  if (!ready) { emit startResult(false, "Start service not available"); return; }

  auto fut = start_client_->async_send_request(req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
      const auto resp = f.get();
      QMetaObject::invokeMethod(this, [this, resp](){
        emit startResult(resp->success, QString::fromStdString(resp->message));
      }, Qt::QueuedConnection);
    });
  (void)fut;
}

void RosClient::stop() {
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  bool ready = false;
  for (int i = 0; i < 20; ++i) {
    if (stop_client_->wait_for_service(250ms)) { ready = true; break; }
    if (!rclcpp::ok()) { emit stopResult(false, "ROS shutdown"); return; }
  }
  if (!ready) { emit stopResult(false, "Stop service not available"); return; }

  auto fut = stop_client_->async_send_request(req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
      const auto resp = f.get();
      QMetaObject::invokeMethod(this, [this, resp](){
        emit stopResult(resp->success, QString::fromStdString(resp->message));
      }, Qt::QueuedConnection);
    });
  (void)fut;
}
