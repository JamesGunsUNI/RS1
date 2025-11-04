#include "soil_moisture_dock.h"

SoilMoistureDock::SoilMoistureDock(QWidget* parent)
: QDockWidget(parent)
{
  setObjectName("dock_soil_moisture");
  setWindowTitle(tr("Soil Moisture"));
  setAllowedAreas(Qt::AllDockWidgetAreas);

  buildUi();

  connect(&uiTimer_, &QTimer::timeout, this, &SoilMoistureDock::onUiTick);
  uiTimer_.setInterval(100); // 10 Hz
  uiTimer_.start();

  startRos(); // assumes rclcpp::init() in your main already
}

SoilMoistureDock::~SoilMoistureDock() {
  stopRos();
}

void SoilMoistureDock::buildUi() {
  wrap_ = new QWidget(this);
  auto* v = new QVBoxLayout(wrap_);

  auto* row = new QHBoxLayout();
  row->addWidget(new QLabel(tr("Latest:"), wrap_));
  moistureLabel_ = new QLabel("--", wrap_);
  QFont f = moistureLabel_->font(); f.setBold(true); f.setPointSizeF(f.pointSizeF()+4);
  moistureLabel_->setFont(f);
  row->addWidget(moistureLabel_);
  row->addStretch();

  heatmapLabel_ = new QLabel(wrap_);
  heatmapLabel_->setMinimumSize(360, 360);
  heatmapLabel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  heatmapLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  heatmapLabel_->setAlignment(Qt::AlignCenter);

  v->addLayout(row);
  v->addWidget(heatmapLabel_, 1);
  wrap_->setLayout(v);
  setWidget(wrap_);
}

void SoilMoistureDock::startRos() {
  node_ = std::make_shared<rclcpp::Node>("ui_soil_bridge");
  exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec_->add_node(node_);

  sub_moist_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/soil_moisture", 10,
    [this](std_msgs::msg::Float32::SharedPtr m) {
      lastMoisture_.store(static_cast<float>(m->data), std::memory_order_relaxed);
      if (moistureLabel_) {
        QMetaObject::invokeMethod(moistureLabel_, [this, val = static_cast<float>(m->data)](){
          moistureLabel_->setText(QString::number(val, 'f', 3));
        }, Qt::QueuedConnection);
      }
      // also heartbeat in dock title
      QMetaObject::invokeMethod(this, [this, val = static_cast<float>(m->data)](){
        setWindowTitle(QStringLiteral("Soil Moisture — %1").arg(val, 0, 'f', 3));
      }, Qt::QueuedConnection);
    });

  sub_loc_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/soil_sample_location", 10,
    [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
      const float v = lastMoisture_.load(std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(heatmapMutex_);
      heatmap_.add(m->point.x, m->point.y, v);
    });

  rosThread_ = std::thread([this](){ exec_->spin(); });
}

void SoilMoistureDock::stopRos() {
  if (exec_) exec_->cancel();
  if (rosThread_.joinable()) rosThread_.join();
  exec_.reset();
  node_.reset();
}

void SoilMoistureDock::onUiTick() {
  if (!heatmapLabel_) return;
  QImage img;
  {
    std::lock_guard<std::mutex> lock(heatmapMutex_);
    img = heatmap_.toImage();
  }
  if (!img.isNull()) {
    const QSize target = heatmapLabel_->size();
    heatmapLabel_->setPixmap(QPixmap::fromImage(
      img.scaled(target, Qt::IgnoreAspectRatio, Qt::FastTransformation)));
  }
}
