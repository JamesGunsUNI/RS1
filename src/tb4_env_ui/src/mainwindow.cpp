#include "mainwindow.h"
#include "process_launcher.h"
#include "ros_image_bridge.h"
#include "image_widget.h"

#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFrame>
#include <QShortcut>
#include <QKeySequence>
#include <QPlainTextEdit>
#include <QDateTime>
#include <QScrollBar>
#include <QMessageBox>
#include <QPixmap>
#include <QTextCursor>
#include <QSizePolicy>
#include <QPainter>
#include <QMouseEvent>
#include <QToolTip>
#include <QMutexLocker>

// ---------------- HeatmapWidget impl ----------------
void HeatmapWidget::paintEvent(QPaintEvent*)
{
  QPainter p(this);
  p.setRenderHint(QPainter::SmoothPixmapTransform, false);

  if (!img_.isNull()) {
    // scale to widget rect
    p.drawImage(rect(), img_);
  } else {
    // placeholder
    p.fillRect(rect(), QColor("#0b1220"));
    p.setPen(QColor("#6b7280"));
    p.drawText(rect(), Qt::AlignCenter, "No heat map data yet");
  }
}

bool HeatmapWidget::sampleAt(const QPoint& pos, double& wx, double& wy, float& value, bool& hasData) const
{
  if (!gv_.nx || !gv_.ny || !gv_.sum || !gv_.cnt || !gv_.min_x || !gv_.min_y || !gv_.res) return false;

  const int W = width();
  const int H = height();
  if (W <= 0 || H <= 0) return false;

  const int nx = *gv_.nx, ny = *gv_.ny;
  if (nx <= 0 || ny <= 0) return false;

  // map pixel -> cell coords
  int cx = int(std::floor(double(pos.x()) * nx / double(W)));
  int cy_img = int(std::floor(double(pos.y()) * ny / double(H)));
  int cy = ny - 1 - cy_img; // invert Y to match toImage flip

  if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) return false;
  const int idx = cy*nx + cx;

  // world coords (cell center)
  const double res = *gv_.res;
  wx = *gv_.min_x + (cx + 0.5) * res;
  wy = *gv_.min_y + (cy + 0.5) * res;

  // read value with a short lock
  float c = 0.f;
  float val = 0.f;
  if (gv_.mutex) {
    QMutexLocker locker(gv_.mutex);
    c   = (gv_.cnt->at(idx) > 0.f) ? gv_.cnt->at(idx) : 0.f;
    val = (c > 0.f) ? (gv_.sum->at(idx) / c) : 0.f;
  } else {
    c   = (gv_.cnt->at(idx) > 0.f) ? gv_.cnt->at(idx) : 0.f;
    val = (c > 0.f) ? (gv_.sum->at(idx) / c) : 0.f;
  }

  value   = val;
  hasData = (c > 0.f);
  return true;
}

void HeatmapWidget::mouseMoveEvent(QMouseEvent* ev)
{
  double wx, wy; float v; bool hasData;
  if (sampleAt(ev->pos(), wx, wy, v, hasData)) {
    const QString txt = hasData
      ? QString("Moisture: %1").arg(QString::number(v, 'f', 3))
      : QString("(no data)");
    QToolTip::showText(ev->globalPos(), txt, this);
  } else {
    QToolTip::hideText();
  }
}


// ---------------- MainWindow impl ----------------

static QString dotColour(const QString& state)
{
  if (state == "running")  return "#10b981"; // green
  if (state == "stopped")  return "#ef4444"; // red
  return "#6b7280";                          // idle grey
}

static QString ts()
{
  return QDateTime::currentDateTime().toString("hh:mm:ss");
}

MainWindow::MainWindow(QWidget* parent) : QWidget(parent)
{
  setWindowTitle("TB4 Environment Controller");
  resize(900, 700);

  // ----- Layout root
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(20, 20, 20, 20);
  root->setSpacing(16);

  // ----- Title
  auto* title = new QLabel("TB4 Environment Controller", this);
  title->setStyleSheet("font-size:22px; font-weight:600;");
  root->addWidget(title);

  // ----- Status row
  auto* statusRow = new QHBoxLayout();
  statusRow->setSpacing(8);

  statusDot_ = new QFrame(this);
  statusDot_->setFixedSize(14, 14);
  statusDot_->setFrameShape(QFrame::NoFrame);
  setStatusDot("stopped");

  statusLabel_ = new QLabel("Status: Stopped", this);
  statusLabel_->setStyleSheet("font-size:14px; color:#e5e7eb;");

  statusRow->addWidget(statusDot_, 0, Qt::AlignVCenter);
  statusRow->addWidget(statusLabel_, 0, Qt::AlignVCenter);
  statusRow->addStretch(1);
  root->addLayout(statusRow);

  // ----- Buttons
  auto* btnRow = new QHBoxLayout();
  btnRow->setSpacing(12);

  startBtn_ = new QPushButton("Start environment", this);
  stopBtn_  = new QPushButton("Stop environment", this);

  for (auto* b : { startBtn_, stopBtn_ }) {
    b->setMinimumHeight(56);
    b->setCursor(Qt::PointingHandCursor);
  }

  new QShortcut(QKeySequence(Qt::Key_S), this, SLOT(onStartClicked()));
  new QShortcut(QKeySequence(Qt::Key_X), this, SLOT(onStopClicked()));

  connect(startBtn_, &QPushButton::clicked, this, &MainWindow::onStartClicked);
  connect(stopBtn_,  &QPushButton::clicked, this, &MainWindow::onStopClicked);

  stopBtn_->setEnabled(false);

  btnRow->addWidget(startBtn_);
  btnRow->addWidget(stopBtn_);
  root->addLayout(btnRow);

  // ----- Second row (placeholders, still disabled)
  auto* row2 = new QHBoxLayout();
  row2->setSpacing(12);
  startScriptBtn_   = new QPushButton("Start Script", this);
  manualControlBtn_ = new QPushButton("Manual Control", this);
  for (auto* b : { startScriptBtn_, manualControlBtn_ }) {
    b->setMinimumHeight(56);
    b->setCursor(Qt::PointingHandCursor);
    b->setEnabled(false);
  }
  row2->addWidget(startScriptBtn_);
  row2->addWidget(manualControlBtn_);
  root->addLayout(row2);

  // ----- Log output
  log_ = new QPlainTextEdit(this);
  log_->setReadOnly(true);
  log_->setMinimumHeight(200);
  root->addWidget(log_);

  // ====== MEDIA ROW: Camera (left) | Heat-map (right) ======
  auto* mediaRow = new QHBoxLayout();
  mediaRow->setSpacing(16);

  // Left column: Camera
  auto* camCol = new QVBoxLayout();
  auto* camLabel = new QLabel("Camera", this);
  camLabel->setStyleSheet("font-size:16px; font-weight:600; margin-top:8px;");
  camCol->addWidget(camLabel);

  camView_ = new ImageWidget(this);
  camView_->setMinimumSize(480, 360);
  camView_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  camCol->addWidget(camView_, 1);

  mediaRow->addLayout(camCol, 1);   // stretch = 1 (expands)

  // Right column: Soil Moisture (fixed-width heat-map)
  auto* soilCol = new QVBoxLayout();
  buildSoilSection(soilCol);        // creates title + fixed 360x360 heatmap
  soilCol->addStretch(1);
  mediaRow->addLayout(soilCol);

  root->addLayout(mediaRow);

  // ----- Styling
  setStyleSheet(R"CSS(
    QWidget { background-color:#111827; color:#f3f4f6; }
    QPushButton {
      border:1px solid #374151;
      border-radius:12px;
      padding:10px 18px;
      font-size:16px;
      background:#1f2937;
    }
    QPushButton:hover { background:#273245; }
    QPushButton:pressed { background:#2e3a4f; }
    QPushButton:disabled { background:#1b2431; color:#9ca3af; border-color:#2b3442; }
    QPlainTextEdit {
      background:#0f172a; border:1px solid #374151; border-radius:12px; padding:12px; font-family:monospace;
    }
    QLabel.sectionTitle { font-size:16px; font-weight:600; margin-top:8px; }
  )CSS");

  {
  QString ss = qApp->styleSheet();
  ss += " QToolTip { color:#ffffff; background-color:rgba(0,0,0,220); "
        "border:1px solid #ffffff; padding:4px 6px; border-radius:6px; }";
  qApp->setStyleSheet(ss);
}

  // ----- Launcher wiring
  launcher_ = new ProcessLauncher(this);
  connect(launcher_, &ProcessLauncher::runningChanged, this, [this](bool running){
    startBtn_->setEnabled(!running);
    stopBtn_->setEnabled(running);
    setStatusDot(running ? "running" : "stopped");
    setStatusText(running ? "Status: Running" : "Status: Stopped");
  });
  connect(launcher_, &ProcessLauncher::outputLine, this, [this](const QString& line){
    log_->appendPlainText(line);
  });
  connect(launcher_, &ProcessLauncher::finished, this, [this](int code, QProcess::ExitStatus){
    log_->appendPlainText(QString("Process finished. Exit code %1").arg(code));
  });

  // ----- ROS image subscriber
  img_ = new RosImageBridge(this);
  connect(img_, &RosImageBridge::frameReady, camView_, &ImageWidget::setImage);
  img_->start(cameraTopic_.toStdString());

  // ----- Soil ROS + timer
  connect(this, &MainWindow::moistureReceived,  this, &MainWindow::onMoisture,  Qt::QueuedConnection);
  connect(this, &MainWindow::sampleXYReceived, this, &MainWindow::onSampleXY, Qt::QueuedConnection);
  uiTimer_.setInterval(100);
  connect(&uiTimer_, &QTimer::timeout, this, &MainWindow::onUiTick);
  uiTimer_.start();
  startSoilSubscriptions();
}

void MainWindow::onStartClicked()
{
  if (launcher_->isRunning()) return;

  const QString pkg = "41068_ignition_bringup";
  const QString launch_file = "41068_ignition.launch.py";
  const QStringList extra_args = {"slam:=true", "nav2:=true", "rviz:=true", "world:=large_demo"};

  log_->appendPlainText("Starting environment...");
  const bool ok = launcher_->startRos2Launch(pkg, launch_file, extra_args);
  if (!ok) {
    log_->appendPlainText("Failed to start. Make sure this UI is run from a ROS 2 sourced shell.");
  }
}

void MainWindow::onStopClicked()
{
  if (!launcher_->isRunning()) return;
  log_->appendPlainText("Stopping environment...");
  launcher_->stop();
}

void MainWindow::setStatusDot(const QString& stateKey)
{
  const QString c = dotColour(stateKey);
  statusDot_->setStyleSheet(
    QString("background-color:%1; border-radius:7px; border:1px solid rgba(0,0,0,0.15);").arg(c)
  );
}

void MainWindow::setStatusText(const QString& text)
{
  statusLabel_->setText(text);
}

// -------- Soil UI & ROS ----------

void MainWindow::buildSoilSection(QVBoxLayout* column)
{
  soilTitle_ = new QLabel("Soil Moisture", this);
  soilTitle_->setObjectName("soilTitle");
  soilTitle_->setProperty("class", "sectionTitle");
  column->addWidget(soilTitle_);

  // Custom widget with hover tooltips
  heatmapView_ = new HeatmapWidget(this);
  heatmapView_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  heatmapView_->setFixedSize(heatmapSize_);

  // crisp white border + black background
  heatmapView_->setStyleSheet("background:#000; border:2px solid #ffffff; border-radius:12px;");

  // Bind grid pointers for hover sampling
  HeatmapWidget::GridView gv;
  gv.min_x = &heatmap_.min_x; gv.min_y = &heatmap_.min_y; gv.res = &heatmap_.res;
  gv.nx = &heatmap_.nx; gv.ny = &heatmap_.ny;
  gv.sum = &heatmap_.sum; gv.cnt = &heatmap_.cnt;
  gv.mutex = &heatmapMutex_;
  heatmapView_->bindGrid(gv);

  // kick an initial frame so the widget paints right away
  heatmapView_->setImage(heatmap_.toImage());

  column->addWidget(heatmapView_);
}

void MainWindow::startSoilSubscriptions()
{
  node_soil_ = std::make_shared<rclcpp::Node>("ui_soil_bridge");
  exec_soil_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec_soil_->add_node(node_soil_);

  sub_moist_ = node_soil_->create_subscription<std_msgs::msg::Float32>(
    "/soil_moisture", 10,
    [this](std_msgs::msg::Float32::SharedPtr m) {
      emit moistureReceived(static_cast<float>(m->data));
    });

  sub_loc_ = node_soil_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/soil_sample_location", 10,
    [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
      emit sampleXYReceived(m->point.x, m->point.y);
    });

  ros_soil_thread_ = std::thread([this]() { exec_soil_->spin(); });
}

void MainWindow::stopSoilSubscriptions()
{
  if (exec_soil_) exec_soil_->cancel();
  if (ros_soil_thread_.joinable()) ros_soil_thread_.join();
  exec_soil_.reset();
  node_soil_.reset();
}

void MainWindow::onMoisture(float v)
{
  lastMoisture_.store(v, std::memory_order_relaxed);
}

void MainWindow::onSampleXY(double x, double y)
{
  const float v = lastMoisture_.load(std::memory_order_relaxed);
  QMutexLocker lock(&heatmapMutex_);
  heatmap_.add(x, y, v);
}

void MainWindow::onUiTick()
{
  if (!heatmapView_) return;

  QImage img;
  {
    QMutexLocker lock(&heatmapMutex_);
    img = heatmap_.toImage();
  }
  // give raw grid image; widget scales it to its fixed rect
  heatmapView_->setImage(img);
}
