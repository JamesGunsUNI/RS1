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
#include <QDebug>

static QString dotColour(const QString& state)
{
  if (state == "running")  return "#10b981"; // green
  if (state == "stopped")  return "#ef4444"; // red
  return "#6b7280";                          // idle grey
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

  // ----- New row for extra buttons
  auto* row2 = new QHBoxLayout();
  row2->setSpacing(12);
  
  startScriptBtn_   = new QPushButton("Start Script", this);
  manualControlBtn_ = new QPushButton("Manual Control", this);
  for (auto* b : { startScriptBtn_, manualControlBtn_ }) {
    b->setMinimumHeight(56);
    b->setCursor(Qt::PointingHandCursor);
  }
  // Initially disabled; enable when functionality is implemented 
  startScriptBtn_->setEnabled(false);
  manualControlBtn_->setEnabled(false);
  row2->addWidget(startScriptBtn_);
  row2->addWidget(manualControlBtn_);
  root->addLayout(row2);
  

  // ----- Log output
  log_ = new QPlainTextEdit(this);
  log_->setReadOnly(true);
  log_->setMinimumHeight(200);
  root->addWidget(log_);

  // ----- Camera label + view  (this must be INSIDE the constructor)
  auto* camLabel = new QLabel("Camera", this);
  camLabel->setStyleSheet("font-size:16px; font-weight:600; margin-top:8px;");
  root->addWidget(camLabel);

  camView_ = new ImageWidget(this);
  camView_->setMinimumHeight(320);
  root->addWidget(camView_);

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
  )CSS");

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

  // ----- ROS image subscriber (starts now, shows frames when topic is live)
  img_ = new RosImageBridge(this);
  connect(img_, &RosImageBridge::frameReady, camView_, &ImageWidget::setImage);
  img_->start(cameraTopic_.toStdString());  // defaults to "/camera/image"
}

void MainWindow::onStartClicked()
{
  if (launcher_->isRunning()) return;

  // Your exact command:
  // ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
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
