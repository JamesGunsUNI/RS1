#pragma once
#include <QWidget>
#include <QString>

class QLabel;
class QPushButton;
class QFrame;
class QPlainTextEdit;
class ProcessLauncher;
class ImageWidget;
class RosImageBridge;

class MainWindow : public QWidget
{
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);

private slots:
  void onStartClicked();
  void onStopClicked();
  

private:
  void setStatusDot(const QString& stateKey);
  void setStatusText(const QString& text);

  // UI
  QLabel*         statusLabel_{};
  QFrame*         statusDot_{};
  QPushButton*    startBtn_{};
  QPushButton*    stopBtn_{};
  QPlainTextEdit* log_{};
  ImageWidget*    camView_{};
  // New placeholders for the extra buttons
  QPushButton* startScriptBtn_   = nullptr;  // left, row 1
  QPushButton* manualControlBtn_ = nullptr;  // right, row 1

  // Backend
  ProcessLauncher* launcher_{};
  RosImageBridge*  img_{};
  QString          cameraTopic_{"/camera/image"}; // change to "/camera/depth/image" if you prefer
};
