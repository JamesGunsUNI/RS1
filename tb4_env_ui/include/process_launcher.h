#pragma once
#include <QObject>
#include <QProcess>  // <- important so QProcess::ExitStatus is known

class ProcessLauncher : public QObject {
  Q_OBJECT
public:
  explicit ProcessLauncher(QObject* parent = nullptr);
  ~ProcessLauncher();

  // Start: runs `ros2 launch <pkg> <launch_file> <extra_args...>`
  bool startRos2Launch(const QString& pkg,
                       const QString& launch_file,
                       const QStringList& extra_args);

  // Stop: send SIGINT first (Ctrl-C), then SIGTERM, then SIGKILL as a last resort
  void stop();
  bool isRunning() const;

signals:
  void runningChanged(bool running);
  void outputLine(QString line);
  void finished(int exitCode, QProcess::ExitStatus status);

private:
  QProcess* proc_ = nullptr;
  void hookSignals();
};
