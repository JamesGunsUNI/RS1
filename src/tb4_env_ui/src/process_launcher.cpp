/*
File: process_launcher.cpp
High‑level behavior:
  - Starts `ros2 launch ...` under /bin/bash -lc so user shell setup (ROS overlays, PATH, hooks) applies.
  - Streams both stdout and stderr line‑by‑line to a single `outputLine` signal for a simple UI console.
  - Tracks lifecycle and exposes a reliable stop() that escalates from SIGINT to SIGTERM to SIGKILL.
Design trade‑offs:
  - Using an external bringup keeps the UI process clean, avoids node lifetime tangles, and matches
    how ROS 2 users usually work in terminals.
*/
#include "process_launcher.h"
#include <QProcessEnvironment>
#include <QStringList>

// POSIX bits for signals
#ifdef Q_OS_UNIX
  #include <signal.h>
  #include <unistd.h>
#endif

ProcessLauncher::ProcessLauncher(QObject* parent) : QObject(parent), proc_(nullptr) {}

ProcessLauncher::~ProcessLauncher() {
  stop();
}

void ProcessLauncher::hookSignals() {
  connect(proc_, &QProcess::readyReadStandardOutput, this, [this]() {
    const QString out = QString::fromLocal8Bit(proc_->readAllStandardOutput());
    for (const auto& line : out.split('\n', Qt::SkipEmptyParts)) emit outputLine(line);
  });
  connect(proc_, &QProcess::readyReadStandardError, this, [this]() {
    const QString out = QString::fromLocal8Bit(proc_->readAllStandardError());
    for (const auto& line : out.split('\n', Qt::SkipEmptyParts)) emit outputLine(line);
  });
  connect(proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          this, [this](int code, QProcess::ExitStatus st){
            emit finished(code, st);
            emit runningChanged(false);
          });
}

bool ProcessLauncher::startRos2Launch(

/** @brief Start `ros2 launch` in a login shell so ROS setup files are honored.
 *  Streams both stdout and stderr to the UI by merging channels.
 */
const QString& pkg,
                                      const QString& launch_file,
                                      const QStringList& extra_args)
{
  if (isRunning()) return false;

  // Build: exec ros2 launch <pkg> <launch_file> <args...>
  QString cmd = "exec ros2 launch " + pkg + " " + launch_file;
  for (const auto& a : extra_args) cmd += " " + a;

  proc_ = new QProcess(this);
  proc_->setProcessChannelMode(QProcess::MergedChannels);
  proc_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  hookSignals();

  // Run via bash -lc so we inherit the user's ROS environment & PATH
  proc_->setProgram("/bin/bash");
  proc_->setArguments({ "-lc", cmd });

  proc_->start();
  const bool started = proc_->waitForStarted(3000);
  if (started) emit runningChanged(true);
  else {
    emit outputLine("Failed to start: ensure this UI is run from a shell with ROS 2 sourced.");
    proc_->deleteLater();
    proc_ = nullptr;
  }
  return started;
}

void ProcessLauncher::stop() {

/** @brief Attempt graceful shutdown first (SIGINT), then escalate to SIGTERM/SIGKILL.
 *  This mirrors how users stop ros2 launch in a terminal with Ctrl-C.
 */

  if (!proc_) return;

#ifdef Q_OS_UNIX
  // 1) Try Ctrl-C (SIGINT) on the actual ros2 launch PID.
  const qint64 qpid = proc_->processId();
  if (qpid > 0) {
    ::kill(static_cast<pid_t>(qpid), SIGINT);
    if (proc_->waitForFinished(3000)) {
      proc_->deleteLater();
      proc_ = nullptr;
      emit runningChanged(false);
      return;
    }
  }
#endif

  // 2) Try graceful terminate (SIGTERM)
  proc_->terminate();
  if (proc_->waitForFinished(3000)) {
    proc_->deleteLater();
    proc_ = nullptr;
    emit runningChanged(false);
    return;
  }

  // 3) Last resort: SIGKILL
  proc_->kill();
  proc_->waitForFinished(2000);
  proc_->deleteLater();
  proc_ = nullptr;
  emit runningChanged(false);
}

bool ProcessLauncher::isRunning() const {
  return proc_ && proc_->state() != QProcess::NotRunning;
}
