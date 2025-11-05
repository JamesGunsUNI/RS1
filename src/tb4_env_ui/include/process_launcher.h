/*
File: process_launcher.h
Role & context:
  - Declares ProcessLauncher, a Qt wrapper that starts and supervises a long‑running ROS 2 launch
    process (e.g., `ros2 launch pkg file.py ...`) and relays its stdout/stderr back to the UI.
Why this exists:
  - It is often simpler to run simulation/bringup using `ros2 launch` rather than embedding nodes
    in-process. This class encapsulates reliable start/stop semantics and signal handling.
Concurrency & signals:
  - Uses QProcess. Emits `runningChanged`, `outputLine`, and `finished` so the UI can reflect state,
    tail logs, and update buttons without blocking.
Shutdown strategy (graceful to forceful):
  1) Send SIGINT (Ctrl‑C) to request a clean shutdown (what ros2 launch expects).
  2) If needed, send SIGTERM.
  3) As a last resort, send SIGKILL to prevent orphaned processes.
Environment handling:
  - Spawns `/bin/bash -lc 'ros2 launch ...'` to inherit the user's ROS environment (PATH, setup.bash).
*/
#pragma once
#include <QObject>
#include <QProcess>  // <- important so QProcess::ExitStatus is known

class ProcessLauncher : public QObject {

/**
 * @class ProcessLauncher
 * @brief Starts/stops `ros2 launch` and surfaces its output to the UI.
 * @details Uses QProcess and emits high-level signals suitable for buttons and log panes.
 */

  Q_OBJECT
public:
  explicit ProcessLauncher(QObject* parent = nullptr);
  ~ProcessLauncher();

  // Start: runs `ros2 launch <pkg> <launch_file> <extra_args...>`
  bool startRos2Launch(
  /// Launches `ros2 launch <pkg> <file> <args...>` under /bin/bash -lc.
const QString& pkg,
                       const QString& launch_file,
                       const QStringList& extra_args);

  // Stop: send SIGINT first (Ctrl-C), then SIGTERM, then SIGKILL as a last resort
  void stop();
  /// Attempts graceful shutdown (SIGINT → SIGTERM → SIGKILL).

  bool isRunning() const;
  /// True if the child process is currently running.


signals:
  void runningChanged(bool running);
  void outputLine(QString line);
  void finished(int exitCode, QProcess::ExitStatus status);

private:
  QProcess* proc_ = nullptr;
  void hookSignals();
};
