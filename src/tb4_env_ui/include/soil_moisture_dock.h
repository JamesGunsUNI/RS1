/*
File: soil_moisture_dock.h
Role & context:
  - Declares SoilMoistureDock, a dockable Qt widget that displays the latest soil moisture value
    and a live heat map generated from ROS 2 topics `/soil_moisture` and `/soil_sample_location`.
Structure:
  - UI elements (labels, timers), an internal HeatmapGrid (sum/cnt), and ROS executor/subscriptions.
Update cadence:
  - A 10 Hz QTimer re-renders the grid into a QImage and scales it into the dock's label.
Threading:
  - A MultiThreadedExecutor spins in a std::thread. A std::mutex protects grid updates.
  - Atomics hold the last numeric moisture sample.
*/
#pragma once
#include <QDockWidget>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QPixmap>
#include <QSizePolicy>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class SoilMoistureDock : public QDockWidget {

/**
 * @class SoilMoistureDock
 * @brief Dockable panel showing latest moisture and a live heat map sourced from ROS 2 topics.
 */

  Q_OBJECT
public:
  explicit SoilMoistureDock(QWidget* parent = nullptr);
  ~SoilMoistureDock() override;

private slots:
  void onUiTick();
  /// Timer slot (10 Hz): re-render the grid and update the pixmap.


private:
  // ---- UI ----
  QWidget* wrap_{nullptr};
  QLabel*  moistureLabel_{nullptr};
  QLabel*  heatmapLabel_{nullptr};
  QTimer   uiTimer_;

  // ---- Heatmap state ----
  struct HeatmapGrid {
  /// Internal accumulation grid (sum/cnt) rendered to a QImage.

    double min_x{-10.0}, max_x{10.0}, min_y{-10.0}, max_y{10.0}, res{0.5};
    int nx{0}, ny{0};
    std::vector<float> sum, cnt;

    HeatmapGrid() { resize(min_x, max_x, min_y, max_y, res); }
    void resize(double minx, double maxx, double miny, double maxy, double resolution) {
      min_x=minx; max_x=maxx; min_y=miny; max_y=maxy; res=resolution;
      nx = int(std::ceil((max_x - min_x)/res));
      ny = int(std::ceil((max_y - min_y)/res));
      sum.assign(nx*ny, 0.f);
      cnt.assign(nx*ny, 0.f);
    }
    bool index(double x, double y, int& i) const {
      const int ix = int(std::floor((x - min_x)/res));
      const int iy = int(std::floor((y - min_y)/res));
      if (ix<0||iy<0||ix>=nx||iy>=ny) return false;
      i = iy*nx + ix; return true;
    }
    void add(double x, double y, float v){
      int i; if(!index(x,y,i)) return;
      sum[i] += v; cnt[i] += 1.f;
    }
    QImage toImage() const {
      if (nx<=0 || ny<=0) return QImage();
      QImage img(nx, ny, QImage::Format_ARGB32);
      for (int yy=0; yy<ny; ++yy){
        for (int xx=0; xx<nx; ++xx){
          const int i = yy*nx + xx;
          const float m = (cnt[i]>0.f) ? (sum[i]/cnt[i]) : 0.f; // 0..1
          const float c = std::clamp(m, 0.f, 1.f);
          int r,g,b;
          if (c < 0.3f){ float t=c/0.3f; r=int((0.8f+0.2f*t)*255); g=int((0.0f+0.4f*t)*255); b=0; }
          else if (c < 0.6f){ float t=(c-0.3f)/0.3f; r=int((1.0f-t)*255); g=int((0.4f+0.6f*t)*255); b=0; }
          else { float t=(c-0.6f)/0.4f; r=0; g=int((1.0f-0.3f*t)*255); b=int((0.7f*t)*255); }
          img.setPixel(xx, ny-1-yy, qRgba(r,g,b,216)); // flip Y, with alpha
        }
      }
      return img;
    }
  };

  HeatmapGrid        heatmap_{-10.0, 10.0, -10.0, 10.0, 0.5};
  std::mutex         heatmapMutex_;
  std::atomic<float> lastMoisture_{0.f};

  // ---- ROS ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_moist_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_loc_;
  std::thread rosThread_;

  void buildUi();
  void startRos();
  void stopRos();
};
