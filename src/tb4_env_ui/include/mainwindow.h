#pragma once

#include <QWidget>
#include <QString>
#include <QTimer>
#include <QLabel>
#include <QMutex>
#include <QImage>
#include <QSize>

#include <atomic>
#include <vector>
#include <thread>
#include <cmath>
#include <algorithm>

// ROS 2 for soil topics
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class QLabel;
class QPushButton;
class QFrame;
class QPlainTextEdit;
class QVBoxLayout;     // <-- for the new buildSoilSection signature
class ProcessLauncher;
class ImageWidget;
class RosImageBridge;

class MainWindow : public QWidget
{
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);

private slots:
  // original two buttons
  void onStartClicked();
  void onStopClicked();

  // heat-map UI thread slots
  void onMoisture(float v);
  void onSampleXY(double x, double y);
  void onUiTick();

signals:
  // cross-thread (ROS → UI)
  void moistureReceived(float v);
  void sampleXYReceived(double x, double y);

private:
  void setStatusDot(const QString& stateKey);
  void setStatusText(const QString& text);

  // ---- Soil helpers ----
  void buildSoilSection(QVBoxLayout* column);   // <-- place heat-map into the given column
  void startSoilSubscriptions();
  void stopSoilSubscriptions();

  // ---- UI (same design as your original) ----
  QLabel*         statusLabel_{};      // “Status: …”
  QFrame*         statusDot_{};        // green/red dot
  QPushButton*    startBtn_{};         // Start environment
  QPushButton*    stopBtn_{};          // Stop environment
  QPlainTextEdit* log_{};              // log console

  // Optional placeholders you had in the “extra row”
  QPushButton* startScriptBtn_   = nullptr;
  QPushButton* manualControlBtn_ = nullptr;

  // Camera
  ImageWidget*    camView_{};          // your viewer widget
  RosImageBridge* img_{};              // your ROS image bridge
  QString         cameraTopic_{"/camera/image"}; // change if your topic differs

  // Backend
  ProcessLauncher* launcher_{};

  // ---- Soil heat-map section (compact, no docks) ----
  struct HeatmapGrid {
    double min_x{-10.0}, max_x{10.0}, min_y{-10.0}, max_y{10.0}, res{0.5};
    int nx{0}, ny{0};
    std::vector<float> sum, cnt;

    HeatmapGrid() { resize(min_x,max_x,min_y,max_y,res); }
    HeatmapGrid(double a,double b,double c,double d,double e){ resize(a,b,c,d,e); }

    void resize(double minx,double maxx,double miny,double maxy,double r){
      min_x=minx; max_x=maxx; min_y=miny; max_y=maxy; res=r;
      nx = int(std::ceil((max_x-min_x)/res));
      ny = int(std::ceil((max_y-min_y)/res));
      sum.assign(nx*ny, 0.f);
      cnt.assign(nx*ny, 0.f);
    }
    bool index(double x,double y,int& i) const {
      const int ix = int(std::floor((x - min_x)/res));
      const int iy = int(std::floor((y - min_y)/res));
      if (ix<0||iy<0||ix>=nx||iy>=ny) return false;
      i = iy*nx + ix; return true;
    }
    void add(double x,double y,float v){ int i; if(index(x,y,i)){ sum[i]+=v; cnt[i]+=1.f; } }
    QImage toImage() const {
      if (nx<=0||ny<=0) return QImage();
      QImage img(nx, ny, QImage::Format_ARGB32);
      for (int y=0; y<ny; ++y){
        for (int x=0; x<nx; ++x){
          const int idx = y*nx + x;
          const float m = (cnt[idx]>0.f) ? (sum[idx]/cnt[idx]) : 0.f; // 0..1 expected
          const float c = std::clamp(m, 0.f, 1.f);
          int r,g,b;
          if (c < 0.3f){ float t=c/0.3f; r=int((0.8f+0.2f*t)*255); g=int((0.0f+0.4f*t)*255); b=0; }
          else if (c < 0.6f){ float t=(c-0.3f)/0.3f; r=int((1.0f-t)*255); g=int((0.4f+0.6f*t)*255); b=0; }
          else { float t=(c-0.6f)/0.4f; r=0; g=int((1.0f-0.3f*t)*255); b=int((0.7f*t)*255); }
          img.setPixel(x, ny-1-y, qRgba(r,g,b,216)); // flip Y, some alpha
        }
      }
      return img;
    }
  };

  QLabel*     soilTitle_{};            // “Soil Moisture”
  QLabel*     heatmapLabel_{};         // QImage display (fixed size)
  const QSize heatmapSize_{360, 360};  // fixed size keeps layout stable
  QTimer      uiTimer_;
  HeatmapGrid heatmap_{-10.0, 10.0, -10.0, 10.0, 0.5};
  QMutex      heatmapMutex_;
  std::atomic<float> lastMoisture_{0.f};

  // ROS soil
  rclcpp::Node::SharedPtr node_soil_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_soil_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_moist_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_loc_;
  std::thread ros_soil_thread_;
};
