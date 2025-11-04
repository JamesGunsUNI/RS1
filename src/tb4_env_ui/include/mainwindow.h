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
class QVBoxLayout;
class ProcessLauncher;
class ImageWidget;
class RosImageBridge;

// ---------------- HeatmapWidget (for hover tooltip) ----------------
class HeatmapWidget : public QWidget {
  Q_OBJECT
public:
  explicit HeatmapWidget(QWidget* parent=nullptr) : QWidget(parent) {
    setMouseTracking(true);
  }

  struct GridView {
    // read-only view of the grid
    const double* min_x{};
    const double* min_y{};
    const double* res{};
    const int* nx{};
    const int* ny{};
    const std::vector<float>* sum{};
    const std::vector<float>* cnt{};
    QMutex* mutex{};
  };

  void bindGrid(const GridView& gv) { gv_ = gv; }
  void setImage(const QImage& img)  { img_ = img; update(); }

protected:
  void paintEvent(QPaintEvent*) override;
  void mouseMoveEvent(QMouseEvent* ev) override;

private:
  bool sampleAt(const QPoint& pos, double& wx, double& wy, float& value, bool& hasData) const;

  QImage  img_;
  GridView gv_{};
};

// ---------------- MainWindow ----------------
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
  void buildSoilSection(QVBoxLayout* column);   // place heat-map into the given column
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

public:
  // ---- Soil heat-map data & rendering ----
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

    // Perceptual gradient (viridis-like)
    // Per-cell color: red (0.0) → blue (1.0)
    static QRgb ramp(float t) {
      t = std::clamp(t, 0.0f, 1.0f);
      struct C { float r,g,b; };
      // RdBu-ish stops (dark red → salmon → light → sky → dark blue)
      static constexpr C stops[] = {
        {0.698f, 0.094f, 0.125f}, // #b2182b
        {0.937f, 0.541f, 0.384f}, // #ef8a62
        {0.969f, 0.855f, 0.780f}, // #fddbc7  (near-white midpoint)
        {0.404f, 0.663f, 0.812f}, // #67a9cf
        {0.129f, 0.400f, 0.675f}  // #2166ac
      };
      constexpr int N = int(sizeof(stops)/sizeof(stops[0]));
      const float pos = t * (N - 1);
      const int   i   = int(std::floor(pos));
      const float a   = pos - i;
      const int   j   = std::min(i + 1, N - 1);
      const float r = (1-a)*stops[i].r + a*stops[j].r;
      const float g = (1-a)*stops[i].g + a*stops[j].g;
      const float b = (1-a)*stops[i].b + a*stops[j].b;
      return qRgba(int(r*255.f), int(g*255.f), int(b*255.f), 216);
    }


    QImage toImage() const {
      if (nx<=0||ny<=0) return QImage();
      QImage img(nx, ny, QImage::Format_ARGB32);
      for (int y=0; y<ny; ++y){
        for (int x=0; x<nx; ++x){
          const int idx = y*nx + x;
          float m = (cnt[idx]>0.f) ? (sum[idx]/cnt[idx]) : 0.f;
          m = std::clamp(m, 0.f, 1.f);
          img.setPixel(x, ny-1-y, ramp(m)); // flip Y
        }
      }
      return img;
    }
  };

private:
  QLabel*     soilTitle_{};            // “Soil Moisture”
  HeatmapWidget* heatmapView_{};       // custom widget w/ hover tooltip
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
