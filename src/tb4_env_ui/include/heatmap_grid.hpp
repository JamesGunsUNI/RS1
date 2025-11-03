#pragma once
#include <vector>
#include <QImage>
#include <algorithm>
#include <cmath>

struct HeatmapGrid {
  double min_x=-10, max_x=10, min_y=-10, max_y=10, res=0.5;
  int nx=0, ny=0;
  std::vector<float> sum, cnt;

  HeatmapGrid(double minx, double maxx, double miny, double maxy, double resolution)
  : min_x(minx), max_x(maxx), min_y(miny), max_y(maxy), res(resolution) {
    nx = int(std::ceil((max_x - min_x)/res));
    ny = int(std::ceil((max_y - min_y)/res));
    sum.assign(nx*ny, 0.f);
    cnt.assign(nx*ny, 0.f);
  }

  bool index(double x, double y, int& i) const {
    int ix = int(std::floor((x - min_x)/res));
    int iy = int(std::floor((y - min_y)/res));
    if (ix<0||iy<0||ix>=nx||iy>=ny) return false;
    i = iy*nx + ix; return true;
  }

  void add(double x, double y, float v){
    int i; if(!index(x,y,i)) return;
    sum[i] += v; cnt[i] += 1.f;
  }

  QImage toImage() const {
    QImage img(nx, ny, QImage::Format_RGBA8888);
    for(int y=0;y<ny;++y){
      for(int x=0;x<nx;++x){
        int i = y*nx + x;
        float m = (cnt[i]>0.f) ? (sum[i]/cnt[i]) : 0.f;
        float c = std::clamp(m, 0.f, 1.f);
        int r,g,b,a=216;
        if (c < 0.3f){ float t=c/0.3f; r=int((0.8f+0.2f*t)*255); g=int((0.0f+0.4f*t)*255); b=0; }
        else if (c < 0.6f){ float t=(c-0.3f)/0.3f; r=int((1.0f-t)*255); g=int((0.4f+0.6f*t)*255); b=0; }
        else { float t=(c-0.6f)/0.4f; r=0; g=int((1.0f-0.3f*t)*255); b=int((0.7f*t)*255); }
        img.setPixel(x, ny-1-y, qRgba(r,g,b,a));
      }
    }
    return img;
  }
};
