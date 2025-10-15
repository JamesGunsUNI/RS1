#pragma once
#include <QWidget>
#include <QImage>

class QLabel;

class ImageWidget : public QWidget {
  Q_OBJECT
public:
  explicit ImageWidget(QWidget* parent = nullptr);
public slots:
  void setImage(const QImage& img);
protected:
  void resizeEvent(QResizeEvent* e) override;
private:
  void updatePixmap();
  QLabel* label_;
  QImage  current_;
};
