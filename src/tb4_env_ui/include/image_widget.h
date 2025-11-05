/*
File: image_widget.h
Role & context:
  - Declares ImageWidget, a minimal Qt QWidget that displays a QImage while preserving aspect ratio.
  - Pairs naturally with RosImageBridge::frameReady to preview a camera stream.
Interaction model:
  - `setImage(QImage)` updates the current frame; the widget scales it on resize to fit its label area.
*/
#pragma once
#include <QWidget>
#include <QImage>

class QLabel;

class ImageWidget : public QWidget {

/**
 * @class ImageWidget
 * @brief Thin viewer that scales and centers a QImage.
 * @note Keeps aspect ratio and uses smooth scaling for readability.
 */

  Q_OBJECT
public:
  explicit ImageWidget(QWidget* parent = nullptr);
public slots:
  void setImage(
  /// Supply a new frame to display; triggers a repaint.
const QImage& img);
protected:
  void resizeEvent(
  /// On resize, rescale the image to fit the available label area.
QResizeEvent* e) override;
private:
  void updatePixmap();
  QLabel* label_;
  QImage  current_;
};