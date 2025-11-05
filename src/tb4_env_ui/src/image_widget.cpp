/*
File: image_widget.cpp
High‑level behavior:
  - Implements ImageWidget by composing a QLabel in a layout and scaling incoming QImages to fit
    while preserving aspect ratio (Qt::KeepAspectRatio). Shows a friendly placeholder if empty.
*/
#include "image_widget.h"
#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent) {
/** Construct an image viewer label with zero margins and center alignment. */
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0,0,0,0);
  label_ = new QLabel(this);
  label_->setAlignment(Qt::AlignCenter);
  label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(label_);
}

void ImageWidget::setImage(
/** Supply a new frame and trigger scaling into the label's current area. */const QImage& img) {
  current_ = img;
  updatePixmap();
}

void ImageWidget::resizeEvent(
/** On resize, recompute the scaled pixmap to preserve aspect ratio. */QResizeEvent* e) {
  QWidget::resizeEvent(e);
  updatePixmap();
}

void ImageWidget::updatePixmap(
/** Convert the current QImage to a QPixmap scaled to fit; show placeholder if empty. */) {
  if (current_.isNull()) {
    label_->setText("No frames yet");
    return;
  }
  const QSize area = label_->size();
  QPixmap pm = QPixmap::fromImage(current_.scaled(area, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  label_->setPixmap(pm);
}
