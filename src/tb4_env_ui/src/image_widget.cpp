#include "image_widget.h"
#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0,0,0,0);
  label_ = new QLabel(this);
  label_->setAlignment(Qt::AlignCenter);
  label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(label_);
}

void ImageWidget::setImage(const QImage& img) {
  current_ = img;
  updatePixmap();
}

void ImageWidget::resizeEvent(QResizeEvent* e) {
  QWidget::resizeEvent(e);
  updatePixmap();
}

void ImageWidget::updatePixmap() {
  if (current_.isNull()) {
    label_->setText("No frames yet");
    return;
  }
  const QSize area = label_->size();
  QPixmap pm = QPixmap::fromImage(current_.scaled(area, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  label_->setPixmap(pm);
}
