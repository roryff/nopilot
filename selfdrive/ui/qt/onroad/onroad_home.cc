#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>
#include <QMouseEvent>

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, this);
    split->insertWidget(0, arCam);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // Create vehicle status widget
  vehicle_status = new VehicleStatusWidget(this);
  vehicle_status->setGeometry(20, 20, vehicle_status->width(), vehicle_status->height());
  stacked_layout->addWidget(vehicle_status);

  // setup stacking order - alerts should be on top of everything
  vehicle_status->raise();
  alerts->raise();  // alerts on top

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  alerts->updateState(s);
  nvg->updateState(s);

  // Update vehicle status widget
  try {
    if (vehicle_status && vehicle_status_visible) {
      vehicle_status->updateState(s);
    }
  } catch (const std::exception &e) {
    // Handle any errors gracefully
  }

  QColor bgColor = bg_colors[s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

void OnroadWindow::mousePressEvent(QMouseEvent *event) {
  // Toggle vehicle status visibility on any mouse press
  toggleVehicleStatus();
  QWidget::mousePressEvent(event);
}

void OnroadWindow::toggleVehicleStatus() {
  vehicle_status_visible = !vehicle_status_visible;
  if (vehicle_status) {
    vehicle_status->setVisible(vehicle_status_visible);
  }
}
