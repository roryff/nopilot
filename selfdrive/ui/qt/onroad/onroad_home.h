#pragma once

#include "selfdrive/ui/qt/onroad/alerts.h"
#include "selfdrive/ui/qt/onroad/annotated_camera.h"
#include "selfdrive/ui/qt/onroad/vehicle_status.h"

class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);

protected:
  void mousePressEvent(QMouseEvent *event) override;

private:
  void paintEvent(QPaintEvent *event);
  void toggleVehicleStatus();

  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  VehicleStatusWidget *vehicle_status;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QHBoxLayout* split;

  bool vehicle_status_visible = true;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
