#pragma once

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPainter>
#include <QFont>
#include <QTimer>
#include <QGuiApplication>
#include <QScreen>
#include "selfdrive/ui/ui.h"

class VehicleStatusWidget : public QWidget {
  Q_OBJECT

public:
  explicit VehicleStatusWidget(QWidget *parent = nullptr);
  void updateState(const UIState &s);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  void setupLayout();
  void updateActuatorData(const UIState &s);
  void updateVehicleData(const UIState &s);
  void updatePandaData(const UIState &s);
  void drawInfoBox(QPainter &p, const QRect &rect, const QString &title, const QStringList &values, const QColor &bgColor);

  // Layout components
  QGridLayout *main_layout;

  // Actuator state labels
  QLabel *torque_label;
  QLabel *accel_label;
  QLabel *gas_label;
  QLabel *brake_label;
  QLabel *long_state_label;

  // Vehicle state labels
  QLabel *speed_label;
  QLabel *steering_angle_label;
  QLabel *steering_torque_label;
  QLabel *yaw_rate_label;
  QLabel *brake_pressed_label;
  QLabel *gas_pressed_label;
  QLabel *gear_label;

  // Control state labels
  QLabel *enabled_label;
  QLabel *active_label;
  QLabel *engageable_label;

  // Panda state labels
  QLabel *panda_connected_label;
  QLabel *panda_ignition_label;

  // Data storage
  float actuator_torque = -1.0f;
  float actuator_accel = -1.0f;
  float actuator_gas = -1.0f;
  float actuator_brake = -1.0;
  int long_control_state = -1.0;

  float vehicle_speed = -1.0f;
  float steering_angle =-1.0f;
  float steering_torque = -1.0f;
  float yaw_rate = -1.0f;
  bool brake_pressed = false;
  bool gas_pressed = false;
  QString gear_state = "N";

  bool controls_enabled = false;
  bool controls_active = false;
  bool controls_engageable = false;

  bool panda_connected = false;
  bool panda_ignition = false;

  bool is_metric = false;

  QFont header_font;
  QFont value_font;
  QFont small_font;

  // Colors
  QColor bg_color = QColor(0, 0, 0, 180);
  QColor active_color = QColor(0, 255, 0, 200);
  QColor warning_color = QColor(255, 165, 0, 200);
  QColor error_color = QColor(255, 0, 0, 200);
  QColor inactive_color = QColor(128, 128, 128, 200);
  QColor text_color = QColor(255, 255, 255);
};