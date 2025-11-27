#include "selfdrive/ui/qt/onroad/vehicle_status.h"

#include <QPaintEvent>
#include <QFontMetrics>
#include <cmath>
#include <stdexcept>

#include "selfdrive/ui/qt/util.h"

VehicleStatusWidget::VehicleStatusWidget(QWidget *parent) : QWidget(parent) {
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    setAttribute(Qt::WA_TranslucentBackground, true);

    // Setup fonts
    header_font = QFont("Inter", 32, QFont::Bold);
    value_font  = QFont("Inter", 28, QFont::DemiBold);
    small_font  = QFont("Inter", 14, QFont::Normal);
    // Full screen height & width
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen) {
        QRect geom = screen->geometry();
        setFixedSize(geom.width()-50, geom.height()-50);
    } else {
        setFixedSize(600, 700); // fallback
    }

    // Allocate all labels
    torque_label = new QLabel("Torque: N/A");
    accel_label = new QLabel("Accel: N/A");
    gas_label = new QLabel("Gas: N/A");
    brake_label = new QLabel("Brake: N/A");
    long_state_label = new QLabel("Long State: N/A");

    speed_label = new QLabel("Speed: N/A");
    steering_angle_label = new QLabel("Steering: N/A");
    steering_torque_label = new QLabel("Steer Torque: N/A");
    yaw_rate_label = new QLabel("Yaw Rate: N/A");
    brake_pressed_label = new QLabel("Brake: N/A");
    gas_pressed_label = new QLabel("Gas: N/A");

    enabled_label = new QLabel("Enabled: N/A");
    active_label = new QLabel("Active: N/A");
    engageable_label = new QLabel("Engageable: N/A");

    panda_connected_label = new QLabel("Connected: N/A");
    panda_ignition_label = new QLabel("Ignition: N/A");
    panda_controls_allowed_label = new QLabel("Controls Allowed: N/A");
    panda_hyundai_long_label = new QLabel("Longitudinal: N/A");
    logging_enabled_label = new QLabel("Logging: N/A");

    setupLayout();
}

void VehicleStatusWidget::setupLayout() {
    main_layout = new QGridLayout(this);
    main_layout->setSpacing(6);
    main_layout->setContentsMargins(10, 10, 10, 10);

    // Create headers
    QLabel *actuator_header = new QLabel("🎮 ACTUATOR");
    QLabel *vehicle_header  = new QLabel("🚗 VEHICLE");
    QLabel *control_header  = new QLabel("🤖 CONTROL");
    QLabel *panda_header    = new QLabel("📡 PANDA");

    QString header_style = "color: cyan; font-weight: bold; font-size: 32px; "
                           "background: rgba(0,50,100,200); padding: 6px; border-radius: 5px;";
    actuator_header->setStyleSheet(header_style);
    vehicle_header->setStyleSheet(header_style);
    control_header->setStyleSheet(header_style);
    panda_header->setStyleSheet(header_style);

    // Column labels
    QList<QLabel*> actuator_labels = { torque_label,accel_label, gas_label, brake_label, long_state_label };

    QList<QLabel*> vehicle_labels = { speed_label, steering_angle_label, steering_torque_label,
                                      yaw_rate_label, brake_pressed_label, gas_pressed_label };

    QList<QLabel*> control_labels = { enabled_label, active_label, engageable_label };
    QList<QLabel*> panda_labels = { panda_connected_label, panda_ignition_label, panda_controls_allowed_label, panda_hyundai_long_label, logging_enabled_label };

    QList<QList<QLabel*>> columns = { actuator_labels, vehicle_labels, control_labels, panda_labels };
    QList<QLabel*> headers = { actuator_header, vehicle_header, control_header, panda_header };

    // Track rows per column
    int rows[4] = {0,0,0,0};

    for (int col = 0; col < 4; ++col) {
        main_layout->addWidget(headers[col], rows[col]++, col);

        for (QLabel* label : columns[col]) {
            label->setFont(value_font);
            label->setStyleSheet(
                "color: white; background: rgba(0,0,0,180); padding: 6px; "
                "border-radius: 6px; margin: 2px; border: 1px solid rgba(255,255,255,50);"
            );
            label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
            label->setMinimumHeight(30);
            main_layout->addWidget(label, rows[col]++, col);
        }

        main_layout->setColumnStretch(col, 1);
    }
}

void VehicleStatusWidget::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    QRect rect = this->rect();

    QLinearGradient gradient(0, 0, 0, rect.height());
    gradient.setColorAt(0, QColor(20, 20, 40, 220));
    gradient.setColorAt(1, QColor(0, 0, 0, 200));

    p.setBrush(QBrush(gradient));
    p.setPen(QPen(QColor(100, 150, 255, 180), 2));
    p.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 12, 12);

    QWidget::paintEvent(event);
}


void VehicleStatusWidget::updateState(const UIState &s) {
  if (!s.scene.started || s.sm == nullptr) return;

  try {
    is_metric = s.scene.is_metric;
    updateActuatorData(s);
    updateVehicleData(s);
    updatePandaData(s);
    update(); // Trigger repaint
  } catch (const std::exception &e) {
    // Silently handle errors to prevent crashes
    return;
  }
}

void VehicleStatusWidget::updateActuatorData(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // Check if carControl message exists and has been received
  try {
    if (sm.alive("carControl") && sm.valid("carControl") && sm.rcv_frame("carControl") > 0) {
      const auto &car_control = sm["carControl"].getCarControl();
      const auto &actuators = car_control.getActuators();

      actuator_torque = actuators.getTorque();
      actuator_accel = actuators.getAccel();
      actuator_gas = actuators.getGas();
      actuator_brake = actuators.getBrake();
      long_control_state = (int)actuators.getLongControlState();

      // Update actuator labels
      torque_label->setText(QString("Torque: %1").arg(actuator_torque, 0, 'f', 2));
      accel_label->setText(QString("Accel: %1 m/s²").arg(actuator_accel, 0, 'f', 2));
      gas_label->setText(QString("Gas: %1").arg(actuator_gas, 0, 'f', 2));
      brake_label->setText(QString("Brake: %1").arg(actuator_brake, 0, 'f', 2));

      QString long_state_text;
      switch (long_control_state) {
        case 0: long_state_text = "OFF"; break;
        case 1: long_state_text = "PID"; break;
        case 2: long_state_text = "STOPPING"; break;
        case 3: long_state_text = "STARTING"; break;
        default: long_state_text = "UNKNOWN"; break;
      }
      long_state_label->setText(QString("Long State: %1").arg(long_state_text));
    }
  } catch (const std::exception &e) {
    // Handle any errors gracefully - keep previous values
  }
}

void VehicleStatusWidget::updateVehicleData(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // Update vehicle state data
  try {
    if (sm.alive("carState") && sm.valid("carState") && sm.rcv_frame("carState") > 0) {
      const auto &car_state = sm["carState"].getCarState();

      vehicle_speed = car_state.getVEgo();
      steering_angle = car_state.getSteeringAngleDeg();
      steering_torque = car_state.getSteeringTorque();
      yaw_rate = car_state.getYawRate();
      brake_pressed = car_state.getBrakePressed();
      gas_pressed = car_state.getGasPressed();

      // Convert speed units
      float display_speed = vehicle_speed;
      QString speed_unit = "m/s";
      if (is_metric) {
        display_speed *= 3.6f; // Convert to km/h
        speed_unit = "km/h";
      } else {
        display_speed *= 2.237f; // Convert to mph
        speed_unit = "mph";
      }

      // Update vehicle labels
      speed_label->setText(QString("Speed: %1 %2").arg(display_speed, 0, 'f', 1).arg(speed_unit));
      steering_angle_label->setText(QString("Steering: %1°").arg(steering_angle, 0, 'f', 1));
      steering_torque_label->setText(QString("Steer Torque: %1").arg(steering_torque, 0, 'f', 1));
      yaw_rate_label->setText(QString("Yaw Rate: %1").arg(yaw_rate, 0, 'f', 2));
      brake_pressed_label->setText(QString("Brake: %1").arg(brake_pressed ? "Pressed" : "Released"));
      gas_pressed_label->setText(QString("Gas: %1").arg(gas_pressed ? "Pressed" : "Released"));


    }
  } catch (const std::exception &e) {
    // Handle any errors gracefully - keep previous values
  }

  // Update control states
  try {
    if (sm.alive("selfdriveState") && sm.valid("selfdriveState") && sm.rcv_frame("selfdriveState") > 0) {
      const auto &selfdrive_state = sm["selfdriveState"].getSelfdriveState();

      controls_enabled = selfdrive_state.getEnabled();
      controls_active = selfdrive_state.getActive();
      controls_engageable = selfdrive_state.getEngageable();

      enabled_label->setText(QString("Enabled: %1").arg(controls_enabled ? "Yes" : "No"));
      active_label->setText(QString("Active: %1").arg(controls_active ? "Yes" : "No"));
      engageable_label->setText(QString("Engageable: %1").arg(controls_engageable ? "Yes" : "No"));

      // Update colors based on state
      QString enabled_style_color = controls_enabled ? "color: lime;" : "color: white;";
      QString active_style_color = controls_active ? "color: lime;" : "color: white;";
      QString engageable_style_color = controls_engageable ? "color: lime;" : "color: orange;";

      enabled_label->setStyleSheet(QString("background: rgba(0,0,0,100); padding: 2px; border-radius: 3px; %1").arg(enabled_style_color));
      active_label->setStyleSheet(QString("background: rgba(0,0,0,100); padding: 2px; border-radius: 3px; %1").arg(active_style_color));
      engageable_label->setStyleSheet(QString("background: rgba(0,0,0,100); padding: 2px; border-radius: 3px; %1").arg(engageable_style_color));
    }
  } catch (const std::exception &e) {
    // Handle any errors gracefully - keep previous values
  }
}

void VehicleStatusWidget::updatePandaData(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // Update panda state data
  try {
    if (sm.alive("pandaStates") && sm.valid("pandaStates") && sm.rcv_frame("pandaStates") > 0) {
      const auto &panda_states = sm["pandaStates"].getPandaStates();

      if (panda_states.size() > 0) {
        const auto &panda_state = panda_states[0]; // Get first panda

        panda_connected = true;
        panda_ignition = panda_state.getIgnitionLine();
        panda_controls_allowed = panda_state.getControlsAllowed();

        // Get longitudinal control status from carParams (works for all car types)
        bool openpilot_longitudinal = false;
        if (sm.alive("carParams") && sm.valid("carParams") && sm.rcv_frame("carParams") > 0) {
          const auto &car_params = sm["carParams"].getCarParams();
          openpilot_longitudinal = car_params.getOpenpilotLongitudinalControl();
        }
        panda_hyundai_longitudinal = openpilot_longitudinal;

        // Update panda labels
        panda_connected_label->setText(QString("Connected: Yes"));
        panda_ignition_label->setText(QString("Ignition: %1").arg(panda_ignition ? "On" : "Off"));
        panda_controls_allowed_label->setText(QString("Controls Allowed: %1").arg(panda_controls_allowed ? "YES" : "NO"));
        panda_hyundai_long_label->setText(QString("Longitudinal: %1").arg(openpilot_longitudinal ? "OPENPILOT" : "STOCK"));

        // Check logging enabled state from testJoystick
        bool logging_enabled = false;
        if (sm.alive("testJoystick") && sm.valid("testJoystick") && sm.rcv_frame("testJoystick") > 0) {
          const auto &test_joystick = sm["testJoystick"].getTestJoystick();
          logging_enabled = test_joystick.getLoggingEnabled();
        }
        logging_enabled_label->setText(QString("Logging: %1").arg(logging_enabled ? "ENABLED" : "DISABLED"));

        // Update colors based on state
        QString connected_color = "color: lime;";
        QString ignition_color = panda_ignition ? "color: lime;" : "color: orange;";
        QString controls_allowed_color = panda_controls_allowed ? "color: lime;" : "color: red;";
        QString longitudinal_color = openpilot_longitudinal ? "color: cyan;" : "color: yellow;";
        QString logging_color = logging_enabled ? "color: lime;" : "color: gray;";

        panda_connected_label->setStyleSheet(QString("background: rgba(0,0,0,180); padding: 3px; border-radius: 4px; margin: 1px; border: 1px solid rgba(255,255,255,50); %1").arg(connected_color));
        panda_ignition_label->setStyleSheet(QString("background: rgba(0,0,0,180); padding: 3px; border-radius: 4px; margin: 1px; border: 1px solid rgba(255,255,255,50); %1").arg(ignition_color));
        panda_controls_allowed_label->setStyleSheet(QString("background: rgba(0,0,0,180); padding: 3px; border-radius: 4px; margin: 1px; border: 1px solid rgba(255,255,255,50); %1").arg(controls_allowed_color));
        panda_hyundai_long_label->setStyleSheet(QString("background: rgba(0,0,0,180); padding: 3px; border-radius: 4px; margin: 1px; border: 1px solid rgba(255,255,255,50); %1").arg(longitudinal_color));
        logging_enabled_label->setStyleSheet(QString("background: rgba(0,0,0,180); padding: 3px; border-radius: 4px; margin: 1px; border: 1px solid rgba(255,255,255,50); %1").arg(logging_color));

      } else {
        panda_connected = false;
        panda_connected_label->setText("Connected: No");
        panda_ignition_label->setText("Ignition: N/A");
        panda_controls_allowed_label->setText("Controls Allowed: N/A");
        panda_hyundai_long_label->setText("Longitudinal: N/A");
        logging_enabled_label->setText("Logging: N/A");
      }
    } else {
      panda_connected = false;
      panda_connected_label->setText("Connected: No");
      panda_ignition_label->setText("Ignition: N/A");
      panda_controls_allowed_label->setText("Controls Allowed: N/A");
      panda_hyundai_long_label->setText("Longitudinal: N/A");
      logging_enabled_label->setText("Logging: N/A");
    }
  } catch (const std::exception &e) {
    // Handle any errors gracefully - keep previous values
  }
}



void VehicleStatusWidget::drawInfoBox(QPainter &p, const QRect &rect, const QString &title,
                                     const QStringList &values, const QColor &bgColor) {
  // Draw background
  p.fillRect(rect, bgColor);
  p.setPen(QPen(text_color, 1));
  p.drawRect(rect);

  // Draw title
  p.setFont(header_font);
  QRect title_rect = rect;
  title_rect.setHeight(30);
  p.drawText(title_rect, Qt::AlignCenter, title);

  // Draw values
  p.setFont(value_font);
  int y_start = rect.top() + 35;
  int line_height = 25;

  for (int i = 0; i < values.size(); ++i) {
    QRect value_rect(rect.left() + 5, y_start + i * line_height, rect.width() - 10, line_height);
    p.drawText(value_rect, Qt::AlignLeft | Qt::AlignVCenter, values[i]);
  }
}