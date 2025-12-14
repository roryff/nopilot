#!/usr/bin/env python3

import math
import os
import numpy as np

from cereal import messaging, car, log
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL, Ratekeeper
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

LongCtrlState = car.CarControl.Actuators.LongControlState

MAX_LAT_ACCEL = 3.0
print_loop=1000

def joystickd_thread():
  params = Params()
  print("joystickd: Starting up...")

  try:
    params_source = "CarParamsPersistent" if os.environ.get("SKIP_FW_QUERY") else "CarParams"
    print("joystickd: Waiting for Carparams")
    CP = messaging.log_from_bytes(params.get(params_source, block=True), car.CarParams)
    print(f"joystickd: Got CarParams for {CP.carFingerprint}")
    VM = VehicleModel(CP)
    print("joystickd: VehicleModel initialized")

    sm = messaging.SubMaster(['carState', 'liveParameters', 'testJoystick'], frequency=1. / DT_CTRL)
    pm = messaging.PubMaster(['carControl', 'controlsState', 'selfdriveState', 'onroadEvents'])

    rk = Ratekeeper(100, print_delay_threshold=None)

    loop_count = 0
    CS_prev = None

    last_joystick_update = 0
    JOYSTICK_TIMEOUT = 5

    system_enabled = False
    user_disabled = True
    joystick_active = False
    graceful_stop_debounce = -1

    while True:
      try:
        sm.update(0)
        loop_count += 1

        # Publish empty onroadEvents so card.py can initialize

        # ---------------------------------------------------------------------
        # Periodic logging & onroadEvents publishing
        # ---------------------------------------------------------------------
        if loop_count % 100 == 1:  # Publish at 1Hz
          events_msg = messaging.new_message('onroadEvents', 0)
          events_msg.valid = True
          pm.send('onroadEvents', events_msg)

        if loop_count % print_loop == 0:  # Print every 5 seconds at 100Hz
          print(f"joystickd: Loop {loop_count}, alive: carState={sm.alive['carState']}, testJoystick={sm.alive['testJoystick']}")
          print(f"joystickd: Valid: carState={sm.valid['carState']}, testJoystick={sm.valid['testJoystick']}")
          print(f"joystickd: Updated: carState={sm.updated['carState']}, testJoystick={sm.updated['testJoystick']}")
          print(f"joystickd: Joystick: last_update={last_joystick_update}, loops_ago={loop_count - last_joystick_update}, active={joystick_active}")

        # ---------------------------------------------------------------------
        # CarControl message
        # ---------------------------------------------------------------------
        cc_msg = messaging.new_message('carControl')
        cc_msg.valid = True
        CC = cc_msg.carControl

        # ---------------------------------------------------------------------
        # Joystick activity detection
        # ---------------------------------------------------------------------
        if sm.updated["testJoystick"] and sm.valid["testJoystick"]:
          last_joystick_update = loop_count
        # Joystick is active if we've seen data within the last 5 loops (50ms at 100Hz)
        prev_joy = joystick_active
        joystick_active = (loop_count - last_joystick_update) <= JOYSTICK_TIMEOUT

        # Joystick lost → start countdown
        if prev_joy and not joystick_active:
          # Joystick just lost - start 2 second countdown (200 loops at 100Hz)
          graceful_stop_debounce = 200
          print("joystickd: JOYSTICK LOST - Starting 2 second graceful stop countdown")

        #joystick reconnected → clear countdown
        elif joystick_active and not prev_joy:
          # Joystick reconnected - clear countdown and allow normal operation
          graceful_stop_debounce = -1
          print("joystickd: JOYSTICK RECONNECTED - Resuming normal operation")
        #countdown active → decrement
        elif not joystick_active and graceful_stop_debounce > 0:
          graceful_stop_debounce -= 1


        # ---------------------------------------------------------------------
        # SelfdriveState early handling if no car data
        # ---------------------------------------------------------------------
        if not sm.alive["carState"] or not sm.valid["carState"]:
          ss = messaging.new_message('selfdriveState')
          ss.valid = True
          sd = ss.selfdriveState

          sd.state = log.SelfdriveState.OpenpilotState.disabled
          sd.alertText1 = "No Car Data"
          sd.alertText2 = "Waiting for car connection"
          sd.alertStatus = log.SelfdriveState.AlertStatus.normal
          sd.alertSize = log.SelfdriveState.AlertSize.small
          sd.enabled = False
          sd.active = False
          sd.engageable = False
          sd.experimentalMode = False

          pm.send('selfdriveState', ss)
          rk.keep_time()
          continue


        CS = sm['carState']


        # ---------------------------------------------------------------------
        # User override detection (brake, gas, steering)
        # ---------------------------------------------------------------------
        if CS_prev is not None:
          brake_override = CS.brakePressed and (not CS_prev.brakePressed or not CS.standstill)
          gas_override = CS.gasPressed and not CS_prev.gasPressed
          steer_override = CS.steeringPressed  # Not used for disable for now, but still tracked

          # Disable system on any override
          if brake_override or gas_override:
            user_disabled = True
            system_enabled = False
            if loop_count % print_loop == 0:  # Print once per second
              override_type = "BRAKE" if brake_override else "GAS"
              print(f"joystickd: {override_type} OVERRIDE - System disabled! Press cruise button to re-enable.")


        # Check for cruise control button to re-enable
        if user_disabled and len(CS.buttonEvents) > 0:
          print(f"joystickd: User disabled, checking buttons for re-enable...")
          for button in CS.buttonEvents:
            print(f"  - Checking button type: {button.type}, pressed: {button.pressed}")
            if button.type in [car.CarState.ButtonEvent.Type.setCruise,
                             car.CarState.ButtonEvent.Type.resumeCruise,
                             car.CarState.ButtonEvent.Type.mainCruise,
                             car.CarState.ButtonEvent.Type.accelCruise] and button.pressed:
              if not joystick_active:
                print(f"joystickd: CRUISE BUTTON pressed but joystick inactive - cannot re-enable.")
                break
              user_disabled = False
              system_enabled = True
              print(f"joystickd: CRUISE BUTTON pressed - System re-enabled!")
              break

        # If never disabled by user, allow joystick to enable
        # if not user_disabled:
        #   system_enabled = joystick_active

        # ---------------------------------------------------------------------
        # Control mode flags
        # ---------------------------------------------------------------------
        CC.enabled = system_enabled and not CS.steerFaultPermanent 
        CC.latActive = CC.enabled and not CS.steerFaultTemporary
        CC.longActive = CC.enabled and CP.openpilotLongitudinalControl

        CC.cruiseControl.cancel = CS.cruiseState.enabled and not CC.enabled
        CC.cruiseControl.override = False  # Not using stock SCC override
        CC.cruiseControl.resume = False    # Not using stock cruise resume

        CC.hudControl.leadDistanceBars = 2
        CC.hudControl.setSpeed = 55 * (1.609 if not CS.cruiseState.available else 1)
        CC.hudControl.leadVisible = False


        if loop_count % print_loop == 0 and joystick_active:
          print(f"joystickd: enabled={CC.enabled}, latActive={CC.latActive}, longActive={CC.longActive}")
          print(f"joystickd: CP.openpilotLongitudinalControl={CP.openpilotLongitudinalControl}")

        # ---------------------------------------------------------------------
        # Joystick input
        # ---------------------------------------------------------------------

        # Get joystick input or default to neutral
        if joystick_active and sm.updated['testJoystick']:
          joystick_axes = sm['testJoystick'].axes
          if loop_count % print_loop == 0:
            print(f"joystickd: Got joystick axes: {joystick_axes}")
        else:
          joystick_axes = [0.0, 0.0]


        actuators = CC.actuators
        # ---------------------------------------------------------------------
        # Longitudinal control
        # ---------------------------------------------------------------------
        # Graceful stop override - takes priority over normal long control
        if system_enabled and not joystick_active and graceful_stop_debounce >= 0:
          if graceful_stop_debounce > 0:
            actuators.accel = -1.0
            if loop_count % 10 == 0:  # Print every 0.1s during countdown
              print(f"joystickd: GRACEFUL STOP - accel: -1.0, countdown: {graceful_stop_debounce / 100:.1f}s")
          elif graceful_stop_debounce == 0:
            actuators.accel = -3.0
            print(f"joystickd: HARD STOP - accel: -3.0")

          actuators.longControlState = (
            LongCtrlState.pid if sm['carState'].vEgo > 0.1 else LongCtrlState.stopping
          )
        elif CC.longActive:
          actuators.accel = 4.0 * float(np.clip(joystick_axes[0], -1, 1))

          if sm['carState'].vEgo > 0.1:
            actuators.longControlState = LongCtrlState.pid
          elif actuators.accel > 0.1:  # User wants to accelerate
            actuators.longControlState = LongCtrlState.pid
          else:
            actuators.longControlState = LongCtrlState.stopping

          if loop_count % print_loop == 0:
            print(f"joystickd: Long control - accel: {actuators.accel:.3f}, state: {actuators.longControlState}, vEgo: {sm['carState'].vEgo:.2f}")
        else:
          actuators.accel = 0.0
          actuators.longControlState = (
            LongCtrlState.pid if sm['carState'].vEgo > 0.1 else LongCtrlState.stopping
          )

          if loop_count % print_loop == 0 and joystick_active:
            print(f"joystickd: Long control DISABLED - CP.openpilotLongitudinalControl={CP.openpilotLongitudinalControl}, enabled={CC.enabled}")


        # ---------------------------------------------------------------------
        # Lateral control
        # ---------------------------------------------------------------------
        if CC.latActive:
          try:
            actuators.torque = float(np.clip(joystick_axes[1], -1, 1))
            if loop_count % print_loop == 0:
              direction = "LEFT" if actuators.torque < -0.2 else ("RIGHT" if actuators.torque > 0.2 else "CENTER")
              print(f"joystickd: Lat control - torque: {actuators.torque:.3f} ({direction}), angle: {actuators.steeringAngleDeg:.1f}")
          except Exception as e:
            print(f"joystickd: ERROR in lateral control: {e}")

        pm.send('carControl', cc_msg)


        # ---------------------------------------------------------------------
        # ControlsState message
        # ---------------------------------------------------------------------
        cs_msg = messaging.new_message('controlsState')
        cs_msg.valid = True
        controlsState = cs_msg.controlsState
        controlsState.lateralControlState.init('debugState')
        try:
          # Simple curvature calculation without live parameter dependencies
          if sm.alive['liveParameters'] and sm.valid['liveParameters']:
            lp = sm['liveParameters']
            steer_angle_without_offset = math.radians(sm['carState'].steeringAngleDeg - lp.angleOffsetDeg)
            controlsState.curvature = -VM.calc_curvature(steer_angle_without_offset, sm['carState'].vEgo, lp.roll)
          else:
            # Fallback: use raw steering angle if liveParameters not available
            steer_angle = math.radians(sm['carState'].steeringAngleDeg)
            controlsState.curvature = -VM.calc_curvature(steer_angle, sm['carState'].vEgo, 0.0)
        except Exception as e:
          print(f"joystickd: ERROR in controlsState: {e}")
          controlsState.curvature = 0.0
        pm.send('controlsState', cs_msg)

        # ---------------------------------------------------------------------
        # SelfdriveState (final status reporting)
        # ---------------------------------------------------------------------
        ss_msg = messaging.new_message('selfdriveState')
        ss_msg.valid = True
        selfdriveState = ss_msg.selfdriveState

        # Set proper state based on our control logic
        if False:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "ACC FAULT"
          selfdriveState.alertText2 = "TAKE CONTROL"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.critical
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.full
        elif CS.steerFaultPermanent:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "STEER FAULT"
          selfdriveState.alertText2 = "TAKE CONTROL"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.critical
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.full

        elif system_enabled and not joystick_active and graceful_stop_debounce >= 0:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "JOYSTICK LOST - TAKE CONTROL"
          selfdriveState.alertText2 = f"Stopping in {graceful_stop_debounce / 100:.1f}s"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.critical
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.full
        elif not joystick_active:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "No Joystick"
          selfdriveState.alertText2 = "Connect joystick input"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.normal
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.small
        elif user_disabled:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "System Disabled"
          selfdriveState.alertText2 = "Press cruise button to re-enable"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.userPrompt
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.mid

        selfdriveState.enabled = CC.enabled
        selfdriveState.active = CC.latActive or CC.longActive
        selfdriveState.engageable = CC.enabled and not CS.steerFaultPermanent
        selfdriveState.experimentalMode = False

        pm.send('selfdriveState', ss_msg)

        # Update previous state for override detection
        CS_prev = CS

        rk.keep_time()

      except Exception as e:
        print(f"joystickd: ERROR in main loop: {e}")
        import traceback
        traceback.print_exc()

  except Exception as e:
    print(f"joystickd: FATAL ERROR during initialization: {e}")
    import traceback
    traceback.print_exc()


def main():
  joystickd_thread()


if __name__ == "__main__":
  main()
