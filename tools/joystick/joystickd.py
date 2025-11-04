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
    if os.environ.get("SKIP_FW_QUERY"):
        params_source="CarParamsPersistent"
    else:
        params_source="CarParams"
    print("joystickd: Waiting for Carparams")
    CP = messaging.log_from_bytes(params.get(params_source, block=True), car.CarParams)
    print(f"joystickd: Got CarParams for {CP.carFingerprint}")
    VM = VehicleModel(CP)
    print("joystickd: VehicleModel initialized")

    sm = messaging.SubMaster(['carState', 'liveParameters', 'testJoystick'], frequency=1. / DT_CTRL)
    pm = messaging.PubMaster(['carControl', 'controlsState', 'selfdriveState'])

    rk = Ratekeeper(100, print_delay_threshold=None)

    loop_count = 0
    CS_prev = None
    last_joystick_update = 0  # Track when we last saw joystick data
    JOYSTICK_TIMEOUT = 5  # Consider joystick dead after 5 loops without update

    # Control state variables
    system_enabled = False  # Overall system enable state
    user_disabled = False   # Set True when user overrides (brake/gas/steering)

    while True:
      try:
        sm.update(0)
        loop_count += 1

        if loop_count % print_loop == 0:  # Print every 5 seconds at 100Hz
          print(f"joystickd: Loop {loop_count}, alive: carState={sm.alive['carState']}, testJoystick={sm.alive['testJoystick']}")
          print(f"joystickd: Valid: carState={sm.valid['carState']}, testJoystick={sm.valid['testJoystick']}")
          print(f"joystickd: Updated: carState={sm.updated['carState']}, testJoystick={sm.updated['testJoystick']}")
          print(f"joystickd: Joystick: last_update={last_joystick_update}, loops_ago={loop_count - last_joystick_update}, active={joystick_active}")
          print(f"joystickd: State: system_enabled={system_enabled}, user_disabled={user_disabled}, CC.enabled={CC.enabled}")

        cc_msg = messaging.new_message('carControl')
        cc_msg.valid = True
        CC = cc_msg.carControl

        # Track joystick updates - if we see fresh data, update timestamp
        if sm.updated["testJoystick"] and sm.valid["testJoystick"]:
          last_joystick_update = loop_count

        # Joystick is active if we've seen data within the last 5 loops (50ms at 100Hz)
        joystick_active = (loop_count - last_joystick_update) <= JOYSTICK_TIMEOUT

        # Always publish selfdriveState so UI knows we're alive
        # Publish selfdriveState for UI visibility
        ss_msg = messaging.new_message('selfdriveState')
        ss_msg.valid = True
        selfdriveState = ss_msg.selfdriveState

        # Set proper state based on our control logic
        if not sm.alive["carState"] or not sm.valid["carState"]:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "No Car Data"
          selfdriveState.alertText2 = "Waiting for car connection"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.normal
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.small
          selfdriveState.enabled = False
          selfdriveState.active = False
          selfdriveState.engageable = False
          selfdriveState.experimentalMode = False
          pm.send('selfdriveState', ss_msg)
          rk.keep_time()
          continue

        CS = sm['carState']

        # Check for user overrides to disable system
        if CS_prev is not None:
          # Brake override: disable on brake press (or if braking while moving)
          brake_override = CS.brakePressed and (not CS_prev.brakePressed or not CS.standstill)

          # Gas override: disable on gas press
          gas_override = CS.gasPressed and not CS_prev.gasPressed

          # Steering override: soft disable on steering input
          steer_override = CS.steeringPressed

          # Disable system on any override
          if brake_override or gas_override:
            user_disabled = True
            system_enabled = False
            if loop_count % print_loop == 0:  # Print once per second
              override_type = "BRAKE" if brake_override else "GAS"
              print(f"joystickd: {override_type} OVERRIDE - System disabled! Press cruise button to re-enable.")

        # Check for cruise control button to re-enable
        if user_disabled and len(CS.buttonEvents) > 0:
          for button in CS.buttonEvents:
            if button.type in [log.CarState.ButtonEvent.Type.setCruise,
                             log.CarState.ButtonEvent.Type.resumeCruise,
                             log.CarState.ButtonEvent.Type.mainCruise] and button.pressed:
              user_disabled = False
              system_enabled = True
              print(f"joystickd: CRUISE BUTTON pressed - System re-enabled!")
              break

        # If never disabled by user, allow joystick to enable
        if not user_disabled:
          system_enabled = joystick_active

        # Set control states
        CC.enabled = system_enabled and not CS.steerFaultPermanent
        CC.latActive = CC.enabled and not CS.steerFaultTemporary and not CS.steeringPressed
        CC.longActive = CC.enabled and CP.openpilotLongitudinalControl and False # Disable longitudinal control for safety for now
        CC.cruiseControl.cancel = False  # Don't interfere with cruise control
        CC.hudControl.leadDistanceBars = 2

        if loop_count % print_loop == 0 and joystick_active:
          print(f"joystickd: enabled={CC.enabled}, latActive={CC.latActive}, longActive={CC.longActive}")

        actuators = CC.actuators

        # Get joystick input or default to neutral
        if joystick_active and sm.updated['testJoystick']:
          joystick_axes = sm['testJoystick'].axes
          if loop_count % print_loop == 0:
            print(f"joystickd: Got joystick axes: {joystick_axes}")
        else:
          joystick_axes = [0.0, 0.0]

        if CC.longActive:
          # Simple acceleration control: joystick forward/back controls acceleration
          actuators.accel = 4.0 * float(np.clip(joystick_axes[0], -1, 1))
          # Use PID control when moving, stopping control when stationary
          actuators.longControlState = LongCtrlState.pid if sm['carState'].vEgo > 0.1 else LongCtrlState.stopping

          if loop_count % print_loop == 0:
            print(f"joystickd: Long control - accel: {actuators.accel:.3f}, state: {actuators.longControlState}")

        if CC.latActive:
          try:
            actuators.torque = float(np.clip(joystick_axes[1], -1, 1))
            if actuators.torque < -0.2:
              CC.leftBlinker = True
              CC.rightBlinker = False
            elif actuators.torque > 0.2:
              CC.rightBlinker = True
              CC.leftBlinker = False
            else:
              CC.leftBlinker = False
              CC.rightBlinker = False

            if loop_count % print_loop == 0:
              print(f"joystickd: Lat control - torque: {actuators.torque:.3f}, angle: {actuators.steeringAngleDeg:.1f}")
          except Exception as e:
            print(f"joystickd: ERROR in lateral control: {e}")

        pm.send('carControl', cc_msg)

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

        # Update selfdriveState with car-dependent logic (carState is available here)
        ss_msg = messaging.new_message('selfdriveState')
        ss_msg.valid = True
        selfdriveState = ss_msg.selfdriveState

        # Set proper state based on our control logic
        if user_disabled:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "System Disabled"
          selfdriveState.alertText2 = "Press cruise button to re-enable"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.userPrompt
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.mid
        elif not joystick_active:
          selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
          selfdriveState.alertText1 = "No Joystick"
          selfdriveState.alertText2 = "Connect joystick input"
          selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.normal
          selfdriveState.alertSize = log.SelfdriveState.AlertSize.small

        selfdriveState.enabled = CC.enabled
        selfdriveState.active = CC.latActive or CC.longActive
        selfdriveState.engageable = joystick_active and not CS.steerFaultPermanent
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