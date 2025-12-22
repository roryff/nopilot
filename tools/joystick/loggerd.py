#!/usr/bin/env python3
"""
Comprehensive Data Logger for Joystick Control
Logs all car data, inputs, actuat            # Actuator Commands (What we send)
            'actuators_accel',
            'actuators_torque',
            'actuators_steeringAngleDeg',
            'actuators_curvature',
            'actuators_speed',
            'actuators_longControlState',

            # Car Output (What actually gets sent to car after safety restrictions)
            'carOutput_valid',
            'carOutput_accel',
            'carOutput_torque',
            'carOutput_steeringAngleDeg',
            'carOutput_curvature',
            'carOutput_speed',
            'carOutput_longControlState',

            # Car Control Flagsds, and control states to CSV
Enable/disable via testJoystick.loggingEnabled field
"""

import csv
import time
import os
from pathlib import Path
from datetime import datetime

from cereal import messaging, car, log
from openpilot.common.realtime import DT_CTRL, Ratekeeper
from openpilot.common.params import Params


class ComprehensiveLogger:
    def __init__(self):
        self.csv_writer = None
        self.csv_file = None
        self.logging_enabled = False
        self.log_dir = Path("/data/joystick_logs")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.current_log_path = None
        self.row_count = 0

    def get_csv_headers(self):
        """Define all CSV column headers - COMPREHENSIVE list"""
        return [
            # Timestamp
            'timestamp',
            'logMonoTime',
            'loop_count',

            # System State
            'system_enabled',
            'controls_allowed',
            'lat_active',
            'long_active',
            'joystick_active',

            # Joystick Inputs (Raw)
            'joy_axis_0_gb',
            'joy_axis_1_steer',
            'joy_button_count',
            'joy_logging_enabled',

            # Car State - Motion
            'vEgo',
            'vEgoRaw',
            'aEgo',
            'yawRate',
            'standstill',
            'wheelSpeeds_fl',
            'wheelSpeeds_fr',
            'wheelSpeeds_rl',
            'wheelSpeeds_rr',

            # Car State - Steering
            'steeringAngleDeg',
            'steeringRateDeg',
            'steeringTorque',
            'steeringTorqueEps',
            'steeringPressed',
            'steerFaultTemporary',
            'steerFaultPermanent',
            'steerWarning',

            # Car State - Blind spots
            'leftBlindspot',
            'rightBlindspot',

            # Car State - Pedals
            'gas',
            'gasPressed',
            'brake',
            'brakePressed',
            'brakeHoldActive',
            'parkingBrake',

            # Car State - Gear & Cruise
            'gearShifter',
            'cruiseState_enabled',
            'cruiseState_available',
            'cruiseState_speed',
            'cruiseState_standstill',

            # Car State - Buttons
            'leftBlinker',
            'rightBlinker',
            'genericToggle',
            'doorOpen',
            'seatbeltUnlatched',
            'espDisabled',

            # Car State - Faults
            'stockAeb',
            'stockFcw',
            'espActive',
            'accFaulted',

            # Actuator Commands (What we're sending)
            'actuators_accel',
            'actuators_torque',
            'actuators_steeringAngleDeg',
            'actuators_curvature',
            'actuators_speed',
            'actuators_longControlState',

            # Car Output (Actual output sent to car after safety restrictions)
            'carOutput_valid',
            'carOutput_accel',
            'carOutput_torque',
            'carOutput_steeringAngleDeg',
            'carOutput_curvature',
            'carOutput_speed',
            'carOutput_longControlState',

            # Car Control Flags
            'enabled',
            'latActive',
            'longActive',
            'leftBlinker_cmd',
            'rightBlinker_cmd',
            # Controls State
            'controlsState_curvature',
            'controlsState_lateralControlState',

        ]

    def start_logging(self):
        """Start a new CSV log file"""
        if self.logging_enabled:
            print("loggerd: Already logging!")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_log_path = self.log_dir / f"joystick_log_{timestamp}.csv"

        try:
            self.csv_file = open(self.current_log_path, 'w', newline='')
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.get_csv_headers())
            self.csv_writer.writeheader()
            self.csv_file.flush()

            self.logging_enabled = True
            self.row_count = 0
            print(f"loggerd: ✓ Started logging to {self.current_log_path}")

        except Exception as e:
            print(f"loggerd: ERROR starting log file: {e}")
            self.logging_enabled = False

    def stop_logging(self):
        """Stop logging and close file"""
        if not self.logging_enabled:
            return

        try:
            if self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None

            print(f"loggerd: ✓ Stopped logging. Wrote {self.row_count} rows to {self.current_log_path}")
            self.logging_enabled = False
            self.row_count = 0

        except Exception as e:
            print(f"loggerd: ERROR stopping log: {e}")

    def log_frame(self, sm, CC, controlsState, selfdriveState, system_state):
        """Log a single frame of data"""
        if not self.logging_enabled or not self.csv_writer:
            return

        try:
            CS = sm['carState']
            joy = sm['testJoystick']
            lp = sm['liveParameters'] if sm.valid.get('liveParameters', False) else None
            carOutput = sm['carOutput'] if sm.valid.get('carOutput', False) else None

            # Use empty string for missing numeric data so we can distinguish from actual 0
            MISSING = ''  # Empty string in CSV indicates missing data

            # Build comprehensive data row
            row = {
                # Timestamp
                'timestamp': time.time(),
                'logMonoTime': sm.logMonoTime.get('carState', 0),
                'loop_count': system_state.get('loop_count', 0),

                # System State
                'system_enabled': system_state.get('system_enabled', False),
                'controls_allowed': system_state.get('controls_allowed', False),
                'lat_active': CC.latActive,
                'long_active': CC.longActive,
                'joystick_active': system_state.get('joystick_active', False),

                # Joystick Inputs
                'joy_axis_0_gb': joy.axes[0] if len(joy.axes) > 0 else MISSING,
                'joy_axis_1_steer': joy.axes[1] if len(joy.axes) > 1 else MISSING,
                'joy_button_count': len(joy.buttons) if hasattr(joy, 'buttons') else MISSING,
                'joy_logging_enabled': getattr(joy, 'loggingEnabled', False),

                # Car State - Motion
                'vEgo': getattr(CS, 'vEgo', MISSING),
                'vEgoRaw': getattr(CS, 'vEgoRaw', MISSING),
                'aEgo': getattr(CS, 'aEgo', MISSING),
                'yawRate': getattr(CS, 'yawRate', MISSING),
                'standstill': getattr(CS, 'standstill', False),
                'wheelSpeeds_fl': getattr(CS.wheelSpeeds, 'fl', MISSING) if hasattr(CS, 'wheelSpeeds') else MISSING,
                'wheelSpeeds_fr': getattr(CS.wheelSpeeds, 'fr', MISSING) if hasattr(CS, 'wheelSpeeds') else MISSING,
                'wheelSpeeds_rl': getattr(CS.wheelSpeeds, 'rl', MISSING) if hasattr(CS, 'wheelSpeeds') else MISSING,
                'wheelSpeeds_rr': getattr(CS.wheelSpeeds, 'rr', MISSING) if hasattr(CS, 'wheelSpeeds') else MISSING,

                # Car State - Steering
                'steeringAngleDeg': getattr(CS, 'steeringAngleDeg', MISSING),
                'steeringRateDeg': getattr(CS, 'steeringRateDeg', MISSING),
                'steeringTorque': getattr(CS, 'steeringTorque', MISSING),
                'steeringTorqueEps': getattr(CS, 'steeringTorqueEps', MISSING),
                'steeringPressed': getattr(CS, 'steeringPressed', MISSING),
                'steerFaultTemporary': getattr(CS, 'steerFaultTemporary', MISSING),
                'steerFaultPermanent': getattr(CS, 'steerFaultPermanent', MISSING),
                'steerWarning': getattr(CS, 'steerWarning', MISSING),


                # Car State - Blind spots
                'leftBlindspot': getattr(CS, 'leftBlindspot', MISSING),
                'rightBlindspot': getattr(CS, 'rightBlindspot', MISSING),

                # Car State - Pedals
                'gas': getattr(CS, 'gas', MISSING),  # Field doesn't exist in CarState
                'gasPressed': getattr(CS, 'gasPressed', MISSING),
                'brake': getattr(CS, 'brake', MISSING),
                'brakePressed': getattr(CS, 'brakePressed', MISSING),
                'brakeHoldActive': getattr(CS, 'brakeHoldActive', MISSING),
                'parkingBrake': getattr(CS, 'parkingBrake', MISSING),

                # Car State - Gear & Cruise
                'gearShifter': str(getattr(CS, 'gearShifter', 'unknown')),
                'cruiseState_enabled': getattr(CS.cruiseState, 'enabled', MISSING) if hasattr(CS, 'cruiseState') else MISSING,
                'cruiseState_available': getattr(CS.cruiseState, 'available', MISSING) if hasattr(CS, 'cruiseState') else MISSING,
                'cruiseState_speed': getattr(CS.cruiseState, 'speed', MISSING) if hasattr(CS, 'cruiseState') else MISSING,
                'cruiseState_standstill': getattr(CS.cruiseState, 'standstill', MISSING) if hasattr(CS, 'cruiseState') else MISSING,

                # Car State - Buttons
                'leftBlinker': getattr(CS, 'leftBlinker', MISSING),
                'rightBlinker': getattr(CS, 'rightBlinker', MISSING),
                'genericToggle': getattr(CS, 'genericToggle', MISSING),
                'doorOpen': getattr(CS, 'doorOpen', MISSING),
                'seatbeltUnlatched': getattr(CS, 'seatbeltUnlatched', MISSING),
                'espDisabled': getattr(CS, 'espDisabled', MISSING),

                # Car State - Faults
                'stockAeb': getattr(CS, 'stockAeb', False),
                'stockFcw': getattr(CS, 'stockFcw', False),
                'espActive': getattr(CS, 'espActive', False),
                'accFaulted': getattr(CS, 'accFaulted', False),


                # Actuator Commands
                'actuators_accel': getattr(CC.actuators, 'accel', MISSING) if hasattr(CC, 'actuators') else MISSING,
                'actuators_torque': getattr(CC.actuators, 'torque', MISSING) if hasattr(CC, 'actuators') else MISSING,
                'actuators_steeringAngleDeg': getattr(CC.actuators, 'steeringAngleDeg', MISSING) if hasattr(CC, 'actuators') else MISSING,
                'actuators_curvature': getattr(CC.actuators, 'curvature', MISSING) if hasattr(CC, 'actuators') else MISSING,
                'actuators_speed': getattr(CC.actuators, 'speed', MISSING) if hasattr(CC, 'actuators') else MISSING,
                'actuators_longControlState': str(getattr(CC.actuators, 'longControlState', 'off')) if hasattr(CC, 'actuators') else 'off',

                # Car Output (Actual output sent to car after safety restrictions)
                'carOutput_valid': sm.valid.get('carOutput', False),
                'carOutput_accel': getattr(carOutput.actuatorsOutput, 'accel', MISSING) if carOutput and hasattr(carOutput, 'actuatorsOutput') else MISSING,
                'carOutput_torque': getattr(carOutput.actuatorsOutput, 'torque', MISSING) if carOutput and hasattr(carOutput, 'actuatorsOutput') else MISSING,
                'carOutput_steeringAngleDeg': getattr(carOutput.actuatorsOutput, 'steeringAngleDeg', MISSING) if carOutput and hasattr(carOutput, 'actuatorsOutput') else MISSING,
                'carOutput_curvature': getattr(carOutput.actuatorsOutput, 'curvature', MISSING) if carOutput and hasattr(carOutput, 'actuatorsOutput') else MISSING,
                'carOutput_speed': getattr(carOutput.actuatorsOutput, 'speed', MISSING) if carOutput and hasattr(carOutput, 'actuatorsOutput') else MISSING,
                'carOutput_longControlState': str(getattr(carOutput.actuatorsOutput, 'longControlState', 'off')) if carOutput and hasattr(carOutput, 'actuatorsOutput') else 'none',

                # Car Control Flags
                'enabled': getattr(CC, 'enabled', MISSING),
                'latActive': getattr(CC, 'latActive', MISSING),
                'longActive': getattr(CC, 'longActive', MISSING),
                'leftBlinker_cmd': getattr(CC, 'leftBlinker', MISSING),
                'rightBlinker_cmd': getattr(CC, 'rightBlinker', MISSING),

                'controlsState_curvature': controlsState.curvature if controlsState else MISSING,
                'controlsState_lateralControlState': str(controlsState.lateralControlState.which()) if controlsState else 'none',


            }

            self.csv_writer.writerow(row)
            self.row_count += 1

            # Flush every 100 rows to prevent data loss
            if self.row_count % 100 == 0:
                self.csv_file.flush()

        except Exception as e:
            print(f"loggerd: ERROR logging frame: {e}")
            import traceback
            traceback.print_exc()


def loggerd_thread():
    """Main logging daemon thread"""
    print("loggerd: Starting comprehensive logging daemon...")

    logger = ComprehensiveLogger()

    # Subscribe to ALL relevant messages
    sm = messaging.SubMaster([
        'carState',
        'carControl',
        'carOutput',  # Actual output sent to car after safety restrictions
        'testJoystick',
        'liveParameters',
        'controlsState',
        'selfdriveState',
        'can'
    ], frequency=1. / DT_CTRL)

    rk = Ratekeeper(100, print_delay_threshold=None)

    loop_count = 0
    last_logging_state = False

    print("loggerd: Waiting for joystick messages...")
    print("loggerd: Send loggingEnabled=True in testJoystick to start logging")

    try:
        while True:
            sm.update(0)
            loop_count += 1

            # Check if logging should be enabled/disabled
            if sm.valid['testJoystick']:
                joy = sm['testJoystick']
                logging_requested = getattr(joy, 'loggingEnabled', False)

                # State change: start logging
                if logging_requested and not last_logging_state:
                    logger.start_logging()

                # State change: stop logging
                elif not logging_requested and last_logging_state:
                    logger.stop_logging()

                last_logging_state = logging_requested

            # Log data if enabled
            if logger.logging_enabled:
                # Wait for required messages
                if not (sm.valid['carState'] and sm.valid['carControl']):
                    rk.keep_time()
                    continue

                # Get all message data
                CC = sm['carControl']
                controlsState = sm['controlsState'] if sm.valid.get('controlsState', False) else None
                selfdriveState = sm['selfdriveState'] if sm.valid.get('selfdriveState', False) else None

                # Build system state dict
                system_state = {
                    'loop_count': loop_count,
                    'system_enabled': CC.enabled if CC else False,
                    'controls_allowed': CC.enabled if CC else False,
                    'joystick_active': sm.valid['testJoystick'],
                }

                # Log the frame (controlsState and selfdriveState are optional)
                logger.log_frame(sm, CC, controlsState, selfdriveState, system_state)            # Print status every 10 seconds
            if loop_count % 1000 == 0:
                status = "LOGGING" if logger.logging_enabled else "IDLE"
                rows = f" ({logger.row_count} rows)" if logger.logging_enabled else ""
                print(f"loggerd: [{status}]{rows} Loop {loop_count}")

            rk.keep_time()

    except KeyboardInterrupt:
        print("\nloggerd: Shutting down...")
        logger.stop_logging()
    except Exception as e:
        print(f"loggerd: FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        logger.stop_logging()


def main():
    loggerd_thread()


if __name__ == "__main__":
    main()
