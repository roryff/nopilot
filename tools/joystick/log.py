#!/usr/bin/env python3
"""
Simple Joystick Data Logger
Usage: python log.py <log_name>
Example: python log.py accel_test_1
"""

import csv
import time
import sys
from pathlib import Path
from datetime import datetime

from cereal import messaging
from openpilot.common.realtime import DT_CTRL, Ratekeeper


def main():
    if len(sys.argv) < 2:
        print("Usage: python log.py <log_name>")
        print("Example: python log.py accel_test_1")
        sys.exit(1)

    log_name = sys.argv[1]

    # Setup log directory and file
    log_dir = Path("/data/joystick_logs")
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"{log_name}_{timestamp}.csv"

    print(f"Starting logging to: {log_path}")
    print("Press Ctrl+C to stop logging")

    # CSV headers
    headers = [
        # Timestamp
        'timestamp',
        'logMonoTime',
        'loop_count',

        # System State
        'system_enabled',
        'lat_active',
        'long_active',
        'joystick_active',

        # Joystick Inputs
        'joy_axis_0_accel',
        'joy_axis_1_steer',

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

        # Car State - Pedals
        'gas',
        'gasPressed',
        'brake',
        'brakePressed',

        # Car State - Cruise
        'cruiseState_enabled',
        'cruiseState_speed',

        # Car State - Faults
        'accFaulted',

        # Live Parameters
        'liveParameters_valid',
        'liveParameters_angleOffsetDeg',
        'liveParameters_steerRatio',

        # Actuator Commands
        'actuators_accel',
        'actuators_torque',
        'actuators_steeringAngleDeg',
        'actuators_longControlState',

        # Car Output (actual commands sent to car)
        'carOutput_valid',
        'carOutput_accel',
        'carOutput_torque',
        'carOutput_steeringAngleDeg',
        'carOutput_longControlState',

        # Car Control Flags
        'enabled',
        'latActive',
        'longActive',

        # Controls State
        'controlsState_curvature',

        # Selfdrive State
        'selfdriveState_state',
        'selfdriveState_enabled',
        'selfdriveState_active',
    ]

    # Subscribe to messages
    sm = messaging.SubMaster([
        'carState',
        'carControl',
        'carOutput',
        'testJoystick',
        'liveParameters',
        'controlsState',
        'selfdriveState',
    ], frequency=1. / DT_CTRL)

    rk = Ratekeeper(100, print_delay_threshold=None)

    loop_count = 0
    row_count = 0

    try:
        with open(log_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=headers)
            writer.writeheader()
            csvfile.flush()

            print("✓ Logging started!")

            while True:
                sm.update(0)
                loop_count += 1

                # Wait for required messages
                if not (sm.valid['carState'] and sm.valid['carControl']):
                    rk.keep_time()
                    continue

                CS = sm['carState']
                CC = sm['carControl']
                joy = sm['testJoystick'] if sm.valid['testJoystick'] else None
                lp = sm['liveParameters'] if sm.valid['liveParameters'] else None
                carOutput = sm['carOutput'] if sm.valid['carOutput'] else None
                controlsState = sm['controlsState'] if sm.valid['controlsState'] else None
                selfdriveState = sm['selfdriveState'] if sm.valid['selfdriveState'] else None

                # Build row
                row = {
                    # Timestamp
                    'timestamp': time.time(),
                    'logMonoTime': sm.logMonoTime.get('carState', 0),
                    'loop_count': loop_count,

                    # System State
                    'system_enabled': CC.enabled,
                    'lat_active': CC.latActive,
                    'long_active': CC.longActive,
                    'joystick_active': sm.valid['testJoystick'],

                    # Joystick Inputs
                    'joy_axis_0_accel': joy.axes[0] if joy and len(joy.axes) > 0 else 0.0,
                    'joy_axis_1_steer': joy.axes[1] if joy and len(joy.axes) > 1 else 0.0,

                    # Car State - Motion
                    'vEgo': CS.vEgo,
                    'vEgoRaw': CS.vEgoRaw,
                    'aEgo': CS.aEgo,
                    'yawRate': CS.yawRate,
                    'standstill': CS.standstill,
                    'wheelSpeeds_fl': CS.wheelSpeeds.fl,
                    'wheelSpeeds_fr': CS.wheelSpeeds.fr,
                    'wheelSpeeds_rl': CS.wheelSpeeds.rl,
                    'wheelSpeeds_rr': CS.wheelSpeeds.rr,

                    # Car State - Steering
                    'steeringAngleDeg': CS.steeringAngleDeg,
                    'steeringRateDeg': CS.steeringRateDeg,
                    'steeringTorque': CS.steeringTorque,
                    'steeringTorqueEps': CS.steeringTorqueEps,
                    'steeringPressed': CS.steeringPressed,
                    'steerFaultTemporary': CS.steerFaultTemporary,
                    'steerFaultPermanent': CS.steerFaultPermanent,

                    # Car State - Pedals
                    # Car State - Cruise
                    'cruiseState_enabled': CS.cruiseState.enabled,
                    'cruiseState_speed': CS.cruiseState.speed,

                    # Car State - Faults
                    'accFaulted': CS.accFaulted,

                    # Live Parameters
                    'liveParameters_valid': sm.valid['liveParameters'],
                    'liveParameters_angleOffsetDeg': lp.angleOffsetDeg if lp else 0.0,
                    'liveParameters_steerRatio': lp.steerRatio if lp else 0.0,

                    # Actuator Commands
                    'actuators_accel': CC.actuators.accel,
                    'actuators_torque': CC.actuators.torque,
                    'actuators_steeringAngleDeg': CC.actuators.steeringAngleDeg,
                    'actuators_longControlState': str(CC.actuators.longControlState),

                    # Car Output
                    'carOutput_valid': sm.valid['carOutput'],
                    'carOutput_accel': carOutput.actuatorsOutput.accel if carOutput else 0.0,
                    'carOutput_torque': carOutput.actuatorsOutput.torque if carOutput else 0.0,
                    'carOutput_steeringAngleDeg': carOutput.actuatorsOutput.steeringAngleDeg if carOutput else 0.0,
                    'carOutput_longControlState': str(carOutput.actuatorsOutput.longControlState) if carOutput else 'none',

                    # Car Control Flags
                    'enabled': CC.enabled,
                    'latActive': CC.latActive,
                    'longActive': CC.longActive,

                    # Controls State
                    'controlsState_curvature': controlsState.curvature if controlsState else 0.0,

                    # Selfdrive State
                    'selfdriveState_state': str(selfdriveState.state) if selfdriveState else '',
                    'selfdriveState_enabled': selfdriveState.enabled if selfdriveState else False,
                    'selfdriveState_active': selfdriveState.active if selfdriveState else False,
                }

                writer.writerow(row)
                row_count += 1

                # Flush every 100 rows
                if row_count % 100 == 0:
                    csvfile.flush()

                # Print status every 10 seconds
                if loop_count % 1000 == 0:
                    print(f"Logged {row_count} rows...")

                rk.keep_time()

    except KeyboardInterrupt:
        print(f"\n✓ Logging stopped. Wrote {row_count} rows to {log_path}")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
