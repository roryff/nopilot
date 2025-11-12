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
            
            # Live Parameters
            'liveParameters_valid',
            'liveParameters_angleOffsetDeg',
            'liveParameters_angleOffsetAverageDeg',
            'liveParameters_stiffnessFactor',
            'liveParameters_steerRatio',
            'liveParameters_roll',
            
            # Actuator Commands (What we're sending)
            'actuators_accel',
            'actuators_torque',
            'actuators_steeringAngleDeg',
            'actuators_curvature',
            'actuators_speed',
            'actuators_longControlState',
            
            # Car Control Flags
            'enabled',
            'latActive',
            'longActive',
            'leftBlinker_cmd',
            'rightBlinker_cmd',
            
            # Cruise Control Commands
            'cruiseControl_cancel',
            'cruiseControl_override',
            'cruiseControl_resume',
            
            # HUD Control
            'hudControl_setSpeed',
            'hudControl_leadVisible',
            'hudControl_leadDistanceBars',
            'hudControl_visualAlert',
            'hudControl_audibleAlert',
            'hudControl_rightLaneVisible',
            'hudControl_leftLaneVisible',
            'hudControl_rightLaneDepart',
            'hudControl_leftLaneDepart',
            
            # Controls State
            'controlsState_curvature',
            'controlsState_lateralControlState',
            
            # Selfdrive State
            'selfdriveState_state',
            'selfdriveState_enabled',
            'selfdriveState_active',
            'selfdriveState_engageable',
            'selfdriveState_alertText1',
            'selfdriveState_alertText2',
            'selfdriveState_alertStatus',
            'selfdriveState_alertSize',
            
            # CAN Message Stats (if available)
            'can_valid',
            'can_error_count',
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
            lp = sm.get('liveParameters', None)
            carOutput = sm.get('carOutput', None)
            
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
                'joy_axis_0_gb': joy.axes[0] if len(joy.axes) > 0 else 0.0,
                'joy_axis_1_steer': joy.axes[1] if len(joy.axes) > 1 else 0.0,
                'joy_button_count': len(joy.buttons) if hasattr(joy, 'buttons') else 0,
                'joy_logging_enabled': getattr(joy, 'loggingEnabled', False),
                
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
                'steerWarning': CS.steerWarning,
                
                # Car State - Pedals
                'gas': CS.gas,
                'gasPressed': CS.gasPressed,
                'brake': CS.brake,
                'brakePressed': CS.brakePressed,
                'brakeHoldActive': CS.brakeHoldActive,
                'parkingBrake': CS.parkingBrake,
                
                # Car State - Gear & Cruise
                'gearShifter': str(CS.gearShifter),
                'cruiseState_enabled': CS.cruiseState.enabled,
                'cruiseState_available': CS.cruiseState.available,
                'cruiseState_speed': CS.cruiseState.speed,
                'cruiseState_standstill': CS.cruiseState.standstill,
                
                # Car State - Buttons
                'leftBlinker': CS.leftBlinker,
                'rightBlinker': CS.rightBlinker,
                'genericToggle': CS.genericToggle,
                'doorOpen': CS.doorOpen,
                'seatbeltUnlatched': CS.seatbeltUnlatched,
                'espDisabled': CS.espDisabled,
                
                # Car State - Faults
                'stockAeb': CS.stockAeb,
                'stockFcw': CS.stockFcw,
                'espActive': getattr(CS, 'espActive', False),
                'accFaulted': CS.accFaulted,
                
                # Live Parameters
                'liveParameters_valid': sm.valid.get('liveParameters', False) if lp else False,
                'liveParameters_angleOffsetDeg': lp.angleOffsetDeg if lp else 0.0,
                'liveParameters_angleOffsetAverageDeg': lp.angleOffsetAverageDeg if lp else 0.0,
                'liveParameters_stiffnessFactor': lp.stiffnessFactor if lp else 0.0,
                'liveParameters_steerRatio': lp.steerRatio if lp else 0.0,
                'liveParameters_roll': lp.roll if lp else 0.0,
                
                # Actuator Commands
                'actuators_accel': CC.actuators.accel,
                'actuators_torque': CC.actuators.torque,
                'actuators_steeringAngleDeg': CC.actuators.steeringAngleDeg,
                'actuators_curvature': CC.actuators.curvature,
                'actuators_speed': CC.actuators.speed,
                'actuators_longControlState': str(CC.actuators.longControlState),
                
                # Car Output (Actual output sent to car after safety restrictions)
                'carOutput_valid': sm.valid.get('carOutput', False),
                'carOutput_accel': carOutput.actuatorsOutput.accel if carOutput else 0.0,
                'carOutput_torque': carOutput.actuatorsOutput.torque if carOutput else 0.0,
                'carOutput_steeringAngleDeg': carOutput.actuatorsOutput.steeringAngleDeg if carOutput else 0.0,
                'carOutput_curvature': carOutput.actuatorsOutput.curvature if carOutput else 0.0,
                'carOutput_speed': carOutput.actuatorsOutput.speed if carOutput else 0.0,
                'carOutput_longControlState': str(carOutput.actuatorsOutput.longControlState) if carOutput else 'none',
                
                # Car Control Flags
                'enabled': CC.enabled,
                'latActive': CC.latActive,
                'longActive': CC.longActive,
                'leftBlinker_cmd': CC.leftBlinker,
                'rightBlinker_cmd': CC.rightBlinker,
                
                # Cruise Control Commands
                'cruiseControl_cancel': CC.cruiseControl.cancel,
                'cruiseControl_override': CC.cruiseControl.override,
                'cruiseControl_resume': CC.cruiseControl.resume,
                
                # HUD Control
                'hudControl_setSpeed': CC.hudControl.setSpeed,
                'hudControl_leadVisible': CC.hudControl.leadVisible,
                'hudControl_leadDistanceBars': CC.hudControl.leadDistanceBars,
                'hudControl_visualAlert': str(CC.hudControl.visualAlert),
                'hudControl_audibleAlert': str(CC.hudControl.audibleAlert),
                'hudControl_rightLaneVisible': CC.hudControl.rightLaneVisible,
                'hudControl_leftLaneVisible': CC.hudControl.leftLaneVisible,
                'hudControl_rightLaneDepart': CC.hudControl.rightLaneDepart,
                'hudControl_leftLaneDepart': CC.hudControl.leftLaneDepart,
                
                # Controls State
                'controlsState_curvature': controlsState.curvature,
                'controlsState_lateralControlState': str(controlsState.lateralControlState.which()),
                
                # Selfdrive State
                'selfdriveState_state': str(selfdriveState.state),
                'selfdriveState_enabled': selfdriveState.enabled,
                'selfdriveState_active': selfdriveState.active,
                'selfdriveState_engageable': selfdriveState.engageable,
                'selfdriveState_alertText1': selfdriveState.alertText1,
                'selfdriveState_alertText2': selfdriveState.alertText2,
                'selfdriveState_alertStatus': str(selfdriveState.alertStatus),
                'selfdriveState_alertSize': str(selfdriveState.alertSize),
                
                # CAN Stats
                'can_valid': sm.valid.get('can', True),
                'can_error_count': getattr(sm.get('can', None), 'canErrorCounter', 0) if sm.get('can') else 0,
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
            if sm.updated['testJoystick'] and sm.valid['testJoystick']:
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
                controlsState = sm.get('controlsState', None)
                selfdriveState = sm.get('selfdriveState', None)
                
                # Build system state dict
                system_state = {
                    'loop_count': loop_count,
                    'system_enabled': CC.enabled if CC else False,
                    'controls_allowed': CC.enabled if CC else False,
                    'joystick_active': sm.valid['testJoystick'],
                }
                
                # Log the frame
                if controlsState and selfdriveState:
                    logger.log_frame(sm, CC, controlsState, selfdriveState, system_state)
            
            # Print status every 10 seconds
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
