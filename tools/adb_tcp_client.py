#!/usr/bin/env python3
"""
ADB TCP Client - runs on comma device
Connects to remote TCP server (thor.lan:3000) and bridges joystick/sensor data
This is the client version that initiates connection to remote server
"""
import sys
import time
import json
import math
import socket
import threading

# Import comma device modules
from cereal import messaging
from openpilot.common.realtime import DT_CTRL, Ratekeeper


# Global message publisher
pm = None
last_joy_time = 0
server_socket = None
server_socket_lock = threading.Lock()


class ServerConnection:
    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()
        self.frame_count = 0
        self.last_print_time = time.time()


def send_json(connection, payload):
    data = (json.dumps(payload, separators=(',', ':')) + '\n').encode('utf-8')
    with connection.lock:
        connection.sock.sendall(data)


def connect_to_server(host, port, max_retries=10):
    """Attempt to connect to remote TCP server with retry logic"""
    for attempt in range(max_retries):
        try:
            print(f"Attempting to connect to {host}:{port} (attempt {attempt + 1}/{max_retries})",
                  file=sys.stderr, flush=True)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect((host, port))
            print(f"Connected to {host}:{port}", file=sys.stderr, flush=True)
            return sock
        except socket.error as e:
            print(f"Connection failed: {e}", file=sys.stderr, flush=True)
            if attempt < max_retries - 1:
                time.sleep(2 ** min(attempt, 3))  # Exponential backoff, max 8 seconds
            else:
                print(f"Failed to connect after {max_retries} attempts", file=sys.stderr, flush=True)
                return None


def handle_server_connection(host, port):
    """Handle connection to remote server"""
    global last_joy_time, server_socket, pm

    while True:
        # Connect to server
        sock = connect_to_server(host, port)
        if sock is None:
            time.sleep(5)
            continue

        with server_socket_lock:
            server_socket = ServerConnection(sock)

        server_file = sock.makefile('r', buffering=1)

        try:
            while True:
                # Read command from server
                line = server_file.readline()
                if not line:
                    print(f"Server closed connection", file=sys.stderr, flush=True)
                    break

                line = line.strip()
                if not line:
                    continue

                recv_time = time.time()

                try:
                    # Parse command
                    cmd = json.loads(line)
                    cmd_type = cmd.get('type', 'unknown')

                    if cmd_type == 'joystick':
                        # Joystick command - publish to testJoystick
                        axes = cmd.get('axes', [0.0, 0.0])
                        logging_enabled = cmd.get('loggingEnabled', False)

                        # Create and send testJoystick message
                        joystick_msg = messaging.new_message('testJoystick')
                        joystick_msg.valid = True
                        joystick_msg.testJoystick.axes = axes
                        joystick_msg.testJoystick.loggingEnabled = logging_enabled
                        pm.send('testJoystick', joystick_msg)

                        last_joy_time = recv_time

                        # Debug: print every 20 messages (at 100Hz = 5Hz output)
                        msg_count = globals().get('msg_count', 0) + 1
                        globals()['msg_count'] = msg_count
                        if msg_count % 20 == 0:
                            log_status = "[LOG]" if logging_enabled else ""
                            print(f'\rJoystick: gb={axes[0]:+.3f}, steer={axes[1]:+.3f} {log_status}', end='', flush=True)

                    elif cmd_type == 'ping':
                        # Ping for latency measurement
                        response = {
                            'type': 'pong',
                            'client_time': cmd.get('time', 0),
                            'server_recv_time': recv_time,
                            'server_send_time': time.time(),
                            'seq': cmd.get('seq', 0)
                        }
                        with server_socket_lock:
                            if server_socket and server_socket.sock == sock:
                                send_json(server_socket, response)

                    else:
                        # Unknown command
                        response = {
                            'type': 'error',
                            'error': f'Unknown command type: {cmd_type}',
                            'server_time': time.time(),
                            'seq': cmd.get('seq', 0)
                        }
                        with server_socket_lock:
                            if server_socket and server_socket.sock == sock:
                                send_json(server_socket, response)

                except json.JSONDecodeError as e:
                    error_response = {
                        'type': 'error',
                        'error': f'JSON decode error: {str(e)}',
                        'server_time': time.time()
                    }
                    with server_socket_lock:
                        if server_socket and server_socket.sock == sock:
                            send_json(server_socket, error_response)

        except Exception as e:
            print(f"Connection handler error: {e}", file=sys.stderr, flush=True)
        finally:
            try:
                sock.close()
            except Exception:
                pass
            with server_socket_lock:
                if server_socket and server_socket.sock == sock:
                    server_socket = None
            print(f"Disconnected from server, will attempt to reconnect...", file=sys.stderr, flush=True)


def watchdog_thread():
    """Monitor for joystick timeout and reset to neutral"""
    global last_joy_time
    while True:
        time.sleep(0.1)
        if last_joy_time > 0 and (time.time() - last_joy_time) > 0.5:
            # No joystick data for 500ms - send neutral position
            joystick_msg = messaging.new_message('testJoystick')
            joystick_msg.valid = True
            joystick_msg.testJoystick.axes = [0.0, 0.0]
            joystick_msg.testJoystick.loggingEnabled = False
            pm.send('testJoystick', joystick_msg)
            last_joy_time = 0  # Reset to avoid spamming


def build_sensor_payload(sm, loop_count):
    def safe_get(obj, attr, default=None):
        return getattr(obj, attr, default) if obj is not None else default

    def safe_axis(axes, idx, default=None):
        if axes is None:
            return default
        try:
            return axes[idx] if len(axes) > idx else default
        except Exception:
            return default

    MISSING = None

    CS = sm['carState'] if sm.valid.get('carState', False) else None
    CC = sm['carControl'] if sm.valid.get('carControl', False) else None
    joy = sm['testJoystick'] if sm.valid.get('testJoystick', False) else None
    controlsState = sm['controlsState'] if sm.valid.get('controlsState', False) else None
    selfdriveState = sm['selfdriveState'] if sm.valid.get('selfdriveState', False) else None
    carOutput = sm['carOutput'] if sm.valid.get('carOutput', False) else None

    wheel_speeds = safe_get(CS, 'wheelSpeeds', None)
    cruise_state = safe_get(CS, 'cruiseState', None)
    actuators = safe_get(CC, 'actuators', None)
    actuators_output = safe_get(carOutput, 'actuatorsOutput', None)

    # Determine speed sign: negative when in reverse
    in_reverse = 'reverse' in str(safe_get(CS, 'gearShifter', '')).lower()
    speed_sign = -1 if in_reverse else 1

    return {
        'type': 'sensor',
        'timestamp': sm.logMonoTime.get('carState', 0),
        'logMonoTime': sm.logMonoTime.get('carState', 0),
        'loop_count': loop_count,

        'system_enabled': safe_get(CC, 'enabled', False),
        'controls_allowed': safe_get(CC, 'enabled', False),
        'lat_active': safe_get(CC, 'latActive', False),
        'long_active': safe_get(CC, 'longActive', False),
        'joystick_active': sm.valid.get('testJoystick', False),

        'joy_axis_0_gb': safe_axis(safe_get(joy, 'axes', None), 0, MISSING) if joy else MISSING,
        'joy_axis_1_steer': safe_axis(safe_get(joy, 'axes', None), 1, MISSING) if joy else MISSING,
        'joy_button_count': len(safe_get(joy, 'buttons', [])) if joy else MISSING,
        'joy_logging_enabled': safe_get(joy, 'loggingEnabled', False),

        # Negate speed when in reverse so callers see signed speed
        'vEgo': speed_sign * safe_get(CS, 'vEgo', 0.0) if safe_get(CS, 'vEgo', MISSING) is not None else MISSING,
        'vEgoRaw': speed_sign * safe_get(CS, 'vEgoRaw', 0.0) if safe_get(CS, 'vEgoRaw', MISSING) is not None else MISSING,
        'aEgo': safe_get(CS, 'aEgo', MISSING),
        'yawRate': safe_get(CS, 'yawRate', MISSING),
        'standstill': safe_get(CS, 'standstill', False),
        'wheelSpeeds_fl': wheel_speeds.fl if wheel_speeds is not None else MISSING,
        'wheelSpeeds_fr': wheel_speeds.fr if wheel_speeds is not None else MISSING,
        'wheelSpeeds_rl': wheel_speeds.rl if wheel_speeds is not None else MISSING,
        'wheelSpeeds_rr': wheel_speeds.rr if wheel_speeds is not None else MISSING,

        'steeringAngleDeg': safe_get(CS, 'steeringAngleDeg', MISSING),
        'steeringRateDeg': safe_get(CS, 'steeringRateDeg', MISSING),
        'steeringTorque': safe_get(CS, 'steeringTorque', MISSING),
        'steeringTorqueEps': safe_get(CS, 'steeringTorqueEps', MISSING),
        'steeringPressed': safe_get(CS, 'steeringPressed', MISSING),
        'steerFaultTemporary': safe_get(CS, 'steerFaultTemporary', MISSING),
        'steerFaultPermanent': safe_get(CS, 'steerFaultPermanent', MISSING),
        'steerWarning': safe_get(CS, 'steerWarning', MISSING),

        'leftBlindspot': safe_get(CS, 'leftBlindspot', MISSING),
        'rightBlindspot': safe_get(CS, 'rightBlindspot', MISSING),

        'gas': safe_get(CS, 'gas', MISSING),
        'gasPressed': safe_get(CS, 'gasPressed', MISSING),
        'brake': safe_get(CS, 'brake', MISSING),
        'brakePressed': safe_get(CS, 'brakePressed', MISSING),
        'brakeHoldActive': safe_get(CS, 'brakeHoldActive', MISSING),
        'parkingBrake': safe_get(CS, 'parkingBrake', MISSING),

        'gearShifter': str(safe_get(CS, 'gearShifter', 'unknown')),
        'cruiseState_enabled': safe_get(cruise_state, 'enabled', MISSING),
        'cruiseState_available': safe_get(cruise_state, 'available', MISSING),
        'cruiseState_speed': safe_get(cruise_state, 'speed', MISSING),
        'cruiseState_standstill': safe_get(cruise_state, 'standstill', MISSING),

        'leftBlinker': safe_get(CS, 'leftBlinker', MISSING),
        'rightBlinker': safe_get(CS, 'rightBlinker', MISSING),
        'genericToggle': safe_get(CS, 'genericToggle', MISSING),
        'doorOpen': safe_get(CS, 'doorOpen', MISSING),
        'seatbeltUnlatched': safe_get(CS, 'seatbeltUnlatched', MISSING),
        'espDisabled': safe_get(CS, 'espDisabled', MISSING),

        'stockAeb': safe_get(CS, 'stockAeb', False),
        'stockFcw': safe_get(CS, 'stockFcw', False),
        'espActive': safe_get(CS, 'espActive', False),
        'accFaulted': safe_get(CS, 'accFaulted', False),

        'actuators_accel': safe_get(actuators, 'accel', MISSING),
        'actuators_torque': safe_get(actuators, 'torque', MISSING),
        'actuators_steeringAngleDeg': safe_get(actuators, 'steeringAngleDeg', MISSING),
        'actuators_curvature': safe_get(actuators, 'curvature', MISSING),
        'actuators_speed': safe_get(actuators, 'speed', MISSING),
        'actuators_longControlState': str(safe_get(actuators, 'longControlState', 'off')),

        'carOutput_valid': sm.valid.get('carOutput', False),
        'carOutput_accel': safe_get(actuators_output, 'accel', MISSING),
        'carOutput_torque': safe_get(actuators_output, 'torque', MISSING),
        'carOutput_steeringAngleDeg': safe_get(actuators_output, 'steeringAngleDeg', MISSING),
        'carOutput_curvature': safe_get(actuators_output, 'curvature', MISSING),
        'carOutput_speed': safe_get(actuators_output, 'speed', MISSING),
        'carOutput_longControlState': str(safe_get(actuators_output, 'longControlState', 'off')),

        'enabled': safe_get(CC, 'enabled', MISSING),
        'latActive': safe_get(CC, 'latActive', MISSING),
        'longActive': safe_get(CC, 'longActive', MISSING),
        'leftBlinker_cmd': safe_get(CC, 'leftBlinker', MISSING),
        'rightBlinker_cmd': safe_get(CC, 'rightBlinker', MISSING),

        'controlsState_curvature': safe_get(controlsState, 'curvature', MISSING),
        'controlsState_lateralControlState': str(controlsState.lateralControlState.which()) if controlsState else 'none',
        'selfdriveState': str(selfdriveState.state) if selfdriveState else 'none',
    }


def build_debug_sensor_payload(loop_count, t):
    # Cycle through drive and reverse every ~15 seconds for demo purposes
    in_reverse = (t % 30.0) > 15.0
    gear = 'reverse' if in_reverse else 'drive'
    speed_sign = -1 if in_reverse else 1

    raw_speed = 10.0 + 5.0 * math.sin(t * 0.6)
    speed = speed_sign * raw_speed
    accel = speed_sign * 0.5 * math.sin(t * 1.2)
    steer = 10.0 * math.sin(t * 0.9)
    yaw = 0.2 * math.sin(t * 0.7)
    standstill = raw_speed < 0.2

    return {
        'type': 'sensor',
        'timestamp': int(time.monotonic() * 1e9),
        'logMonoTime': int(time.time() * 1e9),
        'loop_count': loop_count,

        'system_enabled': True,
        'controls_allowed': True,
        'lat_active': True,
        'long_active': True,
        'joystick_active': False,

        'joy_axis_0_gb': 0.0,
        'joy_axis_1_steer': 0.0,
        'joy_button_count': 0,
        'joy_logging_enabled': False,

        'vEgo': speed,
        'vEgoRaw': speed,
        'aEgo': accel,
        'yawRate': yaw,
        'standstill': standstill,
        'wheelSpeeds_fl': max(raw_speed - 0.2, 0.0),
        'wheelSpeeds_fr': max(raw_speed + 0.2, 0.0),
        'wheelSpeeds_rl': max(raw_speed - 0.1, 0.0),
        'wheelSpeeds_rr': max(raw_speed + 0.1, 0.0),

        'steeringAngleDeg': steer,
        'steeringRateDeg': 5.0 * math.cos(t * 0.9),
        'steeringTorque': 1.5 * math.sin(t * 0.9),
        'steeringTorqueEps': 1.2 * math.sin(t * 0.9),
        'steeringPressed': False,
        'steerFaultTemporary': False,
        'steerFaultPermanent': False,
        'steerWarning': False,

        'leftBlindspot': False,
        'rightBlindspot': False,

        'gas': max(accel, 0.0),
        'gasPressed': accel > 0.1,
        'brake': max(-accel, 0.0),
        'brakePressed': accel < -0.1,
        'brakeHoldActive': False,
        'parkingBrake': False,

        'gearShifter': gear,
        'cruiseState_enabled': True,
        'cruiseState_available': True,
        'cruiseState_speed': raw_speed,
        'cruiseState_standstill': standstill,

        'leftBlinker': False,
        'rightBlinker': False,
        'genericToggle': False,
        'doorOpen': False,
        'seatbeltUnlatched': False,
        'espDisabled': False,

        'stockAeb': False,
        'stockFcw': False,
        'espActive': False,
        'accFaulted': False,

        'actuators_accel': accel,
        'actuators_torque': 0.0,
        'actuators_steeringAngleDeg': steer,
        'actuators_curvature': 0.0,
        'actuators_speed': speed,
        'actuators_longControlState': 'pid',

        'carOutput_valid': True,
        'carOutput_accel': accel,
        'carOutput_torque': 0.0,
        'carOutput_steeringAngleDeg': steer,
        'carOutput_curvature': 0.0,
        'carOutput_speed': speed,
        'carOutput_longControlState': 'pid',

        'enabled': True,
        'latActive': True,
        'longActive': True,
        'leftBlinker_cmd': False,
        'rightBlinker_cmd': False,

        'controlsState_curvature': 0.0,
        'controlsState_lateralControlState': 'lateralActive',
        'selfdriveState': 'enabled',
    }


def sensor_broadcast_thread(debug_sensors=False):
    sm = None
    if not debug_sensors:
        sm = messaging.SubMaster([
            'carState',
            'carControl',
            'carOutput',
            'testJoystick',
            'liveParameters',
            'controlsState',
            'selfdriveState',
            'can'
        ], frequency=1.0 / DT_CTRL)

    rk = Ratekeeper(100, print_delay_threshold=None)
    loop_count = 0

    while True:
        if sm is not None:
            sm.update(0)
        loop_count += 1

        with server_socket_lock:
            current_socket = server_socket

        if current_socket:
            if debug_sensors:
                payload = build_debug_sensor_payload(loop_count, time.time())
            else:
                payload = build_sensor_payload(sm, loop_count)

            try:
                send_json(current_socket, payload)
                current_socket.frame_count += 1

                # Print Hz every 5 seconds
                current_time = time.time()
                if current_time - current_socket.last_print_time >= 5.0:
                    hz = current_socket.frame_count / (current_time - current_socket.last_print_time)
                    print(f"Server connection: {hz:.1f} Hz ({current_socket.frame_count} frames in 5s)", file=sys.stderr, flush=True)
                    current_socket.frame_count = 0
                    current_socket.last_print_time = current_time

            except Exception as e:
                print(f"Error sending sensor data: {e}", file=sys.stderr, flush=True)

        rk.keep_time()


def main():
    global pm
    import argparse
    parser = argparse.ArgumentParser(description='ADB TCP Client - Joystick Bridge')
    parser.add_argument('--host', type=str, default='naplab.lan', help='Remote host to connect to')
    parser.add_argument('--port', type=int, default=3000, help='Remote port to connect to')
    parser.add_argument('--debug-sensors', action='store_true', help='Send spoofed sensor data without CAN')

    # Parse args, but don't fail if running as a module without args
    try:
        args = parser.parse_args()
    except SystemExit:
        # If argparse fails (e.g., running as module), use defaults
        class DefaultArgs:
            host = 'thor.lan'
            port = 3000
            debug_sensors = False
        args = DefaultArgs()

    # Initialize message publisher
    pm = messaging.PubMaster(['testJoystick'])

    # Start watchdog thread
    watchdog = threading.Thread(target=watchdog_thread, daemon=True)
    watchdog.start()

    sensor_thread = threading.Thread(
        target=sensor_broadcast_thread,
        args=(args.debug_sensors,),
        daemon=True
    )
    sensor_thread.start()

    print(f"ADB TCP Client Starting (connecting to {args.host}:{args.port})", file=sys.stderr, flush=True)

    # Main connection loop - handles reconnection
    server_thread = threading.Thread(
        target=handle_server_connection,
        args=(args.host, args.port),
        daemon=False
    )
    server_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClient stopped", file=sys.stderr, flush=True)


if __name__ == '__main__':
    main()
