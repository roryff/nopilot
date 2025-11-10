#!/usr/bin/env python3
"""
ADB Joystick Bridge - Physical joystick to comma device via ADB
Reads from a physical joystick on PC and sends to comma device via ADB
Works like joystick_udp.py but uses ADB instead of local messaging
"""
import socket
import json
import time
import sys
import numpy as np
from inputs import UnpluggedError, get_gamepad

EXPO = 0.4


class ADBJoystickClient:
    """Low-latency client for forwarding joystick commands"""

    def __init__(self, use_adb=True, host='127.0.0.1', port=5555):
        """
        Initialize the ADB Joystick client

        Args:
            use_adb: If True, use ADB forward. If False, connect directly.
            host: Host to connect to (default: 127.0.0.1)
            port: Port to connect to (default: 5555)
        """
        self.use_adb = use_adb
        self.host = host
        self.port = port
        self.sock = None
        self.seq = 0

    def connect(self):
        """Establish connection (and optionally set up ADB forwarding)"""
        if self.use_adb:
            import subprocess
            # Set up ADB forward
            print(f"Setting up ADB forward tcp:{self.port} tcp:{self.port}")
            result = subprocess.run(
                ['adb', 'forward', f'tcp:{self.port}', f'tcp:{self.port}'],
                capture_output=True,
                text=True
            )
            if result.returncode != 0:
                print(f"Warning: ADB forward failed: {result.stderr}")

        # Connect to the server
        print(f"Connecting to {self.host}:{self.port}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Enable TCP_NODELAY for low latency
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.host, self.port))
        print("Connected!")

    def send_joystick(self, axes):
        """
        Send joystick axes to the server

        Args:
            axes: List of two floats [longitudinal, lateral] (gb, steer)
        """
        cmd = {
            'type': 'joystick',
            'axes': axes,
            'time': time.time(),
            'seq': self.seq
        }
        self.seq += 1

        cmd_json = json.dumps(cmd) + '\n'
        self.sock.sendall(cmd_json.encode())
        # No ack expected for joystick commands

    def ping(self):
        """Send a ping and measure round-trip time"""
        start_time = time.time()

        cmd = {
            'type': 'ping',
            'time': start_time,
            'seq': self.seq
        }
        self.seq += 1

        cmd_json = json.dumps(cmd) + '\n'
        self.sock.sendall(cmd_json.encode())

        # Wait for response with timeout
        import select
        ready = select.select([self.sock], [], [], 2.0)

        if not ready[0]:
            return {'success': False}

        try:
            data = self.sock.recv(4096).decode()
            if not data:
                return {'success': False}

            response = json.loads(data.strip().split('\n')[0])

            if response['type'] == 'pong':
                end_time = time.time()
                rtt = (end_time - start_time) * 1000  # ms

                return {
                    'success': True,
                    'rtt_ms': rtt,
                    'server_processing_ms': (response['server_send_time'] - response['server_recv_time']) * 1000
                }
        except Exception as e:
            print(f"Ping error: {e}")

        return {'success': False}

    def close(self):
        """Close the connection"""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass


class Joystick:
    def __init__(self):
        # This class supports a PlayStation 5 DualSense controller
        # Detects PC vs comma device and adjusts mapping
        self.cancel_button = 'BTN_NORTH'  # BTN_NORTH=X/triangle

        # Check if running on PC



        accel_axis = 'ABS_Z'
        steer_axis = 'ABS_RX'
        self.flip_map = {'ABS_RZ': accel_axis}


        self.min_axis_value = {accel_axis: 0., steer_axis: 0.}
        self.max_axis_value = {accel_axis: 255., steer_axis: 255.}
        self.axes_values = {accel_axis: 0., steer_axis: 0.}
        self.axes_order = [accel_axis, steer_axis]
        self.cancel = False

    def update(self):
        try:
            joystick_event = get_gamepad()[0]
        except (OSError, UnpluggedError):
            self.axes_values = dict.fromkeys(self.axes_values, 0.)
            return False

        event = (joystick_event.code, joystick_event.state)

        # flip left trigger to negative accel
        if event[0] in self.flip_map:
            event = (self.flip_map[event[0]], -event[1])

        if event[0] == self.cancel_button:
            if event[1] == 1:
                self.cancel = True
            elif event[1] == 0:   # state 0 is falling edge
                self.cancel = False
        elif event[0] in self.axes_values:
            self.max_axis_value[event[0]] = max(event[1], self.max_axis_value[event[0]])
            self.min_axis_value[event[0]] = min(event[1], self.min_axis_value[event[0]])

            norm = -float(np.interp(event[1], [self.min_axis_value[event[0]], self.max_axis_value[event[0]]], [-1., 1.]))
            norm = norm if abs(norm) > 0.03 else 0.  # center can be noisy, deadzone of 3%
            self.axes_values[event[0]] = EXPO * norm ** 3 + (1 - EXPO) * norm  # less action near center for fine control
        else:
            return False
        return True


class Keyboard:
    def __init__(self):
        try:
            from openpilot.tools.lib.kbhit import KBHit
            self.kb = KBHit()
        except ImportError:
            print("ERROR: Cannot import KBHit. Keyboard mode requires openpilot environment.")
            sys.exit(1)

        self.axis_increment = 0.05  # 5% of full actuation each key press
        self.axes_map = {'w': 'gb', 's': 'gb',
                         'a': 'steer', 'd': 'steer'}
        self.axes_values = {'gb': 0., 'steer': 0.}
        self.axes_order = ['gb', 'steer']
        self.cancel = False

    def update(self):
        # Check if a key is available (non-blocking)
        if not self.kb.kbhit():
            return True  # No key pressed, but keep running

        key = self.kb.getch().lower()
        self.cancel = False
        if key == 'r':
            self.axes_values = dict.fromkeys(self.axes_values, 0.)
        elif key == 'c':
            self.cancel = True
        elif key in self.axes_map:
            axis = self.axes_map[key]
            incr = self.axis_increment if key in ['w', 'a'] else -self.axis_increment
            self.axes_values[axis] = float(np.clip(self.axes_values[axis] + incr, -1, 1))
        else:
            return True  # Unknown key, but keep running
        return True


def send_loop(joystick, client):
    """Main loop: read joystick and send to comma device via ADB"""
    from openpilot.common.realtime import Ratekeeper

    rk = Ratekeeper(100, print_delay_threshold=None)

    print("\n" + "="*60)
    print("Sending joystick data to comma device...")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")

    frame = 0
    try:
        while True:
            # Update joystick state
            joystick.update()

            # Get axes values in order [gb, steer] or [accel_axis, steer_axis]
            axes = [joystick.axes_values[ax] for ax in joystick.axes_order]

            # Send to comma device
            try:
                client.send_joystick(axes)
            except Exception as e:
                print(f"\nConnection error: {e}")
                print("Server may have stopped. Reconnecting...")
                time.sleep(1)
                try:
                    client.connect()
                    print("Reconnected!")
                except Exception as e2:
                    print(f"Reconnect failed: {e2}")
                    break

            # Print status every 5 frames (20 Hz for more responsive display)
            if frame % 5 == 0:
                values_str = ', '.join(f'{name}: {joystick.axes_values[name]:.2f}' for name in joystick.axes_order)
                print(f'\r{values_str}', end='', flush=True)

            frame += 1
            rk.keep_time()

    except KeyboardInterrupt:
        print("\n\nStopping...")
        # Send neutral position before disconnecting
        client.send_joystick([0.0, 0.0])
        time.sleep(0.1)


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Forward physical joystick to comma device via ADB',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This tool reads from a physical joystick/gamepad connected to your PC
and forwards the commands to the comma device via ADB.

Works like joystick_udp.py but sends over ADB instead of local messaging.

Requirements:
  - ADB connection to comma device
  - Server running on comma: python3 tools/adb_bridge_server.py
  - Physical joystick connected to PC

Gamepad mapping:
  - Right stick horizontal: Steering
  - Right trigger (R2): Gas/acceleration
  - Left trigger (L2): Brake (negative acceleration)
  - Triangle/Y button: Cancel
        """
    )
    parser.add_argument('--keyboard', action='store_true',
                       help='Use keyboard instead of gamepad (W/S=gas/brake, A/D=steer)')
    parser.add_argument('--no-adb', action='store_true',
                       help='Connect directly without ADB forwarding')
    parser.add_argument('--port', type=int, default=5555,
                       help='Port to connect to (default: 5555)')
    args = parser.parse_args()

    # Check for inputs library
    if not args.keyboard:
        try:
            import inputs
        except ImportError:
            print("ERROR: 'inputs' library not found!")
            print("\nInstall with: pip install inputs")
            print("Or use keyboard mode: --keyboard")
            sys.exit(1)

    # Create client
    print("="*60)
    print("ADB Joystick Bridge - Physical Gamepad to Comma Device")
    print("="*60)

    client = ADBJoystickClient(use_adb=not args.no_adb, port=args.port)

    # Connect
    print("\nConnecting to comma device...")
    try:
        client.connect()
    except Exception as e:
        print(f"\nConnection failed: {e}")
        print("\nMake sure:")
        print("  1. Comma device is connected via USB")
        print("  2. ADB is working: adb devices")
        print("  3. Server is running on comma:")
        print("     adb shell 'cd /data/openpilot && python3 tools/adb_bridge_server.py'")
        sys.exit(1)

    # Test connection
    print("\nTesting connection...")
    result = client.ping()
    if result['success']:
        print(f"✓ Connection OK! RTT: {result['rtt_ms']:.2f}ms")
    else:
        print("✗ Warning: Ping failed, but will try to continue...")

    # Create joystick
    print()
    if args.keyboard:
        print('Using keyboard control:')
        print('  W/S: Gas/brake')
        print('  A/D: Steering')
        print('  R: Reset to neutral')
        print('  C: Cancel')
        joystick = Keyboard()
    else:
        print('Using physical gamepad:')
        print('  Right stick horizontal: Steering')
        print('  Right trigger (R2): Gas')
        print('  Left trigger (L2): Brake')
        print('  Triangle/Y: Cancel')
        print('\nMake sure your gamepad is connected!')
        joystick = Joystick()

    # Import Ratekeeper
    try:
        from openpilot.common.realtime import Ratekeeper
    except ImportError:
        print("\nERROR: Cannot import openpilot modules.")
        print("Make sure you're running from the openpilot directory.")
        sys.exit(1)

    # Start main loop
    try:
        send_loop(joystick, client)
    finally:
        client.close()
        print("Disconnected")


if __name__ == '__main__':
    main()
