#!/usr/bin/env python3
"""
ADB Joystick Bridge - Physical joystick to comma device via ADB
Reads PS4 controller input via evdev and sends to comma device via ADB
Based on working PS4 controller evdev implementation
"""
import socket
import json
import time
import sys
import os
import argparse
import numpy as np
from evdev import InputDevice, categorize, ecodes, list_devices

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

    def send_joystick(self, axes, logging_enabled=False):
        """
        Send joystick axes to the server

        Args:
            axes: List of two floats [longitudinal, lateral] (gb, steer)
            logging_enabled: Boolean to enable/disable logging on device
        """
        cmd = {
            'type': 'joystick',
            'axes': axes,
            'loggingEnabled': logging_enabled,
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


def normalize_value(value, min_val, max_val, output_min=-1, output_max=1):
    """Normalize a value from input range to output range"""
    return output_min + (value - min_val) * (output_max - output_min) / (max_val - min_val)


def list_input_devices():
    """List all available input devices"""
    devices = [InputDevice(path) for path in list_devices()]

    if not devices:
        print("No input devices found.")
        print("Make sure you have permission to access /dev/input/")
        return

    print("Available input devices:")
    print("=" * 80)
    for device in devices:
        print(f"Path:    {device.path}")
        print(f"Name:    {device.name}")
        print(f"Phys:    {device.phys}")
        print(f"Vendor:  {device.info.vendor:04x}")
        print(f"Product: {device.info.product:04x}")
        print("-" * 80)


class Joystick:
    def __init__(self, device_path):
        """Initialize joystick using evdev"""
        try:
            self.gamepad = InputDevice(device_path)
            print(f"Connected to: {self.gamepad.name}")
        except FileNotFoundError:
            print(f"Device not found: {device_path}")
            print("Use --list-devices to see available devices")
            sys.exit(1)
        except PermissionError:
            print(f"Permission denied accessing {device_path}")
            print("Try running with sudo or add your user to the 'input' group:")
            print("  sudo usermod -a -G input $USER")
            print("Then log out and back in.")
            sys.exit(1)

        # PS4 controller axis codes
        self.AXIS_LEFT_X = ecodes.ABS_X        # Left stick X-axis (steering)
        self.AXIS_LEFT_TRIGGER = ecodes.ABS_Z   # Left trigger (L2) - brake
        self.AXIS_RIGHT_TRIGGER = ecodes.ABS_RZ # Right trigger (R2) - gas

        # Alternative trigger codes for different controllers
        self.ALT_LEFT_TRIGGER = ecodes.ABS_BRAKE
        self.ALT_RIGHT_TRIGGER = ecodes.ABS_GAS

        # Button codes
        self.BTN_TRIANGLE = 307  # Triangle button for cancel (BTN_NORTH)
        self.BTN_X = 304          # X button for logging toggle (BTN_SOUTH)
        self.BTN_L1 = 310         # L1 button - left blinker
        self.BTN_R1 = 311         # R1 button - right blinker

        # Current values
        self.steer = 0.0
        self.left_trigger = 0.0  # brake
        self.right_trigger = 0.0  # gas
        self.cancel = False
        self.logging_enabled = False  # Toggle logging
        self.left_blinker = False
        self.right_blinker = False

        # For axes_values compatibility with send_loop
        self.axes_values = {'gb': 0.0, 'steer': 0.0}
        self.axes_order = ['gb', 'steer']

    def apply_expo(self, value):
        """Apply exponential curve for fine control"""
        return EXPO * value ** 3 + (1 - EXPO) * value

    def read_events(self):
        """Read and process controller events (non-blocking)"""
        try:
            # Read all available events without blocking
            for event in self.gamepad.read():
                if event.type == ecodes.EV_ABS:  # Absolute axis events

                    # Left stick X-axis (steering)
                    if event.code == self.AXIS_LEFT_X:
                        # Normalize to -1 to 1 range (inverted so right = positive)
                        raw_steer = normalize_value(event.value, 0, 255, 1, -1)
                        # Apply deadzone
                        if abs(raw_steer) < 0.03:
                            raw_steer = 0.0
                        # Apply expo curve for fine control
                        self.steer = self.apply_expo(raw_steer)

                    # Left trigger (brake - negative acceleration)
                    elif event.code in [self.AXIS_LEFT_TRIGGER, self.ALT_LEFT_TRIGGER]:
                        self.left_trigger = normalize_value(event.value, 0, 255, 0, 1)

                    # Right trigger (gas - positive acceleration)
                    elif event.code in [self.AXIS_RIGHT_TRIGGER, self.ALT_RIGHT_TRIGGER]:
                        self.right_trigger = normalize_value(event.value, 0, 255, 0, 1)

                elif event.type == ecodes.EV_KEY:  # Button events
                    if event.code == self.BTN_TRIANGLE:
                        self.cancel = (event.value == 1)  # 1 = pressed, 0 = released
                    elif event.code == self.BTN_X:
                        if event.value == 1:  # Button pressed (not released)
                            self.logging_enabled = not self.logging_enabled
                            status = "ENABLED" if self.logging_enabled else "DISABLED"
                            print(f"\n*** LOGGING {status} ***")
                    elif event.code == self.BTN_L1:
                        self.left_blinker = (event.value == 1)
                    elif event.code == self.BTN_R1:
                        self.right_blinker = (event.value == 1)

            # Calculate combined gas/brake value
            # Right trigger = positive (gas), left trigger = negative (brake)
            gb = self.right_trigger - self.left_trigger

            # Update axes_values for compatibility
            self.axes_values['gb'] = gb
            self.axes_values['steer'] = self.steer

            return True

        except BlockingIOError:
            # No events available, that's fine
            return True
        except OSError as e:
            print(f"\nController error: {e}")
            return False

    def update(self):
        """Update joystick state - called by send_loop"""
        return self.read_events()


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

            # Get logging state if available (gamepad only)
            logging_enabled = getattr(joystick, 'logging_enabled', False)

            # Send to comma device
            try:
                client.send_joystick(axes, logging_enabled)
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

Gamepad mapping (PS4 controller):
  - Left stick X-axis: Steering
  - Right trigger (R2): Gas/acceleration
  - Left trigger (L2): Brake (negative acceleration)
  - Triangle button: Cancel
  - X button: Toggle data logging (creates huge CSV file)
  - L1 button: Left blinker (if supported)
  - R1 button: Right blinker (if supported)

Examples:
  # List all available input devices
  %(prog)s --list-devices

  # Use specific device
  %(prog)s --device /dev/input/event8

  # Use keyboard instead
  %(prog)s --keyboard
        """
    )
    parser.add_argument('--list-devices', action='store_true',
                       help='List all available input devices and exit')
    parser.add_argument('--device', '-d', metavar='DEVICE',
                       help='Path to input device (e.g., /dev/input/event8)')
    parser.add_argument('--keyboard', action='store_true',
                       help='Use keyboard instead of gamepad (W/S=gas/brake, A/D=steer)')
    parser.add_argument('--no-adb', action='store_true',
                       help='Connect directly without ADB forwarding')
    parser.add_argument('--port', type=int, default=5555,
                       help='Port to connect to (default: 5555)')
    args = parser.parse_args()

    # Handle list devices mode
    if args.list_devices:
        list_input_devices()
        return

    # Check for evdev library
    if not args.keyboard and not args.device:
        print("ERROR: You must specify either --device or --keyboard")
        print("\nUse --list-devices to see available input devices")
        print("Or use --keyboard for keyboard control")
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
        print('  Left stick X-axis: Steering')
        print('  Right trigger (R2): Gas')
        print('  Left trigger (L2): Brake')
        print('  Triangle button: Cancel')
        print('  X button: Toggle logging')
        print('  L1/R1: Blinkers (if supported)')
        print()
        joystick = Joystick(args.device)

        # Set device to non-blocking mode
        import fcntl
        fd = joystick.gamepad.fd
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

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
        if hasattr(joystick, 'gamepad'):
            joystick.gamepad.close()
        print("Disconnected")


if __name__ == '__main__':
    main()
