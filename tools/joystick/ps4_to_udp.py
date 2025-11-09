#!/usr/bin/env python3
"""
PS4 Controller to UDP Bridge
Reads PS4 controller input via evdev and sends normalized values over UDP as JSON.
"""
import sys
import os
import argparse
import socket
import json
import time
from evdev import InputDevice, categorize, ecodes, list_devices

EXPO = 0.4


def clear_screen():
    os.system('clear')


def normalize_value(value, min_val, max_val, output_min=-1, output_max=1):
    """Normalize a value from input range to output range"""
    return output_min + (value - min_val) * (output_max - output_min) / (max_val - min_val)


def create_bar(value, width=40, char='█'):
    """Create a visual bar representation of a value between -1 and 1"""
    if value < -1:
        value = -1
    elif value > 1:
        value = 1

    center = width // 2
    bar = [' '] * width
    bar[center] = '|'  # Center marker

    if value > 0:
        # Positive values go to the right
        fill_length = int(value * center)
        for i in range(center + 1, center + 1 + fill_length):
            if i < width:
                bar[i] = char
    elif value < 0:
        # Negative values go to the left
        fill_length = int(-value * center)
        for i in range(center - fill_length, center):
            if i >= 0:
                bar[i] = char

    return ''.join(bar)


def create_trigger_bar(value, width=20, char='█'):
    """Create a visual bar for trigger values (0 to 1)"""
    if value < 0:
        value = 0
    elif value > 1:
        value = 1

    fill_length = int(value * width)
    bar = [' '] * width
    for i in range(fill_length):
        bar[i] = char

    return ''.join(bar)


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


def list_device_events(device_path):
    """List all event types and codes for a specific device"""
    try:
        device = InputDevice(device_path)
    except FileNotFoundError:
        print(f"Device not found: {device_path}")
        print("Use --list-devices to see available devices")
        return
    except PermissionError:
        print(f"Permission denied accessing {device_path}")
        print("Try running with sudo or add your user to the 'input' group:")
        print("  sudo usermod -a -G input $USER")
        print("Then log out and back in.")
        return

    print(f"Device: {device.name}")
    print(f"Path:   {device.path}")
    print("=" * 80)

    caps = device.capabilities()

    # Map event type codes to names
    event_types = {
        ecodes.EV_KEY: "EV_KEY (Buttons)",
        ecodes.EV_ABS: "EV_ABS (Absolute Axes)",
        ecodes.EV_REL: "EV_REL (Relative Axes)",
        ecodes.EV_MSC: "EV_MSC (Misc)",
        ecodes.EV_SW: "EV_SW (Switches)",
        ecodes.EV_LED: "EV_LED (LEDs)",
        ecodes.EV_SND: "EV_SND (Sounds)",
        ecodes.EV_FF: "EV_FF (Force Feedback)",
    }

    for event_type, event_codes in caps.items():
        if event_type == 0:  # EV_SYN
            continue

        type_name = event_types.get(event_type, f"Unknown ({event_type})")
        print(f"\n{type_name}:")
        print("-" * 80)

        for code_info in event_codes:
            if event_type == ecodes.EV_ABS:
                # For absolute axes, code_info is (code, absinfo)
                code = code_info[0] if isinstance(code_info, tuple) else code_info
                absinfo = code_info[1] if isinstance(code_info, tuple) and len(code_info) > 1 else None

                code_name = ecodes.ABS.get(code, f"ABS_{code}")
                print(f"  {code:3d} (0x{code:02x}): {code_name:20s}", end='')

                if absinfo:
                    print(f" [min={absinfo.min:4d}, max={absinfo.max:4d}, fuzz={absinfo.fuzz}, flat={absinfo.flat}]")
                else:
                    print()

            elif event_type == ecodes.EV_KEY:
                code = code_info[0] if isinstance(code_info, tuple) else code_info

                # Try different key code mappings
                code_name = None
                for key_dict in [ecodes.BTN, ecodes.KEY]:
                    if code in key_dict.values():
                        code_name = [k for k, v in key_dict.items() if v == code][0]
                        break

                if not code_name:
                    code_name = f"KEY_{code}"

                print(f"  {code:3d} (0x{code:02x}): {code_name}")

            elif event_type == ecodes.EV_REL:
                code = code_info[0] if isinstance(code_info, tuple) else code_info
                code_name = ecodes.REL.get(code, f"REL_{code}")
                print(f"  {code:3d} (0x{code:02x}): {code_name}")
            else:
                code = code_info[0] if isinstance(code_info, tuple) else code_info
                print(f"  {code:3d} (0x{code:02x})")

    device.close()


def monitor_and_send(device_path, host='127.0.0.1', port=9999, show_display=True):
    """Monitor PS4 controller and send values over UDP"""
    try:
        gamepad = InputDevice(device_path)
    except FileNotFoundError:
        print(f"Device not found: {device_path}")
        print("Use --list-devices to see available devices")
        return
    except PermissionError:
        print(f"Permission denied accessing {device_path}")
        print("Try running with sudo or add your user to the 'input' group:")
        print("  sudo usermod -a -G input $USER")
        print("Then log out and back in.")
        return

    print(f"Connected to: {gamepad.name}")
    print(f"Sending to: {host}:{port}")
    print("Press Ctrl+C to exit\n")

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Initialize values
    left_stick_x = 0.0
    left_trigger = 0.0
    right_trigger = 0.0

    # PS4 controller axis codes
    # These may vary slightly by controller model, use --list-events to verify
    AXIS_LEFT_X = ecodes.ABS_X        # Left stick X-axis
    AXIS_LEFT_Y = ecodes.ABS_Y        # Left stick Y-axis
    AXIS_RIGHT_X = ecodes.ABS_RX      # Right stick X-axis (alternate: ABS_Z)
    AXIS_RIGHT_Y = ecodes.ABS_RY      # Right stick Y-axis (alternate: ABS_RZ)
    AXIS_LEFT_TRIGGER = ecodes.ABS_Z   # Left trigger (L2) - alternate: ABS_BRAKE
    AXIS_RIGHT_TRIGGER = ecodes.ABS_RZ # Right trigger (R2) - alternate: ABS_GAS

    # Alternative trigger codes
    ALT_LEFT_TRIGGER = ecodes.ABS_BRAKE
    ALT_RIGHT_TRIGGER = ecodes.ABS_GAS

    last_send_time = time.time()
    send_rate = 0.02  # Send every 20ms (50Hz)

    try:
        for event in gamepad.read_loop():
            if event.type == ecodes.EV_ABS:  # Absolute axis events

                # Left stick X-axis movement
                if event.code == AXIS_LEFT_X:
                    # Normalize to -1 to 1 range
                    left_stick_x = normalize_value(event.value, 0, 255, -1, 1)

                # Left trigger
                elif event.code in [AXIS_LEFT_TRIGGER, ALT_LEFT_TRIGGER]:
                    # Normalize to 0 to 1 range
                    left_trigger = normalize_value(event.value, 0, 255, 0, 1)

                # Right trigger
                elif event.code in [AXIS_RIGHT_TRIGGER, ALT_RIGHT_TRIGGER]:
                    # Normalize to 0 to 1 range
                    right_trigger = normalize_value(event.value, 0, 255, 0, 1)

                # Rate limit sending
                current_time = time.time()
                if current_time - last_send_time >= send_rate:
                    # Create JSON payload
                    data = {
                        'left_stick_x': round(left_stick_x, 4),
                        'left_trigger': round(left_trigger, 4),
                        'right_trigger': round(right_trigger, 4)
                    }

                    # Send UDP packet
                    try:
                        sock.sendto(json.dumps(data).encode('utf-8'), (host, port))
                    except Exception as e:
                        if show_display:
                            print(f"Send error: {e}")

                    last_send_time = current_time

                    # Update display
                    if show_display:
                        clear_screen()
                        print("🎮 PS4 Controller → UDP Bridge")
                        print("=" * 80)
                        print(f"Sending to: {host}:{port}")
                        print()

                        # Left stick display
                        stick_bar = create_bar(left_stick_x, 40)
                        print(f"Left Stick X: [{stick_bar}]")
                        print(f"Value: {left_stick_x:+.3f}")
                        print()

                        # Triggers display
                        left_trig_bar = create_trigger_bar(left_trigger, 20)
                        right_trig_bar = create_trigger_bar(right_trigger, 20)

                        print(f"Left Trigger:  [{left_trig_bar}] {left_trigger:.3f}")
                        print(f"Right Trigger: [{right_trig_bar}] {right_trigger:.3f}")
                        print()

                        # Show JSON being sent
                        print(f"JSON: {json.dumps(data)}")
                        print()

                        # Debug info
                        print(f"Raw event - Code: {event.code} (0x{event.code:x}), Value: {event.value}")
                        print()
                        print("Press Ctrl+C to exit")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()
        gamepad.close()


def main():
    parser = argparse.ArgumentParser(
        description='PS4 Controller to UDP Bridge - Send PS4 controller values over UDP as JSON',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List all available input devices
  %(prog)s --list-devices

  # List all events for a specific device
  %(prog)s --list-events /dev/input/event8

  # Monitor and send PS4 controller data
  %(prog)s --device /dev/input/event8

  # Send to a specific host and port
  %(prog)s --device /dev/input/event8 --host 192.168.1.100 --port 9999

  # Send without display (useful for background operation)
  %(prog)s --device /dev/input/event8 --no-display
"""
    )

    parser.add_argument('--list-devices', action='store_true',
                        help='List all available input devices')
    parser.add_argument('--list-events', metavar='DEVICE',
                        help='List all event codes for the specified device')
    parser.add_argument('--device', '-d', metavar='DEVICE',
                        help='Path to input device (e.g., /dev/input/event8)')
    parser.add_argument('--host', default='127.0.0.1',
                        help='Target host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9999,
                        help='Target UDP port (default: 9999)')
    parser.add_argument('--no-display', action='store_true',
                        help='Disable visual display (useful for background operation)')

    args = parser.parse_args()

    # Handle different modes
    if args.list_devices:
        list_input_devices()
    elif args.list_events:
        list_device_events(args.list_events)
    elif args.device:
        monitor_and_send(args.device, args.host, args.port, show_display=not args.no_display)
    else:
        parser.print_help()
        print("\nNo action specified. Use --list-devices to see available devices,")
        print("then use --device to start sending controller data.")


if __name__ == '__main__':
    main()
