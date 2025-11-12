#!/usr/bin/env python3
"""
ADB Bridge Server - runs on comma device
Receives joystick commands via TCP and publishes to testJoystick
Low-latency bridge for teledriving via ADB
"""
import sys
import time
import json
import socket
import threading
import select

# Import comma device modules
from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper


# Global message publisher
pm = None
last_joy_time = 0

def handle_client_socket(client_sock, client_addr):
    """Handle a single client connection"""
    global last_joy_time
    print(f"Client connected: {client_addr}", file=sys.stderr, flush=True)
    client_file = client_sock.makefile('rw', buffering=1)

    try:
        while True:
            # Read command from client
            line = client_file.readline()
            if not line:
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
                    global msg_count
                    msg_count = globals().get('msg_count', 0) + 1
                    if msg_count % 20 == 0:
                        log_status = "[LOG]" if logging_enabled else ""
                        print(f'\rJoystick: gb={axes[0]:+.3f}, steer={axes[1]:+.3f} {log_status}', end='', flush=True)

                    # No ack needed for joystick - running at 100Hz

                elif cmd_type == 'ping':
                    # Ping for latency measurement
                    response = {
                        'type': 'pong',
                        'client_time': cmd.get('time', 0),
                        'server_recv_time': recv_time,
                        'server_send_time': time.time(),
                        'seq': cmd.get('seq', 0)
                    }
                    client_file.write(json.dumps(response) + '\n')
                    client_file.flush()

                else:
                    # Unknown command
                    response = {
                        'type': 'error',
                        'error': f'Unknown command type: {cmd_type}',
                        'server_time': time.time(),
                        'seq': cmd.get('seq', 0)
                    }
                    client_file.write(json.dumps(response) + '\n')
                    client_file.flush()

            except json.JSONDecodeError as e:
                error_response = {
                    'type': 'error',
                    'error': f'JSON decode error: {str(e)}',
                    'server_time': time.time()
                }
                client_file.write(json.dumps(error_response) + '\n')
                client_file.flush()

    except Exception as e:
        print(f"Client handler error: {e}", file=sys.stderr, flush=True)
    finally:
        client_sock.close()
        print(f"Client disconnected: {client_addr}", file=sys.stderr, flush=True)


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

def main():
    global pm
    import argparse
    parser = argparse.ArgumentParser(description='ADB Bridge Server - Joystick Bridge')
    parser.add_argument('--port', type=int, default=5555, help='TCP port to listen on')
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Host to bind to')

    # Parse args, but don't fail if running as a module without args
    try:
        args = parser.parse_args()
    except SystemExit:
        # If argparse fails (e.g., running as module), use defaults
        class DefaultArgs:
            port = 5555
            host = '127.0.0.1'
        args = DefaultArgs()

    # Initialize message publisher
    pm = messaging.PubMaster(['testJoystick'])

    # Enable joystick debug mode

    # Start watchdog thread
    watchdog = threading.Thread(target=watchdog_thread, daemon=True)
    watchdog.start()

    print(f"ADB Bridge Server Starting (Joystick mode on {args.host}:{args.port})", file=sys.stderr, flush=True)

    # Set TCP_NODELAY for low latency
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server_sock.bind((args.host, args.port))
    server_sock.listen(5)

    print(f"Server listening on {args.host}:{args.port}", file=sys.stderr, flush=True)
    print("Waiting for ROS joystick bridge client...", file=sys.stderr, flush=True)

    try:
        while True:
            client_sock, client_addr = server_sock.accept()
            # Set TCP_NODELAY on client socket too
            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # Handle each client in a new thread
            client_thread = threading.Thread(
                target=handle_client_socket,
                args=(client_sock, client_addr),
                daemon=True
            )
            client_thread.start()
    except KeyboardInterrupt:
        print("\nServer stopped", file=sys.stderr, flush=True)
    finally:
        server_sock.close()

if __name__ == '__main__':
    main()
