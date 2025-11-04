#!/usr/bin/env python3
import os
import argparse
import threading
import socket
import json
import time
import numpy as np
from inputs import UnpluggedError, get_gamepad

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import HARDWARE
from openpilot.tools.lib.kbhit import KBHit

EXPO = 0.4


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.axis_increment = 0.05  # 5% of full actuation each key press
    self.axes_map = {'w': 'gb', 's': 'gb',
                     'a': 'steer', 'd': 'steer'}
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

  def update(self):
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
      return False
    return True


class UdpJoystick:
  def __init__(self, port=9999, timeout=0.1):
    self.port = port
    self.timeout = timeout
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False
    self.last_update_time = time.time()

    # Create UDP socket
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.settimeout(self.timeout)
    self.sock.bind(('0.0.0.0', self.port))
    print(f"UDP Joystick listening on port {self.port}")

  def update(self):
    try:
      # Receive UDP packet
      data, addr = self.sock.recvfrom(1024)
      current_time = time.time()

      try:
        # Parse JSON data
        gamepad_data = json.loads(data.decode('utf-8'))

        # Extract values
        left_stick_x = gamepad_data.get('left_stick_x', 0.0)
        left_trigger = gamepad_data.get('left_trigger', 0.0)
        right_trigger = gamepad_data.get('right_trigger', 0.0)

        # Apply deadzone
        left_stick_x = left_stick_x if abs(left_stick_x) > 0.03 else 0.0
        left_trigger = left_trigger if abs(left_trigger) > 0.03 else 0.0
        right_trigger = right_trigger if abs(right_trigger) > 0.03 else 0.0

        # Apply exponential curve for fine control
        left_stick_x = EXPO * (left_stick_x ** 3) + (1 - EXPO) * left_stick_x if left_stick_x != 0 else 0

        # Calculate combined acceleration: right trigger (gas) - left trigger (brake)
        accel = float(right_trigger - left_trigger)
        accel = EXPO * (accel ** 3) + (1 - EXPO) * accel if accel != 0 else 0.0
        accel = float(np.clip(accel, -1.0, 1.0))

        # Update axes values
        self.axes_values['gb'] = float(np.clip(accel, -1.0, 1.0))
        self.axes_values['steer'] = float(np.clip(-left_stick_x, -1.0, 1.0))  # Negative for correct steering direction

        self.last_update_time = current_time
        return True

      except json.JSONDecodeError:
        print(f"Invalid JSON received from {addr}")
        return False
      except Exception as e:
        print(f"Parse error: {e}")
        return False

    except socket.timeout:
      # Check if we haven't received data for too long
      if time.time() - self.last_update_time > 1.0:
        # Reset to neutral if no data received for 1 second
        self.axes_values = {'gb': 0., 'steer': 0.}
      return False
    except Exception as e:
      print(f"UDP receive error: {e}")
      return False


class Joystick:
  def __init__(self):
    # This class supports a PlayStation 5 DualSense controller on the comma 3X
    # TODO: find a way to get this from API or detect gamepad/PC, perhaps "inputs" doesn't support it
    self.cancel_button = 'BTN_NORTH'  # BTN_NORTH=X/triangle
    if HARDWARE.get_device_type() == 'pc':
      accel_axis = 'ABS_Z'
      steer_axis = 'ABS_RX'
      # TODO: once the longcontrol API is finalized, we can replace this with outputting gas/brake and steering
      self.flip_map = {'ABS_RZ': accel_axis}
    else:
      accel_axis = 'ABS_RX'
      steer_axis = 'ABS_Z'
      self.flip_map = {'ABS_RY': accel_axis}

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


def send_thread(joystick):
  pm = messaging.PubMaster(['testJoystick'])

  rk = Ratekeeper(100, print_delay_threshold=None)

  while True:
    if rk.frame % 20 == 0:
      print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))

    joystick_msg = messaging.new_message('testJoystick')
    joystick_msg.valid = True
    joystick_msg.testJoystick.axes = [joystick.axes_values[ax] for ax in joystick.axes_order]

    pm.send('testJoystick', joystick_msg)

    rk.keep_time()


def joystick_control_thread(joystick):
  Params().put_bool('JoystickDebugMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()


def main():
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting joystick_control. This tool supports ' +
                                               'keyboard, physical joystick, or UDP gamepad input.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
  parser.add_argument('--udp', action='store_true', help='Use UDP gamepad input instead of physical joystick')
  parser.add_argument('--port', type=int, default=9999, help='UDP port to listen on (default: 9999)')
  args = parser.parse_args()

  # if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
  #   print("The car must be off before running joystick_control.")
  #   exit()

  print()
  if args.keyboard:
    print('Using keyboard control:')
    print('Gas/brake control: `W` and `S` keys')
    print('Steering control: `A` and `D` keys')
    print('Buttons')
    print('- `R`: Resets axes')
    print('- `C`: Cancel cruise control')
    joystick = Keyboard()
  elif args.udp:
    print(f'Using UDP gamepad input on port {args.port}:')
    print('- Left stick X: Steering control')
    print('- Left trigger: Brake (negative acceleration)')
    print('- Right trigger: Gas (positive acceleration)')
    print('- Send JSON data: {"left_stick_x": -1.0 to 1.0, "left_trigger": 0.0 to 1.0, "right_trigger": 0.0 to 1.0}')
    print('Waiting for UDP gamepad data...')
    joystick = UdpJoystick(port=args.port)
  else:
    print('Using physical joystick, make sure to run cereal/messaging/bridge on your device if running over the network!')
    print('If not running on a comma device, the mapping may need to be adjusted.')
    joystick = Joystick()

  joystick_control_thread(joystick)


if __name__ == '__main__':
  main()