#!/usr/bin/env python3
"""Simple joystick test script - two basic tests"""

import time
from cereal import messaging
from openpilot.common.realtime import Ratekeeper

# Test parameters
ACCEL_HOLD_TIME = 8.0    # seconds to hold forward
STEER_HOLD_TIME = 2.0    # seconds to hold each direction


def wait_for_joystickd(sm, pm):
    """Wait for joystickd to be ready and engageable"""
    print("\nWaiting for joystickd to be active...")
    print("Make sure:")
    print("  1. joystickd is running")
    print("  2. Press cruise button (SET/RES) to enable")

    # Send heartbeat while waiting
    timeout = 60.0
    start = time.monotonic()
    rk = Ratekeeper(10)  # 10Hz while waiting

    while time.monotonic() - start < timeout:
        sm.update(0)

        # Send joystick heartbeat
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [0.0, 0.0]
        joy_msg.testJoystick.loggingEnabled = False
        pm.send('testJoystick', joy_msg)

        if sm.valid['selfdriveState'] and sm['selfdriveState'].engageable:
            print("✓ joystickd is ready!")
            return True

        if int(time.monotonic() - start) % 5 == 0:
            print(f"  Waiting... ({int(time.monotonic() - start)}s)")

        rk.keep_time()

    print("✗ Timeout waiting for joystickd!")
    return False


def accel_test(sm, pm):
    """Acceleration test: joystick full forward, then full back"""
    print("\n=== ACCELERATION TEST ===")

    rk = Ratekeeper(100)
    start = time.time()

    # Enable logging 1 second before
    print("Enabling logging...")
    joy_msg = messaging.new_message('testJoystick')
    joy_msg.testJoystick.axes = [0.0, 0.0]
    joy_msg.valid = True
    joy_msg.testJoystick.loggingEnabled = True
    pm.send('testJoystick', joy_msg)

    while time.time() - start < 1.0:
        sm.update(0)
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [0.0, 0.0]
        pm.send('testJoystick', joy_msg)
        rk.keep_time()

    print(f"Sending full forward for {ACCEL_HOLD_TIME} seconds...")
    while time.time() - start < ACCEL_HOLD_TIME:
        sm.update(0)
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [1.0, 0.0]  # Full forward
        pm.send('testJoystick', joy_msg)

        if (time.time() - start) % 1.0 < 0.01:
            print(f"  Forward: {time.time()-start:.1f}s, speed={sm['carState'].vEgo*3.6:.1f} km/h")
        rk.keep_time()

    print("Sending full back until stopped...")

    # Full back for 10 seconds
    while time.time() - start < ACCEL_HOLD_TIME + 10.0:
        sm.update(0)
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [-1.0, 0.0]  # Full back
        pm.send('testJoystick', joy_msg)

        if (time.time() - start) % 1.0 < 0.01:
            print(f"  Braking: speed={sm['carState'].vEgo*3.6:.1f} km/h")
        rk.keep_time()

    print("Stopped!")
    print("Disabling logging...")
    joy_msg = messaging.new_message('testJoystick')
    joy_msg.valid = True
    joy_msg.testJoystick.axes = [0.0, 0.0]
    joy_msg.testJoystick.loggingEnabled = False
    pm.send('testJoystick', joy_msg)
    time.sleep(1)


def steer_test(sm, pm):
    """Steering test: max right, then max left"""
    print("\n=== STEERING TEST ===")

    rk = Ratekeeper(100)
    start = time.time()

    # Enable logging 1 second before
    print("Enabling logging...")
    joy_msg = messaging.new_message('testJoystick')
    joy_msg.valid = True
    joy_msg.testJoystick.axes = [0.0, 0.0]
    joy_msg.testJoystick.loggingEnabled = True
    pm.send('testJoystick', joy_msg)

    while time.time() - start < 1.0:
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [0.0, 0.0]
        pm.send('testJoystick', joy_msg)
        sm.update(0)
        rk.keep_time()

    # Get to speed
    print("Accelerating to 10 km/h...")
    while sm['carState'].vEgo * 3.6 < 20.0:
        sm.update(0)
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [0.4, 0.0]  # Moderate acceleration
        joy_msg.testJoystick.loggingEnabled = True
        pm.send('testJoystick', joy_msg)
        rk.keep_time()

    print("Max right steering...")
    start = time.time()
    while time.time() - start < STEER_HOLD_TIME:
        sm.update(0)
        speed_error = 20 - sm['carState'].vEgo * 3.6
        accel = max(-1.0, min(1.0, speed_error * 0.3))

        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [accel, 1.0]  # Max right
        pm.send('testJoystick', joy_msg)

        if (time.time() - start) % 0.5 < 0.01:
            print(f"  Right: {time.time()-start:.1f}s, angle={sm['carState'].steeringAngleDeg:.1f}°")
        rk.keep_time()

    print("Max left steering...")
    start = time.time()
    while time.time() - start < STEER_HOLD_TIME:
        sm.update(0)
        speed_error = 20.0 - sm['carState'].vEgo * 3.6
        accel = max(-1.0, min(1.0, speed_error * 0.3))

        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [accel, -1.0]  # Max left
        pm.send('testJoystick', joy_msg)

        if (time.time() - start) % 0.5 < 0.01:
            print(f"  Left: {time.time()-start:.1f}s, angle={sm['carState'].steeringAngleDeg:.1f}°")
        rk.keep_time()
    # keep speeed and drive forward for 4 seconds
    print("Driving forward for 4 seconds...")
    start = time.time()
    while time.time() - start < 4.0:
        sm.update(0)
        speed_error = 20.0 - sm['carState'].vEgo * 3.6
        accel = max(-1.0, min(1.0, speed_error * 0.3))

        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [accel, 0.0]  # straight
        pm.send('testJoystick', joy_msg)

        if (time.time() - start) % 0.5 < 0.01:
            print(f"  Straight: {time.time()-start:.1f}s, angle={sm['carState'].steeringAngleDeg:.1f}°")
        rk.keep_time()

    # Disable logging
    print("Disabling logging...")
    joy_msg = messaging.new_message('testJoystick')
    joy_msg.valid = True
    joy_msg.testJoystick.axes = [0.0, 0.0]
    joy_msg.testJoystick.loggingEnabled = False
    pm.send('testJoystick', joy_msg)
    print("Done!")


def main():
    print("Simple Joystick Test Script")
    print("="*60)
    print("\nPRE-REQUISITES:")
    print("1. joystickd must be RUNNING")
    print("2. Press cruise button (SET/RES) to enable")
    print("3. Vehicle in safe, open area")

    sm = messaging.SubMaster(['carState', 'selfdriveState'])
    pm = messaging.PubMaster(['testJoystick'])

    # Wait for joystickd to be ready
    if not wait_for_joystickd(sm, pm):
        print("Cannot proceed without joystickd")
        return

    print("\n" + "="*60)
    print("Choose test:")
    print("1. Acceleration test")
    print("2. Steering test")
    print("0. Exit")

    try:
          choice = input("\nEnter test number: ")
          if choice == "1":
              accel_test(sm, pm)
          elif choice == "2":
              steer_test(sm, pm)
          else:
              print("Invalid choice")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Send zero command
        joy_msg = messaging.new_message('testJoystick')
        joy_msg.valid = True
        joy_msg.testJoystick.axes = [0.0, 0.0]
        joy_msg.testJoystick.loggingEnabled = False
        pm.send('testJoystick', joy_msg)


if __name__ == "__main__":
    main()
