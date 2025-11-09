#!/usr/bin/env python3
"""
Steering Response Test Script for Teledriving Tuning

Tests 3 scenarios to characterize steering system:
1. Square wave: -1 to +1 torque (full range)
2. Square wave: 0 to +1 torque (one-sided)
3. Sine wave: -1 to +1 torque (smooth)

This script publishes to 'testJoystick' topic and lets joystickd handle the actual control.
It subscribes to carState, carOutput, and carControl to log the results.

Logs:
- Commanded joystick input (lateral axis)
- Commanded torque (from carControl)
- Actual steering wheel angle (from carState)
- Measured steering torque (from carState)
- Output torque (from carOutput)
- Timestamp

Output: CSV file with all data for analysis
"""

import time
import math
import csv
import os
from datetime import datetime
import numpy as np

from cereal import messaging, car, log
from openpilot.common.realtime import DT_CTRL, Ratekeeper
from openpilot.common.params import Params


class SteeringTest:
    def __init__(self):
        self.params = Params()


        # Setup messaging - subscribe to data, publish to testJoystick
        self.sm = messaging.SubMaster(['carState', 'carOutput', 'carControl', 'selfdriveState'],
                                      frequency=1. / DT_CTRL)
        self.pm = messaging.PubMaster(['testJoystick'])

        # Data storage
        self.test_data = []

        # Test parameters
        self.test_duration = 10.0  # seconds per test
        self.rate = 100  # Hz

    def wait_for_joystickd(self):
        """Wait for joystickd to be ready"""
        print("\nWaiting for joystickd to be active...")
        timeout = 60.0
        start = time.monotonic()

        while time.monotonic() - start < timeout:
            self.sm.update(0)
            if self.sm.valid['selfdriveState'] and self.sm['selfdriveState'].engageable:
                print("✓ joystickd is ready!")
                return True
            time.sleep(0.1)

        print("✗ Timeout waiting for joystickd. Is it running?")
        return False

    def wait_for_user(self, message):
        """Wait for user to press Enter"""
        input(f"\n{message}\nPress ENTER to continue...")

    def log_data_point(self, timestamp, test_name, commanded_input):
        """Log a single data point"""
        CS = self.sm['carState']
        CO = self.sm['carOutput']
        CC = self.sm['carControl']

        # Get actual values from car
        actual_angle = CS.steeringAngleDeg
        actual_torque = CS.steeringTorque
        output_torque = CO.actuatorsOutput.torque if self.sm.valid['carOutput'] else 0.0
        commanded_torque = CC.actuators.torque if self.sm.valid['carControl'] else 0.0

        data_point = {
            'timestamp': timestamp,
            'test_name': test_name,
            'commanded_input': commanded_input,
            'commanded_torque': commanded_torque,
            'actual_steering_angle_deg': actual_angle,
            'actual_steering_torque': actual_torque,
            'output_torque': output_torque,
            'vehicle_speed_mps': CS.vEgo,
            'lat_active': CC.latActive if self.sm.valid['carControl'] else False,
            'enabled': CC.enabled if self.sm.valid['carControl'] else False,
        }

        self.test_data.append(data_point)

    def send_joystick_command(self, lateral_input, longitudinal_input=0.0):
        """Send joystick command via testJoystick topic"""
        msg = messaging.new_message('testJoystick')
        msg.valid = True
        msg.testJoystick.axes = [float(longitudinal_input), float(lateral_input)]
        self.pm.send('testJoystick', msg)

    def test_square_wave_full(self):
        """Test 1: Square wave from -1 to +1"""
        print("\n" + "="*60)
        print("TEST 1: Square Wave -1 to +1 (Full Range)")
        print("="*60)
        print("This will alternate between full left and full right torque")
        print(f"Duration: {self.test_duration} seconds")

        self.wait_for_user("Center the steering wheel and ensure car is ready")

        if not self.wait_for_joystickd():
            return False

        rk = Ratekeeper(self.rate, print_delay_threshold=None)
        start_time = time.monotonic()
        test_name = "square_wave_full"

        # Square wave: 2 second period (1 sec at -1, 1 sec at +1)
        square_period = 2.0

        print("Starting test...")
        while time.monotonic() - start_time < self.test_duration:
            self.sm.update(0)

            elapsed = time.monotonic() - start_time
            phase = (elapsed % square_period) / square_period

            # Square wave: -1 for first half, +1 for second half
            commanded_input = -1.0 if phase < 0.5 else 1.0

            self.send_joystick_command(commanded_input)
            self.log_data_point(elapsed, test_name, commanded_input)

            if int(elapsed * 10) % 10 == 0 and int(elapsed * 10) != int((elapsed - 0.1) * 10):
                angle = self.sm['carState'].steeringAngleDeg if self.sm.valid['carState'] else 0.0
                print(f"  {elapsed:.1f}s: input={commanded_input:+.2f}, angle={angle:.1f}°")

            rk.keep_time()

        # Return to neutral
        self.send_joystick_command(0.0)
        print("Test 1 complete!\n")
        return True

    def test_square_wave_positive(self):
        """Test 2: Square wave from 0 to +1"""
        print("\n" + "="*60)
        print("TEST 2: Square Wave 0 to +1 (One-Sided)")
        print("="*60)
        print("This will alternate between neutral and full right torque")
        print(f"Duration: {self.test_duration} seconds")

        self.wait_for_user("Center the steering wheel and ensure car is ready")

        if not self.wait_for_joystickd():
            return False

        rk = Ratekeeper(self.rate, print_delay_threshold=None)
        start_time = time.monotonic()
        test_name = "square_wave_positive"

        square_period = 2.0

        print("Starting test...")
        while time.monotonic() - start_time < self.test_duration:
            self.sm.update(0)

            elapsed = time.monotonic() - start_time
            phase = (elapsed % square_period) / square_period

            # Square wave: 0 for first half, +1 for second half
            commanded_input = 0.0 if phase < 0.5 else 1.0

            self.send_joystick_command(commanded_input)
            self.log_data_point(elapsed, test_name, commanded_input)

            if int(elapsed * 10) % 10 == 0 and int(elapsed * 10) != int((elapsed - 0.1) * 10):
                angle = self.sm['carState'].steeringAngleDeg if self.sm.valid['carState'] else 0.0
                print(f"  {elapsed:.1f}s: input={commanded_input:+.2f}, angle={angle:.1f}°")

            rk.keep_time()

        # Return to neutral
        self.send_joystick_command(0.0)
        print("Test 2 complete!\n")
        return True

    def test_sine_wave(self):
        """Test 3: Sine wave from -1 to +1"""
        print("\n" + "="*60)
        print("TEST 3: Sine Wave -1 to +1 (Smooth)")
        print("="*60)
        print("This will smoothly vary torque in a sine pattern")
        print(f"Duration: {self.test_duration} seconds")

        self.wait_for_user("Center the steering wheel and ensure car is ready")

        if not self.wait_for_joystickd():
            return False

        rk = Ratekeeper(self.rate, print_delay_threshold=None)
        start_time = time.monotonic()
        test_name = "sine_wave"

        # Sine wave: 4 second period (slower for smooth tracking)
        sine_period = 4.0

        print("Starting test...")
        while time.monotonic() - start_time < self.test_duration:
            self.sm.update(0)

            elapsed = time.monotonic() - start_time

            # Sine wave: varies from -1 to +1
            commanded_input = math.sin(2 * math.pi * elapsed / sine_period)

            self.send_joystick_command(commanded_input)
            self.log_data_point(elapsed, test_name, commanded_input)

            if int(elapsed * 10) % 10 == 0 and int(elapsed * 10) != int((elapsed - 0.1) * 10):
                angle = self.sm['carState'].steeringAngleDeg if self.sm.valid['carState'] else 0.0
                print(f"  {elapsed:.1f}s: input={commanded_input:+.2f}, angle={angle:.1f}°")

            rk.keep_time()

        # Return to neutral
        self.send_joystick_command(0.0)
        print("Test 3 complete!\n")
        return True

    def save_data(self):
        """Save all test data to CSV"""
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"/data/steering_test_{timestamp_str}.csv"

        if not self.test_data:
            print("No data to save!")
            return

        fieldnames = [
            'timestamp',
            'test_name',
            'commanded_input',
            'commanded_torque',
            'actual_steering_angle_deg',
            'actual_steering_torque',
            'output_torque',
            'vehicle_speed_mps',
            'lat_active',
            'enabled',
        ]

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.test_data)

        print(f"\nData saved to: {filename}")
        print(f"Total data points: {len(self.test_data)}")

        # Print summary statistics
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)

        for test_name in ['square_wave_full', 'square_wave_positive', 'sine_wave']:
            test_points = [d for d in self.test_data if d['test_name'] == test_name]
            if test_points:
                angles = [d['actual_steering_angle_deg'] for d in test_points]
                torques = [d['actual_steering_torque'] for d in test_points]

                print(f"\n{test_name}:")
                print(f"  Data points: {len(test_points)}")
                print(f"  Angle range: {min(angles):.1f}° to {max(angles):.1f}°")
                print(f"  Torque range: {min(torques):.1f} to {max(torques):.1f}")

                # Check if control was active
                active_points = sum(1 for d in test_points if d['lat_active'])
                print(f"  Control active: {active_points}/{len(test_points)} samples ({100*active_points/len(test_points):.1f}%)")

    def run_all_tests(self):
        """Run all three tests in sequence"""
        print("\n" + "="*60)
        print("STEERING RESPONSE TEST SUITE")
        print("="*60)
        print(f"Test duration: {self.test_duration} seconds per test")
        print(f"Sample rate: {self.rate} Hz")
        print("\nIMPORTANT:")
        print("- This script publishes to 'testJoystick' topic")
        print("- Make sure joystickd is RUNNING")
        print("- Press RES_ACCEL button to enable controls_allowed")
        print("- Ensure vehicle is in a safe, open area")
        print("- Keep hands near steering wheel to take over if needed")
        print("- Press Ctrl+C to abort at any time")

        self.wait_for_user("Ready to start tests?")

        try:
            # Run all three tests
            success = True
            success = self.test_square_wave_full() and success
            if success:
                success = self.test_square_wave_positive() and success
            if success:
                success = self.test_sine_wave() and success

            if not success:
                print("\n✗ Tests incomplete - joystickd may not be running")

            # Save results regardless
            self.save_data()

            print("\n" + "="*60)
            print("ALL TESTS COMPLETE!" if success else "TESTS INCOMPLETE")
            print("="*60)
            print("You can now analyze the CSV file to:")
            print("- Plot commanded torque vs actual angle")
            print("- Calculate system response time/lag")
            print("- Identify friction and dead zones")
            print("- Tune kP gain for angle control")

        except KeyboardInterrupt:
            print("\n\nTest aborted by user!")
            # Still save whatever data we collected
            if self.test_data:
                self.save_data()
        except Exception as e:
            print(f"\nError during testing: {e}")
            import traceback
            traceback.print_exc()
            # Still save whatever data we collected
            if self.test_data:
                self.save_data()
        finally:
            # Make sure we send zero input at the end
            self.send_joystick_command(0.0)


def main():
    print("Steering Response Test Script (via testJoystick)")
    print("="*60)
    print("\nPRE-REQUISITES:")
    print("1. joystickd must be RUNNING")
    print("2. Press RES_ACCEL button to enable controls_allowed")
    print("3. Vehicle in safe, open area")
    print("4. Engine ON for power steering")

    input("\nPress ENTER when ready...")

    test = SteeringTest()
    test.run_all_tests()


if __name__ == "__main__":
    main()
