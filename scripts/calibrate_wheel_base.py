#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Wheel base calibration via in-place rotation.

Procedure:
  1. Place a tape marker on the floor aligned with the robot's front
  2. Run this script (motor_controller must already be running)
  3. The robot will spin slowly in place
  4. Watch the robot -- press Enter when it completes exactly one
     full 360-degree rotation back to the marker
  5. The script reports the corrected wheel_base value

The math:
  v_angular = (v_right - v_left) / wheel_base
  If odometry reports MORE than 360 deg, wheel_base is too small.
  If odometry reports LESS than 360 deg, wheel_base is too large.
  new_wheel_base = old_wheel_base * (reported_degrees / 360)
"""

import math
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WheelBaseCalibrator(Node):

    def __init__(self, angular_speed, current_wheel_base):
        super().__init__('wheel_base_calibrator')

        self.angular_speed = angular_speed
        self.current_wheel_base = current_wheel_base

        self.initial_yaw = None
        self.current_yaw = None
        self.total_angle = 0.0
        self.last_yaw = None
        self.running = False
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.current_yaw = yaw

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.last_yaw = yaw
            self.get_logger().info(f'Initial yaw: {math.degrees(yaw):.1f} deg')
            return

        if self.running:
            # Accumulate total angle (handles wraparound)
            delta = yaw - self.last_yaw
            if delta > math.pi:
                delta -= 2.0 * math.pi
            elif delta < -math.pi:
                delta += 2.0 * math.pi
            self.total_angle += delta

        self.last_yaw = yaw

    def spin_robot(self):
        """Publish rotation command."""
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_pub.publish(twist)

    def report_results(self):
        """Calculate and print the corrected wheel_base."""
        measured_deg = math.degrees(abs(self.total_angle))
        actual_deg = 360.0

        if measured_deg < 10.0:
            self.get_logger().error(
                f'Only measured {measured_deg:.1f} deg -- too little rotation. '
                'Make sure motor_controller is running and the robot can spin.')
            return

        new_wheel_base = self.current_wheel_base * (measured_deg / actual_deg)
        correction_pct = ((new_wheel_base / self.current_wheel_base) - 1.0) * 100.0

        print('\n' + '=' * 50)
        print('CALIBRATION RESULTS')
        print('=' * 50)
        print(f'  Actual rotation:     {actual_deg:.1f} deg')
        print(f'  Odometry reported:   {measured_deg:.1f} deg')
        print(f'  Current wheel_base:  {self.current_wheel_base:.4f} m')
        print(f'  Corrected wheel_base: {new_wheel_base:.4f} m')
        print(f'  Correction:          {correction_pct:+.1f}%')
        print('=' * 50)
        print()
        print('To apply, update the wheel_base parameter in:')
        print('  motor_controller.py  (default value)')
        print('  or your launch file  (parameter override)')
        print()


def main():
    angular_speed = 0.5  # rad/s -- slow enough to watch
    current_wheel_base = 0.165  # current default

    # Parse optional args
    args = sys.argv[1:]
    if '--speed' in args:
        idx = args.index('--speed')
        angular_speed = float(args[idx + 1])
    if '--wheel-base' in args:
        idx = args.index('--wheel-base')
        current_wheel_base = float(args[idx + 1])

    rclpy.init()
    node = WheelBaseCalibrator(angular_speed, current_wheel_base)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for first odom message
    while node.initial_yaw is None and rclpy.ok():
        pass

    print()
    print('=' * 50)
    print('WHEEL BASE CALIBRATION')
    print('=' * 50)
    print(f'  Current wheel_base: {current_wheel_base} m')
    print(f'  Spin speed: {angular_speed} rad/s')
    print()
    print('1. Place a tape marker aligned with the robot front')
    print('2. Press Enter to start spinning')
    print()

    input('Press Enter to START spinning...')

    node.total_angle = 0.0
    node.running = True

    # Publish spin commands on a timer
    timer = node.create_timer(0.05, node.spin_robot)

    print()
    print('Robot is spinning. Watch for exactly ONE full rotation (360 deg).')
    print(f'  (Live: odometry angle will print below)')
    print()

    # Print live angle updates
    report_timer = node.create_timer(
        0.25, lambda: print(
            f'\r  Odometry angle: {math.degrees(abs(node.total_angle)):7.1f} deg',
            end='', flush=True))

    input('Press Enter when robot completes exactly 360 degrees...')

    # Stop
    timer.cancel()
    report_timer.cancel()
    node.running = False
    node.stop_robot()

    # Give a moment for the stop command to take effect
    import time
    time.sleep(0.2)
    node.stop_robot()

    node.report_results()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
