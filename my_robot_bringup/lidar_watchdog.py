#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Lidar watchdog node.

Monitors /scan subscriber count and stops the RPLIDAR motor when nothing
needs the scan data, then restarts it when a subscriber appears.

Also automatically power cycles the RPLIDAR USB when a crash is detected,
clearing the hardware bad state that causes 80008002/80008000 errors.

Requires sudoers rule (run once on Pi):
  echo 'ryan ALL=(ALL) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/1-1.2/authorized' \
    | sudo tee /etc/sudoers.d/lidar-power-cycle

This conserves battery during development/coding sessions when SLAM and
rviz are not running.

A grace period prevents stop_motor from being called too soon after the
rplidar node starts or restarts, which can cause SDK timeouts.
"""

import subprocess
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

GRACE_PERIOD_SEC = 8.0   # seconds to wait after rplidar comes up before stopping motor
USB_DEVICE_PATH = '/sys/bus/usb/devices/1-1.2/authorized'


class LidarWatchdog(Node):
    """Stop/start RPLIDAR motor based on /scan subscriber count.
    Power cycles USB on crash to clear hardware bad state.
    """

    def __init__(self):
        super().__init__('lidar_watchdog')

        self._start_client = self.create_client(Empty, 'start_motor')
        self._stop_client = self.create_client(Empty, 'stop_motor')

        # Assume motor is running on startup (rplidar node starts it by default)
        self._motor_running = True

        # Track when the service last became available to enforce grace period
        self._service_was_ready = False
        self._service_ready_since = None

        # Cooldown to avoid power cycling repeatedly in quick succession
        self._last_power_cycle = 0.0

        # Startup timeout: power cycle if service never comes up within 20s
        self._startup_time = time.monotonic()
        self._startup_power_cycle_done = False

        self.create_timer(2.0, self._check)
        self.get_logger().info('Lidar watchdog started')

    def _check(self):
        service_ready = self._stop_client.service_is_ready()

        # Detect crash: service was available, now gone
        if self._service_was_ready and not service_ready:
            self.get_logger().warn('RPLIDAR crashed — power cycling USB')
            self._power_cycle_usb()

        # Detect recovery: service transitions from unavailable to available
        if service_ready and not self._service_was_ready:
            self._service_ready_since = self.get_clock().now()
            self.get_logger().info('RPLIDAR service available, grace period started')
            self._motor_running = True

        self._service_was_ready = service_ready

        # Startup timeout: if the service has never come up after 20s, power cycle.
        # This handles the case where rplidar crashes immediately on startup (before
        # its service ever becomes available), which the crash detector above misses.
        if self._service_ready_since is None and not self._startup_power_cycle_done:
            if time.monotonic() - self._startup_time > 20.0:
                self.get_logger().warn('RPLIDAR failed to start in 20s — power cycling USB')
                self._power_cycle_usb()
                self._startup_power_cycle_done = True

        if not service_ready:
            return

        # Don't stop the motor until the rplidar has been up for the grace period
        elapsed = (self.get_clock().now() - self._service_ready_since).nanoseconds / 1e9
        if elapsed < GRACE_PERIOD_SEC:
            return

        count = self.count_subscribers('/scan')

        if count > 0 and not self._motor_running:
            self._call(self._start_client, 'start')
            self._motor_running = True
        elif count == 0 and self._motor_running:
            self._call(self._stop_client, 'stop')
            self._motor_running = False

    def _power_cycle_usb(self):
        now = time.monotonic()
        if now - self._last_power_cycle < 10.0:
            self.get_logger().info('Power cycle skipped (cooldown)')
            return
        self._last_power_cycle = now

        try:
            subprocess.run(
                ['sudo', 'tee', USB_DEVICE_PATH],
                input='0', text=True, capture_output=True, check=True
            )
            time.sleep(2.0)
            subprocess.run(
                ['sudo', 'tee', USB_DEVICE_PATH],
                input='1', text=True, capture_output=True, check=True
            )
            self.get_logger().info('USB power cycle complete')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                f'USB power cycle failed: {e}\n'
                'Run this once to fix:\n'
                "  echo 'ryan ALL=(ALL) NOPASSWD: /usr/bin/tee "
                "/sys/bus/usb/devices/1-1.2/authorized' "
                '| sudo tee /etc/sudoers.d/lidar-power-cycle'
            )

    def _call(self, client, action):
        if not client.service_is_ready():
            self.get_logger().warn(f'{action}_motor service not ready, skipping')
            return
        client.call_async(Empty.Request())
        past_tense = 'started' if action == 'start' else 'stopped'
        self.get_logger().info(f'Lidar motor {past_tense}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
