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

Also sends the RPLIDAR firmware reset command (0xA5 0x40) over serial when a
crash is detected. This resets the RPLIDAR MCU (motor restarts, state cleared),
fixing the 80008002 / "Failed to set scan mode" errors that occur when the motor
is stopped and the node crashes. No sudo required — uses the serial port directly.

This conserves battery during development/coding sessions when SLAM and
rviz are not running.

A grace period prevents stop_motor from being called too soon after the
rplidar node starts or restarts, which can cause SDK timeouts.
"""

import serial
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

GRACE_PERIOD_SEC = 8.0   # seconds to wait after rplidar comes up before stopping motor
LIDAR_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
RPLIDAR_RESET_CMD = bytes([0xA5, 0x40])


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

        # Cooldown to avoid resetting repeatedly in quick succession
        self._last_reset = 0.0

        # Startup timeout: reset firmware if service never comes up within 20s
        self._startup_time = time.monotonic()
        self._startup_reset_done = False

        self.create_timer(2.0, self._check)
        self.get_logger().info('Lidar watchdog started')

    def _check(self):
        service_ready = self._stop_client.service_is_ready()

        # Detect crash: service was available, now gone
        if self._service_was_ready and not service_ready:
            self.get_logger().warn('RPLIDAR crashed — sending firmware reset')
            self._reset_lidar_firmware()

        # Detect recovery: service transitions from unavailable to available
        if service_ready and not self._service_was_ready:
            self._service_ready_since = self.get_clock().now()
            self.get_logger().info('RPLIDAR service available, grace period started')
            self._motor_running = True

        self._service_was_ready = service_ready

        # Startup timeout: if the service has never come up after 20s, reset firmware.
        # This handles the case where rplidar crashes immediately on startup (before
        # its service ever becomes available), which the crash detector above misses.
        if self._service_ready_since is None and not self._startup_reset_done:
            if time.monotonic() - self._startup_time > 20.0:
                self.get_logger().warn('RPLIDAR failed to start in 20s — sending firmware reset')
                self._reset_lidar_firmware()
                self._startup_reset_done = True

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

    def _reset_lidar_firmware(self):
        """Send the RPLIDAR firmware reset command over serial.

        Resets the RPLIDAR MCU directly — motor restarts and state clears.
        This is the software equivalent of a power cycle for the RPLIDAR firmware.
        No sudo required; uses the dialout-accessible serial port.
        """
        now = time.monotonic()
        if now - self._last_reset < 10.0:
            self.get_logger().info('Firmware reset skipped (cooldown)')
            return
        self._last_reset = now

        try:
            with serial.Serial(LIDAR_PORT, 115200, timeout=1) as port:
                port.reset_input_buffer()
                port.reset_output_buffer()
                port.write(RPLIDAR_RESET_CMD)
                self.get_logger().info('RPLIDAR firmware reset command sent — waiting 2s')
                time.sleep(2.0)
        except serial.SerialException as e:
            self.get_logger().error(f'RPLIDAR firmware reset failed: {e}')

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
