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

This conserves battery during development/coding sessions when SLAM and
rviz are not running.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class LidarWatchdog(Node):
    """Stop/start RPLIDAR motor based on /scan subscriber count."""

    def __init__(self):
        super().__init__('lidar_watchdog')

        self._start_client = self.create_client(Empty, 'start_motor')
        self._stop_client = self.create_client(Empty, 'stop_motor')

        # Assume motor is running on startup (rplidar node starts it by default)
        self._motor_running = True

        self.create_timer(2.0, self._check)
        self.get_logger().info('Lidar watchdog started')

    def _check(self):
        count = self.count_subscribers('/scan')

        if count > 0 and not self._motor_running:
            self._call(self._start_client, 'start')
            self._motor_running = True
        elif count == 0 and self._motor_running:
            self._call(self._stop_client, 'stop')
            self._motor_running = False

    def _call(self, client, action):
        if not client.service_is_ready():
            self.get_logger().warn(f'{action}_motor service not ready, skipping')
            return
        client.call_async(Empty.Request())
        self.get_logger().info(f'Lidar motor {action}ped')


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
