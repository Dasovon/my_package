#!/usr/bin/env python3
"""
Keyboard Teleoperation Node
Publishes /cmd_vel based on keyboard input

Controls:
    W/Up    - Forward
    S/Down  - Backward
    A/Left  - Turn left
    D/Right - Turn right
    Space   - Stop
    Q       - Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.3)   # m/s
        self.declare_parameter('angular_speed', 0.8)  # rad/s

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Current velocities
        self.linear = 0.0
        self.angular = 0.0

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info('Keyboard teleop started')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.print_instructions()

    def print_instructions(self):
        print("\n" + "="*50)
        print("KEYBOARD TELEOP CONTROLS")
        print("="*50)
        print("   W / Up    : Forward")
        print("   S / Down  : Backward")
        print("   A / Left  : Turn Left")
        print("   D / Right : Turn Right")
        print("   Space     : Stop")
        print("   Q         : Quit")
        print("="*50 + "\n")

    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
            # Handle arrow keys (escape sequences)
            if key == '\x1b':
                key += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def publish_cmd_vel(self):
        """Publish current velocity"""
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.cmd_vel_pub.publish(msg)

    def run(self):
        """Main keyboard loop"""
        try:
            while rclpy.ok():
                key = self.get_key()

                # Forward
                if key in ['w', 'W', '\x1b[A']:  # W or Up arrow
                    self.linear = self.linear_speed
                    self.angular = 0.0
                    print("Forward")
                    self.publish_cmd_vel()

                # Backward
                elif key in ['s', 'S', '\x1b[B']:  # S or Down arrow
                    self.linear = -self.linear_speed
                    self.angular = 0.0
                    print("Backward")
                    self.publish_cmd_vel()

                # Turn left
                elif key in ['a', 'A', '\x1b[D']:  # A or Left arrow
                    self.linear = 0.0
                    self.angular = self.angular_speed
                    print("Turn Left")
                    self.publish_cmd_vel()

                # Turn right
                elif key in ['d', 'D', '\x1b[C']:  # D or Right arrow
                    self.linear = 0.0
                    self.angular = -self.angular_speed
                    print("Turn Right")
                    self.publish_cmd_vel()

                # Stop
                elif key == ' ':
                    self.linear = 0.0
                    self.angular = 0.0
                    print("STOP")
                    self.publish_cmd_vel()

                # Quit
                elif key in ['q', 'Q', '\x03']:  # Q or Ctrl+C
                    print("\nQuitting...")
                    self.linear = 0.0
                    self.angular = 0.0
                    self.publish_cmd_vel()  # Send stop command
                    break

                # Ignore other keys
                else:
                    continue

        except (OSError, termios.error, ValueError) as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Send stop command on exit
            self.linear = 0.0
            self.angular = 0.0
            self.publish_cmd_vel()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
