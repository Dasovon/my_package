#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Differential drive motor controller with hall effect encoders.

- DG01D-E motors (1:48 gear ratio, 576 ticks/rev)
- L298N dual H-bridge driver
- Open-loop velocity control (encoders for odometry only)
- Odometry publishing (/odom + TF)
"""

import math
import os

import rclpy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MotorController(Node):
    """Control motors, encoders, and odometry publishing."""

    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.236)
        self.declare_parameter('wheel_diameter', 0.065)
        self.declare_parameter('encoder_ticks_per_rev', 288)  # 3 magnets * 2 edges * 48:1 gear
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('min_duty_cycle', 80)  # Increased from 60
        self.declare_parameter('max_duty_cycle', 100)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_tick_delta', 1200)
        self.declare_parameter('motor_enable_a_pin', 17)
        self.declare_parameter('motor_in1_pin', 27)
        self.declare_parameter('motor_in2_pin', 22)
        self.declare_parameter('motor_enable_b_pin', 13)
        self.declare_parameter('motor_in3_pin', 19)
        self.declare_parameter('motor_in4_pin', 26)
        self.declare_parameter('encoder_left_a_pin', 23)
        self.declare_parameter('encoder_left_b_pin', 24)
        self.declare_parameter('encoder_right_a_pin', 25)
        self.declare_parameter('encoder_right_b_pin', 5)
        self.declare_parameter('encoder_left_inverted', True)
        self.declare_parameter('encoder_right_inverted', False)
        self.declare_parameter('encoder_bouncetime_ms', 0)
        self.declare_parameter('enable_debug_logging', False)
        self.declare_parameter('publish_odom_tf', True)

        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.min_duty = self.get_parameter('min_duty_cycle').value
        self.max_duty = self.get_parameter('max_duty_cycle').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_tick_delta = int(self.get_parameter('max_tick_delta').value)
        self.debug_logging = self.get_parameter('enable_debug_logging').value
        self.left_encoder_inverted = self.get_parameter('encoder_left_inverted').value
        self.right_encoder_inverted = self.get_parameter('encoder_right_inverted').value
        self.encoder_bouncetime_ms = int(self.get_parameter('encoder_bouncetime_ms').value)
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value

        # Calculate wheel geometry
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.meters_per_tick = self.wheel_circumference / self.ticks_per_rev

        # Motor control GPIO pins
        self.ENABLE_A = self.get_parameter('motor_enable_a_pin').value
        self.IN1 = self.get_parameter('motor_in1_pin').value
        self.IN2 = self.get_parameter('motor_in2_pin').value
        self.ENABLE_B = self.get_parameter('motor_enable_b_pin').value
        self.IN3 = self.get_parameter('motor_in3_pin').value
        self.IN4 = self.get_parameter('motor_in4_pin').value

        # Encoder GPIO pins
        self.ENC_A_PIN_A = self.get_parameter('encoder_left_a_pin').value
        self.ENC_A_PIN_B = self.get_parameter('encoder_left_b_pin').value
        self.ENC_B_PIN_A = self.get_parameter('encoder_right_a_pin').value
        self.ENC_B_PIN_B = self.get_parameter('encoder_right_b_pin').value

        # Encoder state
        self.ticks_left = 0
        self.ticks_right = 0
        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.encoder_state_left = 0
        self.encoder_state_right = 0

        # Velocity tracking
        self.measured_vel_left = 0.0
        self.measured_vel_right = 0.0
        self.target_vel_left = 0.0
        self.target_vel_right = 0.0

        # Current motor duty cycles
        self.current_duty_a = 0.0
        self.current_duty_b = 0.0

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timing
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # Initialize GPIO
        self.setup_gpio()

        # ROS 2 interfaces
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # Use BEST_EFFORT QoS to match robot_state_publisher's subscription
        joint_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', joint_state_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info('Motor controller initialized')
        self.get_logger().info(f'DG01D-E: 1:48 ratio, {self.ticks_per_rev} ticks/rev')
        self.get_logger().info(f'Wheel base: {self.wheel_base:.3f} m')
        self.get_logger().info(f'Min duty: {self.min_duty}%')

    def setup_gpio(self):
        """Initialize GPIO."""
        self.validate_gpio_access()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Motor pins
        GPIO.setup(self.ENABLE_A, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENABLE_B, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

        self.pwm_a = GPIO.PWM(self.ENABLE_A, self.pwm_freq)
        self.pwm_b = GPIO.PWM(self.ENABLE_B, self.pwm_freq)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        # Encoder pins
        GPIO.setup(self.ENC_A_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_A_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_B_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_B_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.encoder_state_left = self._read_encoder_state(self.ENC_A_PIN_A, self.ENC_A_PIN_B)
        self.encoder_state_right = self._read_encoder_state(self.ENC_B_PIN_A, self.ENC_B_PIN_B)

        self._setup_encoder_events()

    def _read_encoder_state(self, pin_a, pin_b):
        return (GPIO.input(pin_a) << 1) | GPIO.input(pin_b)

    def _setup_encoder_events(self):
        """Configure GPIO edge callbacks for encoders."""
        pins = [
            (self.ENC_A_PIN_A, self._left_encoder_callback),
            (self.ENC_A_PIN_B, self._left_encoder_callback),
            (self.ENC_B_PIN_A, self._right_encoder_callback),
            (self.ENC_B_PIN_B, self._right_encoder_callback),
        ]
        failed = 0
        for pin, cb in pins:
            if not self._add_encoder_event(pin, cb):
                failed += 1
        if failed > 0:
            self.get_logger().warn(
                f'{failed}/4 encoder interrupts failed. '
                'Odometry will be unavailable. '
                'Try: sudo pkill -f motor_controller; sudo GPIO.cleanup()'
            )

    def _add_encoder_event(self, pin, callback):
        """Register a single GPIO edge-detect callback."""
        bouncetime = self.encoder_bouncetime_ms if self.encoder_bouncetime_ms > 0 else None
        try:
            if bouncetime is None:
                GPIO.add_event_detect(pin, GPIO.BOTH, callback=callback)
            else:
                GPIO.add_event_detect(
                    pin, GPIO.BOTH, callback=callback, bouncetime=bouncetime
                )
            return True
        except RuntimeError as exc:
            self.get_logger().error(
                f'Failed to add GPIO event detect for pin {pin}: {exc}'
            )
            return False

    def _quadrature_delta(self, last_state, new_state):
        """Compute delta ticks from quadrature state transition."""
        transition = (last_state << 2) | new_state
        lookup = {
            0b0001: 1,
            0b0010: -1,
            0b0100: -1,
            0b0111: 1,
            0b1000: 1,
            0b1011: -1,
            0b1101: -1,
            0b1110: 1,
        }
        return lookup.get(transition, 0)

    def _left_encoder_callback(self, channel):
        self._update_encoder('left')

    def _right_encoder_callback(self, channel):
        self._update_encoder('right')

    def _update_encoder(self, side):
        if side == 'left':
            pin_a = self.ENC_A_PIN_A
            pin_b = self.ENC_A_PIN_B
            last_state = self.encoder_state_left
            inverted = self.left_encoder_inverted
        else:
            pin_a = self.ENC_B_PIN_A
            pin_b = self.ENC_B_PIN_B
            last_state = self.encoder_state_right
            inverted = self.right_encoder_inverted

        new_state = self._read_encoder_state(pin_a, pin_b)
        delta = self._quadrature_delta(last_state, new_state)
        if inverted:
            delta = -delta

        if side == 'left':
            self.ticks_left += delta
            self.encoder_state_left = new_state
        else:
            self.ticks_right += delta
            self.encoder_state_right = new_state

    def cmd_vel_callback(self, msg):
        """Convert Twist to wheel velocities."""
        self.last_cmd_time = self.get_clock().now()

        linear = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        # Differential drive kinematics
        self.target_vel_left = linear - (angular * self.wheel_base / 2.0)
        self.target_vel_right = linear + (angular * self.wheel_base / 2.0)

    def control_loop(self):
        """Run the main control loop with odometry."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Calculate measured velocities from encoders
        delta_left = self.ticks_left - self.last_ticks_left
        delta_right = self.ticks_right - self.last_ticks_right

        delta_left = self.clamp_tick_delta(delta_left, 'left')
        delta_right = self.clamp_tick_delta(delta_right, 'right')

        self.measured_vel_left = (delta_left * self.meters_per_tick) / dt
        self.measured_vel_right = (delta_right * self.meters_per_tick) / dt

        # Update odometry
        self.update_odometry(dt)

        # Convert target velocities to duty cycles
        duty_left = self.velocity_to_duty(self.target_vel_left)
        duty_right = self.velocity_to_duty(self.target_vel_right)

        # Fast ramping for quick response
        max_change = 10.0  # 10% per iteration (was 4%)

        # Ramp motor A
        if abs(duty_left - self.current_duty_a) < max_change:
            self.current_duty_a = duty_left
        elif duty_left > self.current_duty_a:
            self.current_duty_a += max_change
        else:
            self.current_duty_a -= max_change

        # Ramp motor B
        if abs(duty_right - self.current_duty_b) < max_change:
            self.current_duty_b = duty_right
        elif duty_right > self.current_duty_b:
            self.current_duty_b += max_change
        else:
            self.current_duty_b -= max_change

        # Apply to motors
        self.set_motor_a(self.current_duty_a)
        self.set_motor_b(self.current_duty_b)

        # Update state
        self.last_ticks_left = self.ticks_left
        self.last_ticks_right = self.ticks_right
        self.last_time = current_time

    def velocity_to_duty(self, velocity):
        """Map velocity to duty cycle."""
        if abs(velocity) < 0.01:
            return 0.0

        # Map velocity range to duty range
        velocity_fraction = min(abs(velocity) / self.max_speed, 1.0)
        duty = self.min_duty + (velocity_fraction * (self.max_duty - self.min_duty))

        return duty if velocity >= 0 else -duty

    def update_odometry(self, dt):
        """Calculate odometry from encoder-measured velocities."""
        v_left = self.measured_vel_left
        v_right = self.measured_vel_right

        v_linear = (v_left + v_right) / 2.0
        v_angular = (v_right - v_left) / self.wheel_base

        # Euler integration
        delta_theta = v_angular * dt
        delta_x = v_linear * math.cos(self.theta + delta_theta / 2.0) * dt
        delta_y = v_linear * math.sin(self.theta + delta_theta / 2.0) * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish
        self.publish_odometry(v_linear, v_angular)
        self.publish_joint_states()

    def publish_odometry(self, v_linear, v_angular):
        """Publish odometry and TF."""
        current_time = self.get_clock().now()
        quat = self.yaw_to_quaternion(self.theta)

        # TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat

        if self.publish_odom_tf:
            self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular

        # Covariance values for downstream nodes (SLAM, Nav2)
        # 6x6 covariance matrix flattened (x, y, z, roll, pitch, yaw)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,   # x variance
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,   # y variance
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,   # z variance
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,   # roll variance
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,   # pitch variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01    # yaw variance
        ]
        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,   # linear x
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,   # linear y
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,   # linear z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,   # angular roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,   # angular pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01    # angular yaw
        ]

        self.odom_pub.publish(odom)

    def publish_joint_states(self):
        """Publish wheel joint states from encoder ticks."""
        current_time = self.get_clock().now()

        # Convert encoder ticks to radians
        # radians = (ticks / ticks_per_rev) * 2 * pi
        radians_per_tick = (2.0 * math.pi) / self.ticks_per_rev
        left_wheel_pos = self.ticks_left * radians_per_tick
        right_wheel_pos = self.ticks_right * radians_per_tick

        # Calculate wheel velocities in rad/s
        left_wheel_vel = self.measured_vel_left / (self.wheel_diameter / 2.0)
        right_wheel_vel = self.measured_vel_right / (self.wheel_diameter / 2.0)

        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [left_wheel_pos, right_wheel_pos]
        js.velocity = [left_wheel_vel, right_wheel_vel]
        js.effort = []

        self.joint_state_pub.publish(js)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw to quaternion."""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def set_motor_a(self, duty):
        """Set motor A (left) duty cycle (inverted)."""
        # Debug logging (throttled to 1 Hz, configurable)
        if self.debug_logging and abs(duty) > 0:
            self.get_logger().info(
                f'Motor A: {duty:.1f}%',
                throttle_duration_sec=1.0
            )

        if duty > 0:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
            self.pwm_a.ChangeDutyCycle(abs(duty))
        elif duty < 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(abs(duty))
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(0)

    def set_motor_b(self, duty):
        """Set motor B (right) duty cycle (inverted)."""
        # Debug logging (throttled to 1 Hz, configurable)
        if self.debug_logging and abs(duty) > 0:
            self.get_logger().info(
                f'Motor B: {duty:.1f}%',
                throttle_duration_sec=1.0
            )

        if duty > 0:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
            self.pwm_b.ChangeDutyCycle(abs(duty))
        elif duty < 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
            self.pwm_b.ChangeDutyCycle(abs(duty))
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)
            self.pwm_b.ChangeDutyCycle(0)

    def watchdog_check(self):
        """Check watchdog safety timeout."""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if time_since_cmd > 1.0:
            self.target_vel_left = 0.0
            self.target_vel_right = 0.0

    def stop_motors(self):
        """Stop motors immediately."""
        self.get_logger().info('Stopping motors...')
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    def cleanup(self):
        """Clean up GPIO on shutdown."""
        self.stop_motors()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        self.get_logger().info('GPIO cleanup complete')

    def validate_gpio_access(self):
        """Ensure GPIO device access is available before setup."""
        gpio_paths = ('/dev/gpiomem', '/dev/mem')
        has_access = any(os.access(path, os.R_OK | os.W_OK) for path in gpio_paths)
        if not has_access:
            msg = (
                'GPIO access not available. Ensure you are running on a Raspberry Pi '
                'with permission to access /dev/gpiomem or /dev/mem.'
            )
            self.get_logger().error(msg)
            raise RuntimeError(msg)

    def clamp_tick_delta(self, delta, label):
        """Sanity check encoder ticks to avoid extreme spikes."""
        if abs(delta) > self.max_tick_delta:
            clamped = self.max_tick_delta if delta > 0 else -self.max_tick_delta
            self.get_logger().warn(
                f'Encoder delta for {label} wheel out of range ({delta}); '
                f'clamping to {clamped}.',
                throttle_duration_sec=1.0
            )
            return clamped
        return delta


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
