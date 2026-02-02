#!/usr/bin/env python3
"""
Differential Drive Motor Controller with Hall Effect Encoders
- DG01D-E motors (1:48 gear ratio, 576 ticks/rev)
- L298N dual H-bridge driver
- Open-loop velocity control (encoders for odometry only)
- Odometry publishing (/odom + TF)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import math
from tf2_ros import TransformBroadcaster


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.165)
        self.declare_parameter('wheel_diameter', 0.06475)
        self.declare_parameter('encoder_ticks_per_rev', 576)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('min_duty_cycle', 80)  # Increased from 60
        self.declare_parameter('max_duty_cycle', 100)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
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
        
        # Calculate wheel geometry
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.meters_per_tick = self.wheel_circumference / self.ticks_per_rev
        
        # Motor control GPIO pins
        self.ENABLE_A = 17
        self.IN1 = 27
        self.IN2 = 22
        self.ENABLE_B = 13
        self.IN3 = 19
        self.IN4 = 26
        
        # Encoder GPIO pins
        self.ENC_A_PIN_A = 23
        self.ENC_A_PIN_B = 24
        self.ENC_B_PIN_A = 25
        self.ENC_B_PIN_B = 5
        
        # Encoder state
        self.ticks_left = 0
        self.ticks_right = 0
        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.prev_enc_a_state = 0
        self.prev_enc_b_state = 0
        
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
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.encoder_timer = self.create_timer(0.01, self.read_encoders)  # 100 Hz
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        
        self.get_logger().info('Motor controller initialized')
        self.get_logger().info(f'DG01D-E: 1:48 ratio, 576 ticks/rev')
        self.get_logger().info(f'Wheel base: {self.wheel_base:.3f} m')
        self.get_logger().info(f'Min duty: {self.min_duty}%')
    
    def setup_gpio(self):
        """Initialize GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
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
        
        self.prev_enc_a_state = GPIO.input(self.ENC_A_PIN_A)
        self.prev_enc_b_state = GPIO.input(self.ENC_B_PIN_A)
    
    def read_encoders(self):
        """Quadrature encoder reading"""
        # Motor A (left)
        curr_a_a = GPIO.input(self.ENC_A_PIN_A)
        curr_a_b = GPIO.input(self.ENC_A_PIN_B)
        
        if curr_a_a != self.prev_enc_a_state:
            if curr_a_a == curr_a_b:
                self.ticks_left += 1
            else:
                self.ticks_left -= 1
            self.prev_enc_a_state = curr_a_a
        
        # Motor B (right)
        curr_b_a = GPIO.input(self.ENC_B_PIN_A)
        curr_b_b = GPIO.input(self.ENC_B_PIN_B)
        
        if curr_b_a != self.prev_enc_b_state:
            if curr_b_a == curr_b_b:
                self.ticks_right += 1
            else:
                self.ticks_right -= 1
            self.prev_enc_b_state = curr_b_a
    
    def cmd_vel_callback(self, msg):
        """Convert Twist to wheel velocities"""
        self.last_cmd_time = self.get_clock().now()
        
        linear = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        
        # Differential drive kinematics
        self.target_vel_left = linear - (angular * self.wheel_base / 2.0)
        self.target_vel_right = linear + (angular * self.wheel_base / 2.0)
    
    def control_loop(self):
        """Main control loop - open-loop with odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate measured velocities from encoders
        delta_left = self.ticks_left - self.last_ticks_left
        delta_right = self.ticks_right - self.last_ticks_right
        
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
        """Map velocity to duty cycle"""
        if abs(velocity) < 0.01:
            return 0.0
        
        # Map velocity range to duty range
        velocity_fraction = abs(velocity) / self.max_speed
        duty = self.min_duty + (velocity_fraction * (self.max_duty - self.min_duty))
        
        return duty if velocity >= 0 else -duty
    
    def update_odometry(self, dt):
        """Calculate odometry from encoder velocities"""
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
    
    def publish_odometry(self, v_linear, v_angular):
        """Publish odometry and TF"""
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
        
        self.odom_pub.publish(odom)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    def set_motor_a(self, duty):
        """Motor A (left) - INVERTED"""
        # Debug logging (throttled to 1 Hz)
        if abs(duty) > 0:
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
        """Motor B (right) - INVERTED"""
        # Debug logging (throttled to 1 Hz)
        if abs(duty) > 0:
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
        """Watchdog safety timeout"""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > 1.0:
            self.target_vel_left = 0.0
            self.target_vel_right = 0.0
    
    def stop_motors(self):
        """Emergency stop"""
        self.get_logger().info('Stopping motors...')
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
    
    def cleanup(self):
        """Shutdown cleanup"""
        self.stop_motors()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        self.get_logger().info('GPIO cleanup complete')


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