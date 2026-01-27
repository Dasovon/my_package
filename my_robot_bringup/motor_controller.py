#!/usr/bin/env python3
"""
Differential Drive Motor Controller for L298N
Subscribes to /cmd_vel and controls motors via GPIO

Safety features:
- Velocity ramping (soft start/stop)
- Maximum speed limits
- Minimum duty cycle for reliable starting
- Emergency stop on node shutdown
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.179)  # Distance between wheels (m)
        self.declare_parameter('max_speed', 0.5)     # Max linear velocity (m/s)
        self.declare_parameter('min_duty_cycle', 60) # Minimum PWM for starting
        self.declare_parameter('max_duty_cycle', 100)
        self.declare_parameter('pwm_frequency', 1000)
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_duty = self.get_parameter('min_duty_cycle').value
        self.max_duty = self.get_parameter('max_duty_cycle').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        
        # GPIO pins (BCM numbering)
        self.ENABLE_A = 17
        self.IN1 = 27
        self.IN2 = 22
        self.ENABLE_B = 13
        self.IN3 = 19
        self.IN4 = 26
        
        # Current motor speeds (duty cycle percentage)
        self.current_speed_a = 0.0
        self.current_speed_b = 0.0
        
        # Target motor speeds (from cmd_vel)
        self.target_speed_a = 0.0
        self.target_speed_b = 0.0
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for motor control loop (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Watchdog timer - stop motors if no cmd_vel for 1 second
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        
        self.get_logger().info('Motor controller initialized')
        self.get_logger().info(f'Min duty cycle: {self.min_duty}%')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
    
    def setup_gpio(self):
        """Initialize GPIO pins and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Set up all pins
        GPIO.setup(self.ENABLE_A, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENABLE_B, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        
        # Initialize all LOW
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        
        # Create PWM objects
        self.pwm_a = GPIO.PWM(self.ENABLE_A, self.pwm_freq)
        self.pwm_b = GPIO.PWM(self.ENABLE_B, self.pwm_freq)
        
        # Start at 0% duty cycle
        self.pwm_a.start(0)
        self.pwm_b.start(0)
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to differential drive motor speeds
        
        msg.linear.x: Forward/backward velocity (m/s)
        msg.angular.z: Rotation velocity (rad/s)
        """
        self.last_cmd_time = self.get_clock().now()
        
        linear = msg.linear.x   # Forward speed
        angular = msg.angular.z # Rotation speed
        
        # Differential drive kinematics
        # Left wheel velocity = linear - (angular * wheel_base / 2)
        # Right wheel velocity = linear + (angular * wheel_base / 2)
        
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        
        # Convert m/s to duty cycle percentage
        # Map: 0 m/s → 0%, max_speed → max_duty
        self.target_speed_a = self.velocity_to_duty(v_left)
        self.target_speed_b = self.velocity_to_duty(v_right)
    
    def velocity_to_duty(self, velocity):
        """
        Convert velocity (m/s) to PWM duty cycle (%)
        Maps velocity range to usable duty cycle range (min_duty to max_duty)
        """
        # Clamp velocity to max speed
        velocity = max(-self.max_speed, min(self.max_speed, velocity))
        
        # Dead zone - treat very small velocities as zero
        if abs(velocity) < 0.01:
            return 0.0
        
        # Map velocity to duty cycle range: [min_duty, max_duty]
        # At any non-zero velocity, use at least min_duty
        # Scale from min_duty to max_duty based on velocity
        
        duty_range = self.max_duty - self.min_duty
        velocity_fraction = abs(velocity) / self.max_speed
        
        duty = self.min_duty + (velocity_fraction * duty_range)
        
        # Preserve sign for direction
        return duty if velocity >= 0 else -duty
    
    def control_loop(self):
        """
        Motor control loop - implements soft ramping
        Runs at 50 Hz
        """
        # Ramp rate: max change per control loop iteration
        # 50 duty cycle units per second / 50 Hz = 1.0 per iteration
        max_change = 4.0
        
        # Ramp motor A toward target
        if abs(self.target_speed_a - self.current_speed_a) < max_change:
            self.current_speed_a = self.target_speed_a
        elif self.target_speed_a > self.current_speed_a:
            self.current_speed_a += max_change
        else:
            self.current_speed_a -= max_change
        
        # Ramp motor B toward target
        if abs(self.target_speed_b - self.current_speed_b) < max_change:
            self.current_speed_b = self.target_speed_b
        elif self.target_speed_b > self.current_speed_b:
            self.current_speed_b += max_change
        else:
            self.current_speed_b -= max_change
        
        # Apply to motors
        self.set_motor_a(self.current_speed_a)
        self.set_motor_b(self.current_speed_b)
    
    def set_motor_a(self, duty):
        """Set motor A speed and direction"""
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
        """Set motor B speed and direction - INVERTED to match motor A"""
        if duty > 0:
            GPIO.output(self.IN3, GPIO.HIGH)   # SWAPPED
            GPIO.output(self.IN4, GPIO.LOW)  # SWAPPED
            self.pwm_b.ChangeDutyCycle(abs(duty))
        elif duty < 0:
            GPIO.output(self.IN3, GPIO.LOW)  # SWAPPED
            GPIO.output(self.IN4, GPIO.HIGH)   # SWAPPED
            self.pwm_b.ChangeDutyCycle(abs(duty))
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)
            self.pwm_b.ChangeDutyCycle(0)
    
    def watchdog_check(self):
        """Stop motors if no cmd_vel received for 1 second"""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > 1.0:
            # Timeout - stop motors
            self.target_speed_a = 0.0
            self.target_speed_b = 0.0
    
    def stop_motors(self):
        """Emergency stop - called on shutdown"""
        self.get_logger().info('Stopping motors...')
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
    
    def cleanup(self):
        """Clean up GPIO on shutdown"""
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
