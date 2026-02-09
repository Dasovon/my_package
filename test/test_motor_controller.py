#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
# SPDX-License-Identifier: MIT
"""Unit tests for motor_controller.py."""

import math

import pytest


# ============================================================================
# Extracted pure functions for testing (matching motor_controller.py logic)
# ============================================================================

def velocity_to_duty(velocity, max_speed, min_duty, max_duty):
    """Map velocity to duty cycle."""
    if abs(velocity) < 0.01:
        return 0.0

    velocity_fraction = abs(velocity) / max_speed
    duty = min_duty + (velocity_fraction * (max_duty - min_duty))

    return duty if velocity >= 0 else -duty


def clamp_tick_delta(delta, max_tick_delta):
    """Clamp encoder tick delta to avoid extreme spikes."""
    if abs(delta) > max_tick_delta:
        return max_tick_delta if delta > 0 else -max_tick_delta
    return delta


def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion (x, y, z, w)."""
    x = 0.0
    y = 0.0
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return (x, y, z, w)


def differential_drive_kinematics(linear, angular, wheel_base):
    """Convert linear/angular velocity to wheel velocities."""
    vel_left = linear - (angular * wheel_base / 2.0)
    vel_right = linear + (angular * wheel_base / 2.0)
    return (vel_left, vel_right)


def inverse_kinematics(vel_left, vel_right, wheel_base):
    """Convert wheel velocities to linear/angular velocity."""
    v_linear = (vel_left + vel_right) / 2.0
    v_angular = (vel_right - vel_left) / wheel_base
    return (v_linear, v_angular)


def update_odometry(x, y, theta, v_linear, v_angular, dt):
    """Update odometry using Euler integration."""
    delta_theta = v_angular * dt
    delta_x = v_linear * math.cos(theta + delta_theta / 2.0) * dt
    delta_y = v_linear * math.sin(theta + delta_theta / 2.0) * dt

    new_x = x + delta_x
    new_y = y + delta_y
    new_theta = theta + delta_theta

    # Normalize theta to [-pi, pi]
    new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

    return (new_x, new_y, new_theta)


def encoder_ticks_to_distance(ticks, ticks_per_rev, wheel_diameter):
    """Convert encoder ticks to distance traveled."""
    wheel_circumference = math.pi * wheel_diameter
    meters_per_tick = wheel_circumference / ticks_per_rev
    return ticks * meters_per_tick


# ============================================================================
# Test Cases
# ============================================================================

class TestVelocityToDuty:
    """Tests for velocity_to_duty function."""

    # Default parameters matching motor_controller.py
    MAX_SPEED = 0.5
    MIN_DUTY = 80
    MAX_DUTY = 100

    def test_zero_velocity_returns_zero(self):
        """Zero velocity should return zero duty cycle."""
        result = velocity_to_duty(0.0, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result == 0.0

    def test_near_zero_velocity_returns_zero(self):
        """Very small velocity (< 0.01) should return zero."""
        result = velocity_to_duty(0.005, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result == 0.0

        result = velocity_to_duty(-0.005, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result == 0.0

    def test_max_speed_returns_max_duty(self):
        """Maximum speed should return maximum duty cycle."""
        result = velocity_to_duty(self.MAX_SPEED, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result == self.MAX_DUTY

    def test_negative_max_speed_returns_negative_max_duty(self):
        """Negative maximum speed should return negative maximum duty."""
        result = velocity_to_duty(-self.MAX_SPEED, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result == -self.MAX_DUTY

    def test_half_speed_returns_midpoint_duty(self):
        """Half speed should return duty between min and max."""
        result = velocity_to_duty(self.MAX_SPEED / 2, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        expected = self.MIN_DUTY + 0.5 * (self.MAX_DUTY - self.MIN_DUTY)
        assert result == expected

    def test_small_velocity_returns_min_duty(self):
        """Small but significant velocity should return near min duty."""
        result = velocity_to_duty(0.02, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result >= self.MIN_DUTY
        assert result < self.MAX_DUTY

    def test_negative_velocity_returns_negative_duty(self):
        """Negative velocity should return negative duty cycle."""
        result = velocity_to_duty(-0.25, self.MAX_SPEED, self.MIN_DUTY, self.MAX_DUTY)
        assert result < 0


class TestClampTickDelta:
    """Tests for clamp_tick_delta function."""

    MAX_TICK_DELTA = 1200

    def test_normal_delta_unchanged(self):
        """Delta within range should be unchanged."""
        assert clamp_tick_delta(100, self.MAX_TICK_DELTA) == 100
        assert clamp_tick_delta(-100, self.MAX_TICK_DELTA) == -100
        assert clamp_tick_delta(0, self.MAX_TICK_DELTA) == 0

    def test_max_delta_unchanged(self):
        """Delta at max should be unchanged."""
        assert clamp_tick_delta(self.MAX_TICK_DELTA, self.MAX_TICK_DELTA) == self.MAX_TICK_DELTA
        assert clamp_tick_delta(-self.MAX_TICK_DELTA, self.MAX_TICK_DELTA) == -self.MAX_TICK_DELTA

    def test_exceeding_delta_clamped(self):
        """Delta exceeding max should be clamped."""
        assert clamp_tick_delta(5000, self.MAX_TICK_DELTA) == self.MAX_TICK_DELTA
        assert clamp_tick_delta(-5000, self.MAX_TICK_DELTA) == -self.MAX_TICK_DELTA

    def test_slightly_over_clamped(self):
        """Delta slightly over max should be clamped."""
        assert (
            clamp_tick_delta(self.MAX_TICK_DELTA + 1, self.MAX_TICK_DELTA)
            == self.MAX_TICK_DELTA
        )


class TestYawToQuaternion:
    """Tests for yaw_to_quaternion function."""

    def test_zero_yaw(self):
        """Zero yaw should give identity-like quaternion."""
        x, y, z, w = yaw_to_quaternion(0.0)
        assert x == 0.0
        assert y == 0.0
        assert z == 0.0
        assert w == 1.0

    def test_90_degree_yaw(self):
        """90 degree yaw (pi/2) rotation."""
        x, y, z, w = yaw_to_quaternion(math.pi / 2)
        assert x == 0.0
        assert y == 0.0
        assert abs(z - math.sin(math.pi / 4)) < 1e-10
        assert abs(w - math.cos(math.pi / 4)) < 1e-10

    def test_180_degree_yaw(self):
        """180 degree yaw (pi) rotation."""
        x, y, z, w = yaw_to_quaternion(math.pi)
        assert x == 0.0
        assert y == 0.0
        assert abs(z - 1.0) < 1e-10
        assert abs(w) < 1e-10

    def test_negative_yaw(self):
        """Negative yaw should give negative z component."""
        x, y, z, w = yaw_to_quaternion(-math.pi / 2)
        assert z < 0

    def test_quaternion_is_normalized(self):
        """Quaternion should be unit length."""
        for yaw in [0, math.pi/4, math.pi/2, math.pi, -math.pi/4]:
            x, y, z, w = yaw_to_quaternion(yaw)
            magnitude = math.sqrt(x**2 + y**2 + z**2 + w**2)
            assert abs(magnitude - 1.0) < 1e-10


class TestDifferentialDriveKinematics:
    """Tests for differential drive kinematics."""

    WHEEL_BASE = 0.165  # meters (matching motor_controller.py)

    def test_forward_motion(self):
        """Pure forward motion: both wheels same speed."""
        vel_left, vel_right = differential_drive_kinematics(0.5, 0.0, self.WHEEL_BASE)
        assert vel_left == 0.5
        assert vel_right == 0.5

    def test_backward_motion(self):
        """Pure backward motion: both wheels negative same speed."""
        vel_left, vel_right = differential_drive_kinematics(-0.5, 0.0, self.WHEEL_BASE)
        assert vel_left == -0.5
        assert vel_right == -0.5

    def test_pure_rotation_ccw(self):
        """Counter-clockwise rotation: left wheel backward, right forward."""
        vel_left, vel_right = differential_drive_kinematics(0.0, 1.0, self.WHEEL_BASE)
        assert vel_left < 0
        assert vel_right > 0
        assert abs(vel_left) == abs(vel_right)

    def test_pure_rotation_cw(self):
        """Clockwise rotation: left wheel forward, right backward."""
        vel_left, vel_right = differential_drive_kinematics(0.0, -1.0, self.WHEEL_BASE)
        assert vel_left > 0
        assert vel_right < 0

    def test_arc_turn_left(self):
        """Arc turn left: both wheels forward, left slower."""
        vel_left, vel_right = differential_drive_kinematics(0.3, 0.5, self.WHEEL_BASE)
        assert vel_left < vel_right
        assert vel_left > 0
        assert vel_right > 0

    def test_inverse_kinematics_roundtrip(self):
        """Forward and inverse kinematics should be inverses."""
        linear_in = 0.3
        angular_in = 0.5

        vel_left, vel_right = differential_drive_kinematics(
            linear_in,
            angular_in,
            self.WHEEL_BASE,
        )
        linear_out, angular_out = inverse_kinematics(vel_left, vel_right, self.WHEEL_BASE)

        assert abs(linear_in - linear_out) < 1e-10
        assert abs(angular_in - angular_out) < 1e-10


class TestOdometryUpdate:
    """Tests for odometry update function."""

    def test_stationary(self):
        """No movement should maintain position."""
        x, y, theta = update_odometry(1.0, 2.0, 0.5, 0.0, 0.0, 0.1)
        assert x == 1.0
        assert y == 2.0
        assert abs(theta - 0.5) < 1e-10

    def test_forward_motion_at_zero_heading(self):
        """Forward motion at zero heading increases x only."""
        x, y, theta = update_odometry(0.0, 0.0, 0.0, 1.0, 0.0, 1.0)
        assert abs(x - 1.0) < 1e-10
        assert abs(y) < 1e-10
        assert abs(theta) < 1e-10

    def test_forward_motion_at_90_degrees(self):
        """Forward motion at 90 degrees increases y only."""
        x, y, theta = update_odometry(0.0, 0.0, math.pi / 2, 1.0, 0.0, 1.0)
        assert abs(x) < 1e-10
        assert abs(y - 1.0) < 1e-10

    def test_pure_rotation(self):
        """Pure rotation should only change theta."""
        x, y, theta = update_odometry(0.0, 0.0, 0.0, 0.0, 1.0, 0.1)
        assert abs(x) < 1e-10
        assert abs(y) < 1e-10
        assert abs(theta - 0.1) < 1e-10

    def test_theta_normalization(self):
        """Theta should be normalized to [-pi, pi]."""
        # Start near pi, rotate further
        x, y, theta = update_odometry(0.0, 0.0, 3.0, 0.0, 1.0, 1.0)
        assert theta >= -math.pi
        assert theta <= math.pi

    def test_arc_motion(self):
        """Combined linear and angular motion."""
        x, y, theta = update_odometry(0.0, 0.0, 0.0, 0.5, 0.5, 0.1)
        # Should move forward and to the left
        assert x > 0
        assert y > 0
        assert theta > 0


class TestEncoderConversion:
    """Tests for encoder tick to distance conversion."""

    # DG01D-E motor parameters
    TICKS_PER_REV = 576
    WHEEL_DIAMETER = 0.06475  # meters

    def test_zero_ticks(self):
        """Zero ticks should give zero distance."""
        dist = encoder_ticks_to_distance(0, self.TICKS_PER_REV, self.WHEEL_DIAMETER)
        assert dist == 0.0

    def test_one_revolution(self):
        """One revolution should equal wheel circumference."""
        dist = encoder_ticks_to_distance(
            self.TICKS_PER_REV,
            self.TICKS_PER_REV,
            self.WHEEL_DIAMETER,
        )
        expected = math.pi * self.WHEEL_DIAMETER
        assert abs(dist - expected) < 1e-10

    def test_negative_ticks(self):
        """Negative ticks should give negative distance."""
        dist = encoder_ticks_to_distance(
            -self.TICKS_PER_REV,
            self.TICKS_PER_REV,
            self.WHEEL_DIAMETER,
        )
        expected = -math.pi * self.WHEEL_DIAMETER
        assert abs(dist - expected) < 1e-10

    def test_partial_revolution(self):
        """Half revolution should give half circumference."""
        dist = encoder_ticks_to_distance(
            self.TICKS_PER_REV // 2,
            self.TICKS_PER_REV,
            self.WHEEL_DIAMETER,
        )
        expected = math.pi * self.WHEEL_DIAMETER / 2
        assert abs(dist - expected) < 1e-6


class TestIntegration:
    """Integration tests combining multiple functions."""

    WHEEL_BASE = 0.165
    TICKS_PER_REV = 576
    WHEEL_DIAMETER = 0.06475

    def test_straight_line_odometry(self):
        """Simulate straight line motion and verify odometry."""
        x, y, theta = 0.0, 0.0, 0.0

        # Simulate 10 steps of 0.1s each at 0.3 m/s
        for _ in range(10):
            x, y, theta = update_odometry(x, y, theta, 0.3, 0.0, 0.1)

        # Should have traveled ~0.3 m in x direction
        assert abs(x - 0.3) < 0.01
        assert abs(y) < 1e-10
        assert abs(theta) < 1e-10

    def test_circle_returns_to_origin(self):
        """Full rotation should return near starting position."""
        x, y, theta = 0.0, 0.0, 0.0

        # Linear velocity and angular velocity for circular motion
        # v = omega * r, so for r=0.5m at omega=1 rad/s, v=0.5 m/s
        v_linear = 0.5
        v_angular = 1.0

        # Full circle: 2*pi radians at 1 rad/s = ~6.28 seconds
        dt = 0.01
        steps = int(2 * math.pi / (v_angular * dt))

        for _ in range(steps):
            x, y, theta = update_odometry(x, y, theta, v_linear, v_angular, dt)

        # Should be back near origin (with some integration error)
        assert abs(x) < 0.1
        assert abs(y) < 0.1


if __name__ == '__main__':
    if pytest:
        pytest.main([__file__, '-v'])
    else:
        # Run tests manually without pytest
        import sys
        passed = 0
        failed = 0

        test_classes = [
            TestVelocityToDuty,
            TestClampTickDelta,
            TestYawToQuaternion,
            TestDifferentialDriveKinematics,
            TestOdometryUpdate,
            TestEncoderConversion,
            TestIntegration,
        ]

        for test_class in test_classes:
            instance = test_class()
            for method_name in dir(instance):
                if method_name.startswith('test_'):
                    try:
                        getattr(instance, method_name)()
                        print(f'  PASS: {test_class.__name__}.{method_name}')
                        passed += 1
                    except AssertionError as e:
                        print(f'  FAIL: {test_class.__name__}.{method_name}: {e}')
                        failed += 1
                    except Exception as e:
                        print(f'  ERROR: {test_class.__name__}.{method_name}: {e}')
                        failed += 1

        print(f'\n{passed} passed, {failed} failed')
        sys.exit(0 if failed == 0 else 1)
