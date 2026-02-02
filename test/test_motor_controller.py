#!/usr/bin/env python3
"""
Unit tests for motor_controller.py

Tests core logic functions without requiring GPIO or ROS 2 runtime.
Run with: python3 -m pytest test/test_motor_controller.py -v
"""

import math
import pytest


class TestVelocityToDuty:
    """Tests for velocity to duty cycle conversion logic."""

    def velocity_to_duty(self, velocity, max_speed=0.5, min_duty=80, max_duty=100):
        """
        Replicates the velocity_to_duty logic from motor_controller.py
        for isolated unit testing.
        """
        if abs(velocity) < 0.01:
            return 0.0

        velocity_fraction = abs(velocity) / max_speed
        duty = min_duty + (velocity_fraction * (max_duty - min_duty))

        return duty if velocity >= 0 else -duty

    def test_zero_velocity_returns_zero(self):
        """Zero velocity should return zero duty cycle."""
        assert self.velocity_to_duty(0.0) == 0.0

    def test_small_velocity_returns_zero(self):
        """Very small velocities (below threshold) should return zero."""
        assert self.velocity_to_duty(0.005) == 0.0
        assert self.velocity_to_duty(-0.005) == 0.0

    def test_max_velocity_returns_max_duty(self):
        """Maximum velocity should return maximum duty cycle."""
        result = self.velocity_to_duty(0.5)  # max_speed = 0.5
        assert result == 100.0

    def test_min_velocity_returns_min_duty(self):
        """Minimum non-zero velocity should return minimum duty cycle."""
        # Just above threshold
        result = self.velocity_to_duty(0.01)
        assert result == pytest.approx(80.4, rel=0.1)

    def test_negative_velocity_returns_negative_duty(self):
        """Negative velocity should return negative duty cycle."""
        result = self.velocity_to_duty(-0.25)
        assert result < 0
        assert abs(result) == pytest.approx(90.0, rel=0.1)

    def test_half_speed_returns_mid_duty(self):
        """Half of max speed should return midpoint duty cycle."""
        result = self.velocity_to_duty(0.25)  # 50% of max_speed=0.5
        assert result == pytest.approx(90.0, rel=0.1)  # midpoint of 80-100


class TestClampTickDelta:
    """Tests for encoder tick delta clamping logic."""

    def clamp_tick_delta(self, delta, max_tick_delta=1200):
        """
        Replicates the clamp_tick_delta logic from motor_controller.py
        for isolated unit testing.
        """
        if abs(delta) > max_tick_delta:
            return max_tick_delta if delta > 0 else -max_tick_delta
        return delta

    def test_normal_delta_unchanged(self):
        """Normal deltas within range should pass through unchanged."""
        assert self.clamp_tick_delta(100) == 100
        assert self.clamp_tick_delta(-100) == -100
        assert self.clamp_tick_delta(0) == 0

    def test_max_delta_unchanged(self):
        """Delta at exactly max should pass through unchanged."""
        assert self.clamp_tick_delta(1200) == 1200
        assert self.clamp_tick_delta(-1200) == -1200

    def test_over_max_positive_clamped(self):
        """Positive delta over max should be clamped."""
        assert self.clamp_tick_delta(5000) == 1200
        assert self.clamp_tick_delta(1201) == 1200

    def test_over_max_negative_clamped(self):
        """Negative delta over max should be clamped."""
        assert self.clamp_tick_delta(-5000) == -1200
        assert self.clamp_tick_delta(-1201) == -1200


class TestOdometryCalculation:
    """Tests for odometry calculation logic."""

    def update_odometry(self, x, y, theta, v_left, v_right, dt, wheel_base=0.165):
        """
        Replicates the odometry calculation from motor_controller.py
        for isolated unit testing.
        """
        v_linear = (v_left + v_right) / 2.0
        v_angular = (v_right - v_left) / wheel_base

        delta_theta = v_angular * dt
        delta_x = v_linear * math.cos(theta + delta_theta / 2.0) * dt
        delta_y = v_linear * math.sin(theta + delta_theta / 2.0) * dt

        new_x = x + delta_x
        new_y = y + delta_y
        new_theta = theta + delta_theta

        # Normalize theta
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

        return new_x, new_y, new_theta

    def test_stationary_no_movement(self):
        """Stationary robot should not move."""
        x, y, theta = self.update_odometry(0, 0, 0, 0, 0, 0.02)
        assert x == 0.0
        assert y == 0.0
        assert theta == 0.0

    def test_forward_motion(self):
        """Equal wheel velocities should produce forward motion."""
        x, y, theta = self.update_odometry(0, 0, 0, 0.5, 0.5, 1.0)
        assert x == pytest.approx(0.5, rel=0.01)
        assert y == pytest.approx(0.0, abs=0.001)
        assert theta == pytest.approx(0.0, abs=0.001)

    def test_backward_motion(self):
        """Negative equal velocities should produce backward motion."""
        x, y, theta = self.update_odometry(0, 0, 0, -0.5, -0.5, 1.0)
        assert x == pytest.approx(-0.5, rel=0.01)
        assert y == pytest.approx(0.0, abs=0.001)

    def test_rotation_in_place(self):
        """Opposite wheel velocities should produce rotation."""
        # Left wheel backward, right wheel forward = turn left (positive theta)
        x, y, theta = self.update_odometry(0, 0, 0, -0.1, 0.1, 1.0, wheel_base=0.165)
        assert abs(x) < 0.01  # Should stay roughly in place
        assert abs(y) < 0.01
        assert theta > 0  # Should have rotated

    def test_theta_normalization(self):
        """Theta should be normalized to [-pi, pi]."""
        # Start at theta close to pi, rotate more
        x, y, theta = self.update_odometry(0, 0, 3.0, -0.5, 0.5, 1.0)
        assert -math.pi <= theta <= math.pi


class TestYawToQuaternion:
    """Tests for yaw to quaternion conversion."""

    def yaw_to_quaternion(self, yaw):
        """Replicates yaw_to_quaternion from motor_controller.py"""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def test_zero_yaw(self):
        """Zero yaw should produce identity-like quaternion."""
        q = self.yaw_to_quaternion(0.0)
        assert q['x'] == 0.0
        assert q['y'] == 0.0
        assert q['z'] == pytest.approx(0.0, abs=0.001)
        assert q['w'] == pytest.approx(1.0, abs=0.001)

    def test_90_degree_yaw(self):
        """90 degree yaw should produce correct quaternion."""
        q = self.yaw_to_quaternion(math.pi / 2)
        assert q['z'] == pytest.approx(math.sqrt(2) / 2, rel=0.01)
        assert q['w'] == pytest.approx(math.sqrt(2) / 2, rel=0.01)

    def test_180_degree_yaw(self):
        """180 degree yaw should produce correct quaternion."""
        q = self.yaw_to_quaternion(math.pi)
        assert q['z'] == pytest.approx(1.0, rel=0.01)
        assert q['w'] == pytest.approx(0.0, abs=0.01)

    def test_quaternion_normalized(self):
        """Quaternion should be normalized (magnitude = 1)."""
        for yaw in [0, math.pi / 4, math.pi / 2, math.pi, -math.pi / 2]:
            q = self.yaw_to_quaternion(yaw)
            magnitude = math.sqrt(q['x']**2 + q['y']**2 + q['z']**2 + q['w']**2)
            assert magnitude == pytest.approx(1.0, rel=0.001)


class TestDifferentialDriveKinematics:
    """Tests for differential drive velocity calculations."""

    def cmd_vel_to_wheel_velocities(self, linear, angular, wheel_base=0.165):
        """
        Replicates the cmd_vel_callback wheel velocity calculation
        from motor_controller.py
        """
        vel_left = linear - (angular * wheel_base / 2.0)
        vel_right = linear + (angular * wheel_base / 2.0)
        return vel_left, vel_right

    def test_forward_command(self):
        """Pure forward motion should give equal wheel velocities."""
        left, right = self.cmd_vel_to_wheel_velocities(0.5, 0.0)
        assert left == pytest.approx(0.5, rel=0.001)
        assert right == pytest.approx(0.5, rel=0.001)

    def test_rotation_command(self):
        """Pure rotation should give opposite wheel velocities."""
        left, right = self.cmd_vel_to_wheel_velocities(0.0, 1.0, wheel_base=0.165)
        assert left == pytest.approx(-0.0825, rel=0.01)
        assert right == pytest.approx(0.0825, rel=0.01)

    def test_combined_motion(self):
        """Combined linear and angular should work correctly."""
        left, right = self.cmd_vel_to_wheel_velocities(0.3, 0.5, wheel_base=0.165)
        # left = 0.3 - (0.5 * 0.165 / 2) = 0.3 - 0.04125 = 0.25875
        # right = 0.3 + (0.5 * 0.165 / 2) = 0.3 + 0.04125 = 0.34125
        assert left == pytest.approx(0.25875, rel=0.01)
        assert right == pytest.approx(0.34125, rel=0.01)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
