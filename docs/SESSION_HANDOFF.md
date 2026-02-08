# Session Handoff - Hoverbot Development Status

**Date:** 2026-02-08
**Branch:** `claude/analyze-my-package-kB5q7`
**Latest Commit:** `eefa5d0` - Use commanded velocity for odometry (encoder direction unreliable)

---

## Project Overview

**Hoverbot** is a ROS 2 Humble differential-drive robot running on a Raspberry Pi 4/5 with Ubuntu 22.04. Development occurs on a separate Ubuntu 22.04 desktop machine.

### Current Hardware
- **Motors:** DG01D-E DC gearmotors (1:48 gear ratio) with L298N dual H-bridge driver
- **Encoders:** Hall effect wheel encoders (576 ticks/rev with quadrature, 288 effective after direction fix)
- **LIDAR:** RPLIDAR A1 (2D laser scanner)
- **Dimensions:** Wheel base 0.165m, wheel diameter 0.06475m

### Network Setup
- **Robot hostname:** `hoverbot` (Raspberry Pi)
- **Dev machine:** Ubuntu 22.04 desktop
- **ROS 2 Domain ID:** 42
- **Communication:** TCP/IP over Ethernet

---

## What's Working

1. **Motor control** - L298N PWM control with duty cycle ramping (80-100%)
2. **Keyboard teleoperation** - WASD control via `teleop_keyboard.py`
3. **RPLIDAR** - Laser scans publishing correctly (was rotated 180°, now fixed)
4. **URDF/TF tree** - Robot description and transforms publishing
5. **Odometry** - Position tracking (using commanded velocity as workaround)
6. **RViz visualization** - Robot model and laser scan display on dev machine

---

## Recent Fixes Applied

| Commit | Issue | Fix |
|--------|-------|-----|
| `eefa5d0` | Odometry X increased in both forward/backward | Use commanded velocity instead of unreliable encoder velocity |
| `9653ff3` | LIDAR orientation wrong | Rotated 180 degrees in config |
| `82e1306` | Left encoder direction wrong | Fixed for mirrored motor mount |
| `3625e9b` | Encoder ticks incorrect | Fixed ticks_per_rev, increased polling to 500Hz |
| `d865705` | Joint states not visible | Fixed QoS mismatch |

---

## Known Issues / Limitations

### Encoder Direction Detection (Main Issue)
The polling-based encoder reading at 500Hz cannot reliably detect wheel direction. When driving backward:
- Velocity oscillates between positive and negative values
- This caused odometry to always increase X (robot appeared to only move forward)

**Current Workaround:** Using commanded velocity (`self.current_linear_velocity`, `self.current_angular_velocity`) for odometry calculation instead of encoder-measured velocity.

**Proper Fix Needed:** Implement interrupt-based encoder reading using GPIO edge detection for accurate quadrature decoding and direction detection.

### Affected Code
File: `my_robot_bringup/motor_controller.py` (lines 358-368)

```python
# Current workaround - uses commanded velocity
linear_velocity = self.current_linear_velocity
angular_velocity = self.current_angular_velocity

# TODO: Replace with interrupt-based encoder reading
# left_velocity = self.calculate_wheel_velocity('left', current_time)
# right_velocity = self.calculate_wheel_velocity('right', current_time)
# linear_velocity = (right_velocity + left_velocity) / 2.0
# angular_velocity = (right_velocity - left_velocity) / self.wheel_base
```

---

## Key Files

| File | Purpose |
|------|---------|
| `my_robot_bringup/motor_controller.py` | Main motor control, encoder reading, odometry (488 lines) |
| `my_robot_bringup/teleop_keyboard.py` | Keyboard teleoperation node |
| `config/motor_controller.yaml` | GPIO pins, wheel params, speed limits |
| `config/rplidar.yaml` | LIDAR serial port settings |
| `launch/motor_control.launch.py` | Launch motor controller node |
| `launch/full_bringup.launch.py` | Launch complete robot (motors + LIDAR + state publisher) |

---

## GPIO Pin Assignments

### Motors (L298N)
- Enable A (Left PWM): GPIO 17
- Enable B (Right PWM): GPIO 13
- IN1 (Left Forward): GPIO 27
- IN2 (Left Backward): GPIO 22
- IN3 (Right Forward): GPIO 19
- IN4 (Right Backward): GPIO 26

### Encoders
- Left Encoder A: GPIO 23
- Left Encoder B: GPIO 24
- Right Encoder A: GPIO 25
- Right Encoder B: GPIO 5

---

## Commands Reference

### On Robot (hoverbot)
```bash
# Build workspace
cd ~/robot_ws && colcon build --symlink-install

# Launch motor controller only
ros2 launch my_robot_bringup motor_control.launch.py

# Launch full robot (motors + LIDAR + state publisher)
ros2 launch my_robot_bringup full_bringup.launch.py

# Monitor odometry
ros2 topic echo /odom --field twist.twist.linear.x

# Check TF
ros2 run tf2_ros tf2_echo odom base_footprint
```

### On Dev Machine
```bash
# Launch RViz
rviz2

# Keyboard teleop
ros2 run my_robot_bringup teleop_keyboard

# View TF tree
ros2 run tf2_tools view_frames
```

---

## Next Steps (Priority Order)

1. **Test current fix** - Verify forward/backward movement works correctly in RViz
2. **Implement interrupt-based encoders** - Use `GPIO.add_event_detect()` for edge-triggered quadrature decoding
3. **Add IMU (BNO055)** - For better odometry and orientation
4. **SLAM integration** - Map building with LIDAR
5. **Autonomous navigation** - Nav2 stack

---

## Testing Procedure

1. SSH into hoverbot: `ssh ryan@hoverbot`
2. Build: `cd ~/robot_ws && colcon build --symlink-install`
3. Launch: `ros2 launch my_robot_bringup motor_control.launch.py`
4. On dev machine: Open RViz, add RobotModel and TF, set Fixed Frame to `odom`
5. Run teleop: `ros2 run my_robot_bringup teleop_keyboard`
6. Test: Press W (forward) - robot should move forward in RViz
7. Test: Press S (backward) - robot should move backward in RViz
8. Verify: X translation should increase forward, decrease backward

---

## Repository Structure

```
my_package/
├── my_robot_bringup/
│   ├── motor_controller.py    # Main control node
│   └── teleop_keyboard.py     # Keyboard control
├── launch/                    # ROS 2 launch files
├── config/                    # YAML parameters
├── urdf/                      # Robot description
├── worlds/                    # Gazebo environments
├── test/                      # Unit tests
└── docs/                      # Documentation
```

---

## Important Context

- Package name is `my_robot_bringup` (not `my_package`)
- Robot workspace on Pi: `~/robot_ws`
- Dev workspace: `~/dev_ws`
- The encoder issue is a hardware/timing limitation of polling vs interrupts
- Current solution prioritizes working visualization over accurate velocity measurement
