# Session Handoff - Hoverbot Development Status

**Date:** 2026-02-08
**Branch:** `claude/analyze-my-package-kB5q7`
**Latest Commit:** `eefa5d0` - Use commanded velocity for odometry (encoder direction unreliable)

---

## Project Overview

**Hoverbot** is a ROS 2 Humble differential-drive robot running on a Raspberry Pi 4/5 with Ubuntu 22.04. Development occurs on a separate Ubuntu 22.04 desktop machine.

### Current Hardware
- **Motors:** DG01D-E DC gearmotors (1:48 gear ratio) with L298N dual H-bridge driver
- **Encoders:** Hall effect wheel encoders (288 ticks/rev: 3 pulses × 2 edges × 48:1 gear), interrupt-based quadrature decoding
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
5. **Odometry** - Encoder-based position tracking with interrupt-driven quadrature decoding
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

### ~~Encoder Direction Detection~~ (RESOLVED)
Previously, polling-based encoder reading at 500Hz could not reliably detect wheel direction. This has been replaced with **interrupt-based quadrature decoding** using `GPIO.add_event_detect()` on both encoder channels (A and B pins).

**Implementation:** `motor_controller.py` uses a quadrature state machine (`_quadrature_delta()`) with a lookup table for accurate direction sensing on every edge transition. Encoder inversion is handled via `encoder_left_inverted` (True) and `encoder_right_inverted` (False) parameters.

**Debounce:** Configurable via `encoder_bouncetime_ms` parameter (default: 1ms).

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

All pins are declared as ROS 2 parameters in `motor_controller.py` and overridden via `config/motor_controller.yaml`. Code defaults use physical pin numbers; YAML provides BCM GPIO numbers used at runtime (`GPIO.setmode(GPIO.BCM)`).

### Motors (L298N)
- Enable A (Left PWM): BCM 17 / physical pin 11 (`motor_enable_a_pin`)
- IN1 (Left Direction): BCM 27 / physical pin 13 (`motor_in1_pin`)
- IN2 (Left Direction): BCM 22 / physical pin 15 (`motor_in2_pin`)
- Enable B (Right PWM): BCM 13 / physical pin 33 (`motor_enable_b_pin`)
- IN3 (Right Direction): BCM 19 / physical pin 35 (`motor_in3_pin`)
- IN4 (Right Direction): BCM 26 / physical pin 37 (`motor_in4_pin`)

### Encoders (interrupt-based quadrature)
- Left Encoder A: BCM 23 / physical pin 16 (`encoder_left_a_pin`)
- Left Encoder B: BCM 24 / physical pin 18 (`encoder_left_b_pin`)
- Right Encoder A: BCM 25 / physical pin 22 (`encoder_right_a_pin`)
- Right Encoder B: BCM 5 / physical pin 29 (`encoder_right_b_pin`)

### Encoder Configuration
- `encoder_ticks_per_rev`: 288 (3 pulses × 2 edges × 48:1 gear)
- `encoder_left_inverted`: True
- `encoder_right_inverted`: False
- `encoder_bouncetime_ms`: 1

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

1. ~~**Implement interrupt-based encoders**~~ - DONE: `GPIO.add_event_detect()` quadrature decoding implemented
2. **Add IMU (BNO055)** - For better odometry and orientation
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
- Encoder direction detection now uses interrupt-based quadrature decoding (resolved)
- Encoder inversion handled via `encoder_left_inverted` / `encoder_right_inverted` parameters
