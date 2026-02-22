# Hoverbot — Claude Instructions

**Robot:** Differential drive hoverbot on Raspberry Pi 4 (hostname: `hoverbot`, IP: 192.168.86.33)
**Stack:** ROS 2 Humble, Ubuntu 22.04 arm64
**Package:** `~/dev_ws/src/my_package` on dev machine (GitHub: `Dasovon/my_package`)
**Session notes:** `~/dev_ws/src/my_package/docs/SESSION_NOTES.md` — read for full history

---

## Session Protocol

**At the start of every session:**
- Read `~/dev_ws/src/my_package/docs/SESSION_NOTES.md` in full before doing anything else.

**At the end of every session:**
- Update `~/dev_ws/src/my_package/docs/SESSION_NOTES.md` with what was done, what changed, any new problems/fixes discovered, and what's left for next session.
- Update this file (`CLAUDE.md`) to reflect any changes to hardware, key files, build steps, or common fixes.
- Commit and push both files.

---

## Environment

**Claude Code runs on the dev machine** (`ryan@192.168.86.52`, `~/dev_ws`).

- Dev workspace: `~/dev_ws/src/my_package`
- Pi is accessed via SSH: `ssh ryan@192.168.86.33`
- Bidirectional SSH key auth is set up (no password needed in either direction)
- ROS_DOMAIN_ID=0 on both machines

To run commands on the Pi from dev:
```bash
ssh ryan@192.168.86.33 "<command>"
```

---

## Project Status (as of 2026-02-22)

- Phases 1–5 complete: motors, encoders, RPLIDAR, IMU, SLAM all working
- Map saved: `maps/my_map.*`
- **Phase 6 (Nav2) in progress** — config and launch files exist, not yet tested end-to-end
- Next task: drive to a Nav2 goal and verify end-to-end

---

## Hardware

| Component | Detail |
|---|---|
| Motors | DG01D-E, 1:48 gear ratio, L298N driver |
| Encoders | Hall effect, 3 magnets × 2 edges × 48 = 288 ticks/rev |
| Wheel diameter | 0.065 m |
| Wheel base (effective) | 0.236 m (physical 0.165 m, larger due to wheel scrub) |
| IMU | Adafruit BNO055, I2C bus 1, address 0x28 |
| Lidar | RPLIDAR A1, `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| ROS_DOMAIN_ID | 0 (default) |

### GPIO (BCM numbering)
| Function | BCM Pin |
|---|---|
| Motor A Enable (PWM) | 17 |
| Motor A IN1/IN2 | 27, 22 |
| Motor B Enable (PWM) | 13 |
| Motor B IN3/IN4 | 19, 26 |
| Left Encoder A/B | 23, 24 |
| Right Encoder A/B | 25, 5 |

`encoder_left_inverted: true`, `encoder_right_inverted: false`

---

## Key Files

| File | Purpose |
|---|---|
| `my_robot_bringup/motor_controller.py` | Wheel odometry, publishes `/odom` and `odom→base_footprint` TF |
| `my_robot_bringup/lidar_watchdog.py` | Monitors `/scan`, stops/starts lidar motor, auto power cycles USB on crash |
| `launch/full_bringup.launch.py` | Launches all Pi nodes (RSP, motor, lidar, IMU, EKF, watchdog) |
| `launch/slam.launch.py` | Wraps slam_toolbox with `config/slam.yaml` (run on dev machine) |
| `launch/nav2.launch.py` | Nav2 bringup (run on dev machine) |
| `config/motor_controller.yaml` | Motor/encoder params, `min_duty_cycle: 90`, `wheel_base: 0.236` |
| `config/slam.yaml` | SLAM params (resolution 0.025, conservative quality settings) |
| `config/ekf.yaml` | EKF: fuses `/odom` + `/imu/data`, `publish_tf: false` |
| `config/bno055.yaml` | IMU config + calibration offsets (loaded on startup) |
| `config/nav2_params.yaml` | Nav2 params (transform_tolerance: 0.5 for network latency) |
| `maps/my_map.*` | Saved map (nav2 + slam_toolbox formats) |

---

## TF Tree

```
map  ← slam_toolbox (dev machine)
└── odom  ← motor_controller (Pi)
    └── base_footprint
        └── base_link
            ├── laser_frame  (URDF)
            └── imu_link     (static TF, 0,0,0)
```

**EKF does NOT own TF** (`publish_tf: false`) — motor controller owns `odom→base_footprint` because EKF at 30 Hz occasionally gaps and freezes SLAM.

---

## Build and Launch

### On Pi (via SSH from dev)
```bash
ssh ryan@192.168.86.33 "cd ~/robot_ws && colcon build --packages-select my_robot_bringup --base-paths src/my_package && source ~/robot_ws/install/setup.bash"
ssh ryan@192.168.86.33 "source ~/robot_ws/install/setup.bash && ros2 launch my_robot_bringup full_bringup.launch.py"
```

**Kill stale nodes on Pi:**
```bash
ssh ryan@192.168.86.33 "ps aux | grep -E 'rplidar_composition|motor_controller.py|bno055|ekf_node|robot_state_pub|static_transform|lidar_watchdog' | grep -v grep | awk '{print \$2}' | xargs kill -9"
```

### On Dev Machine
```bash
cd ~/dev_ws/src/my_package && git pull origin main
cd ~/dev_ws && colcon build --packages-select my_robot_bringup
source ~/dev_ws/install/setup.bash
ros2 launch my_robot_bringup slam.launch.py       # SLAM
ros2 launch my_robot_bringup nav2.launch.py       # Nav2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **Note:** `slam.yaml` must be manually copied to the install dir after changes — symlink-install doesn't pick up config files added after first build.

---

## Common Problems and Fixes

### ROS daemon crash (`rclpy.ok()` error)
```bash
ros2 daemon stop && ros2 daemon start
```

### RPLIDAR health error (80008002, 80008000, 80008001)
Lidar watchdog auto power cycles USB if sudoers rule is in place. Manual power cycle (from dev):
```bash
ssh ryan@192.168.86.33 "echo '0' | sudo tee /sys/bus/usb/devices/1-1.2/authorized && sleep 2 && echo '1' | sudo tee /sys/bus/usb/devices/1-1.2/authorized"
```
If sudoers rule is missing (run on Pi):
```bash
sudo python3 -c "open('/etc/sudoers.d/lidar-power-cycle','w').write('ryan ALL=(ALL) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/1-1.2/authorized\n')"
```

### RPLIDAR timeout loop on startup
Software authorized-toggle may not fully reset hardware. **Physical unplug/replug is the reliable fix.**

### RPLIDAR 2 processes on same port
```bash
ssh ryan@192.168.86.33 "lsof /dev/ttyUSBx"   # find conflicting PIDs
ssh ryan@192.168.86.33 "kill -9 <pid1> <pid2>"
```

### Encoder showing 0.0 forever
Check `/joint_states`. If one wheel stays at 0, encoder VCC is disconnected.
Right encoder VCC → Pi 3.3V (physical pin 1 or 17).

### Nav2 "Timed out waiting for transform from base_footprint to map"
AMCL needs an initial pose. In rviz2: **2D Pose Estimate** → click+drag on map.

### SLAM map freezing / "no transform from" errors
1. EKF publishing TF with gaps → ensure `publish_tf: false` in `config/ekf.yaml`
2. Duplicate nodes → kill everything and relaunch clean

### git repo corruption (empty object files)
```bash
find .git/objects -size 0 -delete
git fetch origin
```

---

## Lidar Notes
- **Always use the by-id path**, not `ttyUSB0` — port number changes on each power cycle
- URDF has `rpy="0 0 ${pi}"` for laser_frame — this is correct (A1 0° points away from cable)
- Scan mode: Standard (2kHz, 10Hz). Sensitivity mode causes buffer overflow crashes.
- Watchdog toggles lidar motor when `/scan` has no subscribers (battery conservation)
- `respawn_delay: 7.0` in rplidar.launch.py — allows watchdog USB power cycle to complete

---

## Installed Packages

**Pi:** `ros-humble-robot-localization`, `ros-humble-bno055`
**Dev:** `ros-humble-slam-toolbox`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`
