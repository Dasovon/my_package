# Session Notes — For Future Claude Sessions
**Last updated: 2026-02-20 (session 3)**
**Read this at the start of every session. It covers everything done so far.**

---

## Current Robot State

The robot is a differential drive hoverbot on a Raspberry Pi 4. As of this session:

- Wheel base is calibrated (0.236m effective)
- Both encoders are wired and working
- BNO055 IMU is wired and initialized
- EKF (robot_localization) is running but NOT publishing TF (motor controller owns TF)
- RPLIDAR A1 USB instability mostly resolved via auto power cycle in lidar_watchdog
- Motor min duty increased to 90% (motors were underpowered at 80%)
- **SLAM WORKING** — map builds and persists while driving
- slam.yaml tuned: resolution 0.025, smear deviation 0.03, link match response 0.45
- **BNO055 IMU CALIBRATED** — all 3s on calib_status; offsets saved to config/bno055.yaml

---

## Hardware

### Robot
- Differential drive, DG01D-E motors (1:48 gear ratio)
- L298N dual H-bridge motor driver
- Hall effect encoders: 3 magnets × 2 edges × 48:1 = 288 ticks/rev
- Raspberry Pi 4 (hostname: hoverbot)
- Dev machine (hostname: dev) for rviz/teleop/SLAM

### Calibrated Values
- `wheel_base`: 0.236m effective (physical center-to-center is 0.165m — effective is larger due to wheel scrub)
- `wheel_diameter`: 0.065m
- `encoder_ticks_per_rev`: 288

### GPIO Pin Assignments (BCM numbering)
| Function | Pin |
|---|---|
| Motor A Enable (PWM) | 17 |
| Motor A IN1 | 27 |
| Motor A IN2 | 22 |
| Motor B Enable (PWM) | 13 |
| Motor B IN3 | 19 |
| Motor B IN4 | 26 |
| Left Encoder A | 23 |
| Left Encoder B | 24 |
| Right Encoder A | 25 |
| Right Encoder B | 5 |

**encoder_left_inverted: true, encoder_right_inverted: false**

### Right Encoder Wiring (physical Pi4 header pins)
- `+` (VCC) → 3.3V → **Physical pin 1 or 17**
- `A` signal → GPIO 25 → **Physical pin 22**
- `B` signal → GPIO 5 → **Physical pin 29**
- `GND` → any ground pin

**THIS WAS THE PROBLEM THIS SESSION: right encoder VCC and A pin were both disconnected. Robot thought it was always turning right because right wheel reported 0 ticks. Caused clockwise scan spin in rviz and broke SLAM.**

### BNO055 IMU
- Adafruit BNO055 on breadboard
- I2C bus 1, address 0x28 (confirmed with i2cdetect)
- Wiring: VIN→3.3V, GND→GND, SDA→GPIO2 (pin 3), SCL→GPIO3 (pin 5), ADR floating
- Mounted at robot center (base_link), frame_id: `imu_link`
- Static TF published: `base_link` → `imu_link` (0,0,0 offset, identity rotation)
- Config: `config/bno055.yaml`

### RPLIDAR A1
- Connected via USB → CP2102 USB-UART bridge
- **USE THE BY-ID PATH, NOT ttyUSBx** — the port number changes every power cycle:
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- Cable faces the **back** of the robot
- URDF has `rpy="0 0 ${pi}"` — this is **CORRECT**. The A1's 0° points away from the cable, so pi rotation aligns it forward. Verified empirically: object placed in front of robot appears in front in rviz.
- Scan mode: Standard (2kHz, 10Hz). Sensitivity mode (8kHz) was tried but caused buffer overflow crashes.
- Config: `config/rplidar.yaml`

**RPLIDAR INSTABILITY ISSUE**: The lidar repeatedly enters bad states (health error 80008002, buffer overflow, or stuck at 84% CPU with 0 scan publishers). Root cause suspected: insufficient USB current from Pi. Symptoms:
  - Health error 80008002 → power cycle USB
  - Buffer overflow crash → stale process on wrong port (check `lsof /dev/ttyUSBx`)
  - Two processes fighting over port → kill both, restart one
  - Stuck process (no "Start" message) → `kill -9 <pid>` then power cycle USB
  - **Fix added**: `respawn=True, respawn_delay=7.0` in rplidar.launch.py so it auto-restarts
  - **Fix added**: lidar_watchdog auto power cycles USB on crash (see sudoers setup below)
  - **Long term fix needed**: powered USB hub

**RPLIDAR USB AUTO POWER CYCLE**: lidar_watchdog detects crashes and power cycles `/sys/bus/usb/devices/1-1.2/authorized`. Requires one-time sudoers setup on Pi:
```bash
echo 'ryan ALL=(ALL) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/1-1.2/authorized' | sudo tee /etc/sudoers.d/lidar-power-cycle
```
Without this, watchdog logs an error with the fix instructions. The power cycle has a 10s cooldown to prevent loops.

---

## Software Architecture

### ROS 2 Humble on Pi
Nodes launched by `full_bringup.launch.py`:
1. **robot_state_publisher** — publishes TF from URDF (base_footprint→base_link→laser_frame etc.)
2. **motor_controller** — wheel odometry, publishes `/odom` and `odom→base_footprint` TF
3. **rplidar_composition** — publishes `/scan`
4. **bno055** — publishes `/imu/data`, `/imu/imu_raw`, etc.
5. **static_transform_publisher** — `base_link` → `imu_link` (0,0,0)
6. **ekf_node** — fuses `/odom` + `/imu/data` → `/odometry/filtered`
7. **lidar_watchdog** — stops RPLIDAR motor when `/scan` has no subscribers, restarts when a subscriber appears; auto power cycles USB on crash

### TF Tree
```
map (published by slam_toolbox on dev machine)
└── odom (published by motor_controller via TF broadcaster)
    └── base_footprint
        └── base_link
            ├── laser_frame (from URDF)
            └── imu_link (static TF)
```

### EKF Configuration (current)
- `publish_tf: false` — motor controller owns odom→base_footprint TF
- `frequency: 30.0` Hz
- Fuses: wheel odom (x, y, yaw, vx, vyaw) + IMU (vyaw, ax)
- `imu0_relative: true` — don't use absolute orientation (magnetometer unreliable indoors)
- `imu0_remove_gravitational_acceleration: true`
- Config: `config/ekf.yaml`

**WHY EKF DOESN'T OWN TF**: When EKF published TF (`publish_tf: true`), it caused TF gaps that froze SLAM. The EKF at 30Hz on the Pi occasionally takes 200ms to process, creating gaps. Motor controller TF is event-driven (encoder interrupts) and never gaps. So motor controller publishes TF, EKF just publishes `/odometry/filtered` for reference.

### SLAM
- slam_toolbox online async mode
- Runs on **dev machine**, subscribes to `/scan` and TF from Pi over network
- Launch: `ros2 launch my_robot_bringup slam.launch.py` (wraps slam_toolbox with our params)
- Config: `config/slam.yaml`
- Key params: `base_frame: base_footprint`, `transform_timeout: 0.5`, `minimum_travel_distance: 0.2`

**SLAM STATUS: WORKING**. Map builds and persists while driving. Tuned for better quality — thinner walls, finer resolution.

**rviz2 tip**: To reduce LaserScan dot size → click LaserScan display → **Size (m)**: `0.02`, **Style**: `Points`.

---

## Key Files and What Changed

### `my_robot_bringup/motor_controller.py`
- `wheel_base` default: 0.165 → **0.236**
- `velocity_to_duty()`: added `min(..., 1.0)` clamp to prevent duty cycle > 100% (was crashing with ValueError when wheel_base increased)
- Added `publish_odom_tf` parameter (default True). TF broadcast is gated on it:
  ```python
  self.declare_parameter('publish_odom_tf', True)
  self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
  # In publish_odometry():
  if self.publish_odom_tf:
      self.tf_broadcaster.sendTransform(t)
  ```

### `config/motor_controller.yaml`
- `wheel_base: 0.236` (was 0.165)

### `config/rplidar.yaml`
- `serial_port`: changed to stable by-id path (not ttyUSB0/1/2)
- `scan_mode: Standard`

### `config/slam.yaml`
- `minimum_travel_distance: 0.2` (was 0.5)
- `minimum_travel_heading: 0.25` (was 0.5)
- `transform_timeout: 0.5` (was 0.2)
- `resolution: 0.025` (was 0.05) — finer map grid
- `link_match_minimum_response_fine: 0.45` (was 0.1) — only high-quality scans accepted
- `correlation_search_space_smear_deviation: 0.03` (was 0.1) — less wall smearing

### `config/ekf.yaml` (new)
- Fuses /odom + /imu/data
- `publish_tf: false` (motor controller owns TF)
- `frequency: 30.0`

### `config/bno055.yaml` (new)
- i2c_bus: 1, i2c_addr: 0x28
- operation_mode: 0x0C (NDOF)
- ros_topic_prefix: "imu/"
- data_query_frequency: 100
- `set_offsets: true` — calibration offsets saved and loaded on startup
- offset_acc: [-12, -2, -34] mg | offset_mag: [-203, 228, 550] uT/16 | offset_gyr: [-1, 0, -1] dps/16
- radius_acc: 1000 | radius_mag: 720

### `my_robot_bringup/lidar_watchdog.py` (new)
- Monitors `/scan` subscriber count every 2 seconds
- Calls `/stop_motor` when count drops to 0, `/start_motor` when count rises above 0
- Conserves battery during coding sessions when SLAM/rviz are not running
- Auto power cycles USB when crash detected (service goes unavailable), with 10s cooldown
- 8s grace period after startup before it will stop the motor (prevents SDK timeout)
- Motor can also be toggled manually: `ros2 service call /stop_motor std_srvs/srv/Empty {}` / `ros2 service call /start_motor std_srvs/srv/Empty {}`

### `launch/full_bringup.launch.py`
- Added BNO055 node
- Added static TF publisher (base_link → imu_link)
- Added EKF node
- Motor controller launched with `publish_odom_tf: true`
- Added lidar_watchdog node

### `launch/rplidar.launch.py`
- `respawn_delay`: 3.0 → **7.0** (allows watchdog power cycle to complete before respawn)

### `config/motor_controller.yaml`
- `min_duty_cycle`: 80 → **90** (motors were underpowered)

### `launch/motor_control.launch.py`
- Added `publish_odom_tf` launch argument (default 'true'), passed to node parameters

### `launch/rplidar.launch.py`
- Added `respawn=True, respawn_delay=3.0`

### `launch/slam.launch.py` (new)
- Wraps `slam_toolbox online_async_launch.py` with our `config/slam.yaml`
- Use this instead of typing the long slam_toolbox command

### `scripts/calibrate_wheel_base.py` (new)
- Spins robot for 20 seconds, user counts rotations
- Calculates effective wheel_base from ratio of expected vs actual rotations

### `maps/`
- `my_map.pgm` + `my_map.yaml` — nav2 format map
- `my_map_slam.data` + `my_map_slam.posegraph` — slam_toolbox serialized state

---

## Build and Launch

### On Pi (hoverbot)
```bash
cd ~/robot_ws && colcon build --packages-select my_robot_bringup --base-paths src/my_package
source ~/robot_ws/install/setup.bash
ros2 launch my_robot_bringup full_bringup.launch.py
```

### On Dev Machine
```bash
cd ~/dev_ws/src/my_package && git pull origin main
cd ~/dev_ws && colcon build --packages-select my_robot_bringup
source ~/dev_ws/install/setup.bash

# SLAM
ros2 launch my_robot_bringup slam.launch.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### rviz Setup for SLAM
- Fixed Frame: `map`
- Add: **Map** → `/map`
- Add: **LaserScan** → `/scan`
- Add: **RobotModel**

---

## Network
- Both machines: `ROS_DOMAIN_ID=0` (default)
- Dev machine previously had `ROS_DOMAIN_ID=42` in `~/.bashrc` — was removed 2026-02-12
- If rviz sees no topics: check `echo $ROS_DOMAIN_ID` on both machines

---

## Common Problems and Fixes

### `rclpy.ok()` error on `ros2 topic list`
ROS 2 daemon crashed. Fix:
```bash
ros2 daemon stop && ros2 daemon start
```

### Multiple duplicate nodes (motor_controller x3, bno055 x3, etc.)
Caused by partial relaunches stacking. Every bringup attempt that wasn't fully killed left orphan processes. Fix:
```bash
ps aux | grep -E "ros2|rplidar_composition|motor_controller.py|bno055|ekf_node|robot_state_pub|static_transform" | grep -v grep | awk '{print $2}' | xargs kill -9
```
Then relaunch once.

### RPLIDAR health error 80008002 / 80008000 / 80008001
The lidar_watchdog auto power cycles USB when it detects a crash — no manual action needed if the sudoers rule is set up. If not set up, the watchdog logs the fix command.

Manual power cycle:
```bash
echo '0' | sudo tee /sys/bus/usb/devices/1-1.2/authorized && sleep 2 && echo '1' | sudo tee /sys/bus/usb/devices/1-1.2/authorized
```
The device may also move from ttyUSB0 → ttyUSB1 → ttyUSB2 after each cycle — the by-id path handles this automatically.

### RPLIDAR buffer overflow / 2 processes on same port
```bash
lsof /dev/ttyUSBx   # find conflicting PIDs
kill -9 <pid1> <pid2>
```
Then restart the lidar node.

### RPLIDAR stuck (no "Start" message, 80%+ CPU, 0 scan publishers)
Kill the process and power cycle USB. The SDK is stuck waiting for hardware response.

### git repo corruption (empty object files)
Happened once due to power/disk issue. Recovery:
```bash
find .git/objects -size 0 -delete
git fetch origin  # may fail if corrupt objects were local commits
# If it fails, do the fresh clone approach:
git clone <remote> fresh_copy
# copy your files over, commit, push, swap directories
```

### Wheel base calibration method
Run `scripts/calibrate_wheel_base.py` — robot spins for 20 seconds, user counts full rotations.
Formula: `new_wheel_base = old_wheel_base × (reported_angle / actual_angle)`
Result: 0.165m physical → 0.236m effective (larger due to wheel scrub on floor)

### Encoder not working (0.0 position forever)
Check `/joint_states` topic — if right_wheel_joint stays at 0.0 while left accumulates, right encoder is dead.
Most likely cause: VCC wire disconnected. Right encoder VCC → Pi 3.3V (physical pin 1 or 17).

### SLAM map freezing / "no transform from" errors
Two known causes:
1. **EKF publishing TF with gaps** → Set `publish_tf: false` in ekf.yaml, let motor controller own TF
2. **Multiple node instances** → Kill everything and relaunch once clean

---

## What's Left (Next Session)

1. ~~**Test SLAM properly**~~ — **DONE**. SLAM working, map builds and persists.

2. ~~**IMU calibration**~~ — **DONE 2026-02-20**. Achieved all 3s on calib_status. Offsets saved to `config/bno055.yaml` (`set_offsets: true`). Robot starts calibrated from now on.

3. ~~**Save a good map**~~ — **DONE**. Saved 2026-02-20:
   - `maps/my_map.pgm` + `maps/my_map.yaml` — nav2 format
   - `maps/my_map_slam.data` + `maps/my_map_slam.posegraph` — slam_toolbox state

4. **Sudoers rule for USB power cycle** — must be run once on Pi if not already done:
   ```bash
   echo 'ryan ALL=(ALL) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/1-1.2/authorized' | sudo tee /etc/sudoers.d/lidar-power-cycle
   ```

5. **Stale process cleanup** — always kill all nodes before relaunching to avoid duplicate node issues:
   ```bash
   ps aux | grep -E "rplidar_composition|motor_controller.py|bno055|ekf_node|robot_state_pub|static_transform|lidar_watchdog" | grep -v grep | awk '{print $2}' | xargs kill -9
   ```

6. **Powered USB hub** — RPLIDAR still benefits from better USB power. Auto power cycle works around it but a hub is the real fix.

---

## Installed Packages (Pi)
```bash
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-bno055
```

## Installed Packages (Dev)
```bash
sudo apt install ros-humble-slam-toolbox
```
