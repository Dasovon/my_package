# Project Roadmap and TODO List

**Last Updated:** February 20, 2026 (session 3)

---

## Phase 1: Base System Setup (COMPLETE)

- [x] Install Ubuntu 22.04 on dev and hoverbot
- [x] Configure network (hostname resolution, SSH)
- [x] Install ROS 2 Humble on both machines
- [x] Set up ROS_DOMAIN_ID for network isolation
- [x] Create workspaces (dev_ws, robot_ws)
- [x] Create my_robot_bringup package
- [x] Verify cross-machine ROS 2 communication
- [x] Set up Git workflow for package sync
- [x] Configure proper package install directives
- [x] Update package.xml metadata
- [x] Create comprehensive documentation

---

## Phase 2: Sensor Integration (COMPLETE)

### RPLIDAR A1 Integration
- [x] Install rplidar_ros ROS 2 package
- [x] Configure serial permissions (/dev/ttyUSB0)
- [x] Create rplidar.launch.py with parameters
- [x] Verify /scan topic publishes LaserScan messages
- [x] Set up static transform (base_link -> laser_frame)
- [x] Create minimal URDF for robot structure
- [x] Add robot_state_publisher to launch
- [x] Visualize laser scans in RViz

---

## Phase 3: Robot Description and Visualization (COMPLETE)

- [x] Create robot base geometry (chassis dimensions)
- [x] Add wheel positions and sizes
- [x] Define all sensor frame positions
- [x] Use Xacro for parametric design
- [x] Publish complete robot_state_publisher
- [x] Verify all TF frames published
- [x] Check transform timestamps

---

## Phase 4: Motor Control and Odometry (COMPLETE)

### Motor Controller
- [x] L298N motor driver with DG01D-E gearmotors
- [x] Wire controller to Raspberry Pi GPIO (BCM pin mapping)
- [x] Create motor_controller.py node (subscribe /cmd_vel)
- [x] Implement differential drive velocity control
- [x] Add PWM ramping and duty cycle limits
- [x] Hall effect encoder integration (quadrature decoding)
- [x] Publish /odom topic with covariance values
- [x] Publish /joint_states for wheel visualization
- [x] Configure transforms (odom -> base_footprint -> base_link)
- [x] Implement watchdog timeout safety stop
- [x] Encoder tick delta clamping (noise rejection)
- [x] GPIO access validation before initialization
- [x] Teleop keyboard working from dev machine

### Calibration (COMPLETE)
- [x] Calibrate encoder_ticks_per_rev (288 ticks/rev)
- [x] Calibrate wheel_diameter (65mm)
- [x] Calibrate wheel_base via rotation test (0.165m physical -> 0.236m effective, +0.08% heading error)
- [x] Set BCM pin defaults to match hardware
- [x] Disable encoder bouncetime (hall effect sensors don't need it)

### Unit Tests (COMPLETE - 69/69 passing)
- [x] Velocity to duty cycle mapping
- [x] Encoder tick clamping
- [x] Yaw to quaternion conversion
- [x] Differential drive kinematics (forward + inverse)
- [x] Odometry integration
- [x] Encoder distance conversion
- [x] Integration tests (straight line, circle)
- [x] Lint: flake8, pep257, copyright, cmake, xmllint

---

## Phase 5: SLAM and Mapping (IN PROGRESS)

### slam_toolbox Setup
- [x] Install slam_toolbox on dev machine
- [x] Create slam.yaml config tuned for hardware
- [x] Verify TF tree: map -> odom -> base_footprint -> base_link -> laser_frame
- [x] Launch SLAM alongside full_bringup (use `slam.launch.py` wrapper)
- [x] Visualize map building in rviz2 from dev machine
- [x] Tune wheel_base via rotation calibration (0.165m -> 0.236m, heading error now <0.1%)
- [x] Add BNO055 IMU (I2C bus 1, address 0x28) -- integrated and publishing
- [x] Add robot_localization EKF -- fusing /odom + /imu/data -> /odometry/filtered
- [x] Save initial maps (maps/ dir: .pgm/.yaml + slam_toolbox serialized)
- [x] RPLIDAR USB auto power cycle -- lidar_watchdog detects crashes and power cycles USB automatically (requires sudoers rule, see SESSION_NOTES.md)
- [x] Lidar motor watchdog -- stops motor when nothing subscribed to /scan, starts when SLAM/rviz connects
- [x] **SLAM map stable during driving** -- working as of 2026-02-20, map builds and persists
- [x] SLAM map quality tuned -- resolution 0.025, smear deviation 0.03, link match response 0.45
- [ ] Map a full room successfully and save
- [ ] IMU calibration -- BNO055 gyro needs calibration (leave still 30s on startup)

---

## Phase 6: Navigation (NOT STARTED)

### Nav2 Setup
- [ ] Install Nav2 packages
- [ ] Configure costmaps (global, local)
- [ ] Set up planners (NavFn, DWB)
- [ ] Configure recovery behaviors
- [ ] Test navigation to waypoints

### Autonomous Behavior
- [ ] Waypoint following
- [ ] Patrol behavior
- [ ] Obstacle avoidance testing

---

## Phase 7: Advanced Features (Future)

### Additional Sensors
- [x] BNO055 IMU -- integrated (I2C, address 0x28, NDOF mode, 100Hz)
- [ ] RealSense D435i depth camera

### Sensor Fusion
- [x] IMU + wheel odometry fusion (robot_localization EKF) -- running, not yet owning TF
- [ ] Visual odometry from RealSense

### Computer Vision
- [ ] Object detection
- [ ] AprilTag detection

---

## Known Issues

### To Fix
- [x] ~~SLAM map freezing during driving~~ -- fixed (TF ownership + lidar watchdog)
- [x] ~~Heading drift ~10 deg over 600mm straight-line travel~~ -- fixed via wheel_base calibration (0.236m)
- [ ] RPLIDAR USB power instability -- auto power cycle workaround added; get powered USB hub for real fix
- [ ] Remove duplicate cmd_vel publishing in teleop (timer + immediate)
- [ ] Calculate accurate URDF inertia values (using placeholders)

### Resolved
- [x] ROS_DOMAIN_ID -- both machines using default 0 (dev machine had 42 in .bashrc, removed 2026-02-12)
- [x] Right encoder dead -- VCC and signal A wires disconnected (fixed 2026-02-17)

### To Investigate
- [ ] WiFi power management on Pi (potential disconnects)
- [ ] Motor controller CPU usage ~33% (consider C++ port if needed)

---

## Hardware Reference

### Current Configuration
- **SBC:** Raspberry Pi (Ubuntu 22.04, ROS 2 Humble)
- **Motors:** DG01D-E with 1:48 gear ratio, hall effect encoders
- **Encoder:** 3 magnets, 288 ticks/wheel rev (quadrature)
- **Driver:** L298N dual H-bridge
- **Lidar:** RPLIDAR A1 (0.15-12m range, 5.5-10Hz, 8kHz sample rate)
- **Wheels:** 65mm diameter, 165mm physical wheel base (0.236m effective)

### GPIO Pin Map (BCM)
| Function | Pin |
|---|---|
| Motor Enable A | 17 |
| Motor IN1 | 27 |
| Motor IN2 | 22 |
| Motor Enable B | 13 |
| Motor IN3 | 19 |
| Motor IN4 | 26 |
| Encoder Left A | 23 |
| Encoder Left B | 24 |
| Encoder Right A | 25 |
| Encoder Right B | 5 |
