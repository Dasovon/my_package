# Project Roadmap and TODO List

**Last Updated:** February 11, 2026

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
- [x] Install slam_toolbox on robot
- [x] Create slam.yaml config tuned for hardware
- [x] Verify TF tree: map -> odom -> base_footprint -> base_link -> laser_frame
- [x] Launch SLAM alongside full_bringup
- [x] Visualize map building in rviz2 from dev machine
- [ ] **Fix map drift/quality** -- map is unstable during driving
  - Root cause: 288 ticks/rev encoder resolution is too low for
    reliable odometry (articubot_one uses 3436 ticks/rev = 12x more)
  - Heading drift measured at ~10 deg over 600mm straight-line travel
  - SLAM params tuned conservatively (matching articubot_one) across
    multiple sessions -- param tuning alone is insufficient
  - correlation_search_space_dimension > 0.5 crashes Pi (out of memory)
  - **Next steps to try (priority order):**
    - [ ] Tune wheel_base via rotation calibration (spin 360 deg in place)
          -- free, heading error is likely the biggest contributor
    - [ ] Add BNO055 IMU for heading correction (I2C/UART)
          -- would dramatically improve odometry quality for SLAM
    - [ ] Test with robot_localization EKF once IMU added
    - [ ] Try sync mode instead of async
    - [ ] Upgrade to higher-resolution encoders (last resort, hardware change)
- [ ] Save and load maps
- [ ] Map a full room successfully

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
- [ ] BNO055 IMU (I2C/UART) -- would significantly improve odometry
- [ ] RealSense D435i depth camera

### Sensor Fusion
- [ ] IMU + wheel odometry fusion (robot_localization EKF)
- [ ] Visual odometry from RealSense

### Computer Vision
- [ ] Object detection
- [ ] AprilTag detection

---

## Known Issues

### To Fix
- [ ] Map drift during SLAM (encoder resolution limiting factor)
- [ ] Heading drift ~10 deg over 600mm straight-line travel
- [ ] Remove duplicate cmd_vel publishing in teleop (timer + immediate)
- [ ] Calculate accurate URDF inertia values (using placeholders)

### Resolved
- [x] ROS_DOMAIN_ID -- both machines using default 0, cross-machine comms working

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
- **Wheels:** 65mm diameter, 165mm wheel base

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
