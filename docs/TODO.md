# Project Roadmap and TODO List

**Last Updated:** February 3, 2026

---

## ✅ Phase 1: Base System Setup (COMPLETE)

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

## ✅ Phase 2: Sensor Integration (COMPLETE)

### RPLIDAR A1 Integration
- [x] Install rplidar_ros ROS 2 package
- [x] Configure serial permissions (/dev/ttyUSB0)
- [x] Create rplidar.launch.py with parameters
- [x] Verify /scan topic publishes LaserScan messages
- [x] Set up static transform (base_link → laser_frame)
- [x] Create minimal URDF for robot structure
- [x] Add robot_state_publisher to launch
- [x] Visualize laser scans in RViz
- [x] Document RPLIDAR setup in docs/

### BNO055 IMU Integration
- [ ] Wire IMU to Raspberry Pi I2C or UART
- [ ] Install BNO055 ROS 2 driver
- [ ] Configure IMU parameters (frame_id, update rate)
- [ ] Verify /imu topic publishes Imu messages
- [ ] Check orientation, axis mapping, and covariance
- [ ] Add static transform (base_link → imu_link)
- [ ] Update URDF with IMU position
- [ ] Visualize IMU data in RViz (orientation arrows)
- [ ] Document IMU setup in docs/IMU.md

### RealSense D435i Integration
- [ ] Install librealsense2 and realsense-ros
- [ ] Connect camera via USB 3.0
- [ ] Configure camera parameters (resolution, frame rate)
- [ ] Verify depth, color, and IMU topics
- [ ] Add transforms (base_link → camera_link → camera_depth_frame)
- [ ] Update URDF with camera position
- [ ] Visualize depth point cloud in RViz
- [ ] Test USB bandwidth and latency
- [ ] Document camera setup in docs/CAMERA.md

---

## ✅ Phase 3: Robot Description and Visualization (COMPLETE)

### URDF Development
- [x] Create robot base geometry (chassis dimensions)
- [x] Add wheel positions and sizes
- [x] Define all sensor frame positions
- [x] Use Xacro for parametric design
- [x] Publish complete robot_state_publisher

### RViz Configuration
- [ ] Create default RViz config file
- [ ] Configure fixed frame (map or base_link)
- [ ] Add LaserScan display for RPLIDAR
- [ ] Add IMU orientation display
- [ ] Add PointCloud2 display for RealSense
- [ ] Add TF display for frame tree
- [ ] Save config to package (config/default.rviz)

### TF Tree Validation
- [x] Verify all frames published
- [x] Check transform timestamps (avoid stale warnings)
- [ ] Use tf2_tools to visualize tree
- [ ] Document frame conventions in docs/FRAMES.md

---

## ✅ Phase 4: Motor Control and Odometry (COMPLETE)

### Motor Controller Setup
- [x] L298N motor driver with DG01D-E gearmotors
- [x] Wire controller to Raspberry Pi GPIO
- [x] Create motor_controller.py node (subscribe /cmd_vel)
- [x] Implement differential drive velocity control
- [x] Add PWM ramping and duty cycle limits
- [x] Hall effect encoder integration (576 ticks/rev)
- [x] Publish /odom topic with covariance values
- [x] Configure transforms (odom → base_footprint → base_link)
- [x] Test on bench (wheels off ground)
- [x] Implement emergency stop mechanism
- [x] Document motor setup in docs/L298N_MOTOR_CONTROL.md
- [x] Document encoder odometry in docs/ENCODER_ODOMETRY_COMPLETE.md

### Safety Features
- [x] Velocity limits (max linear 0.3 m/s, max angular 1.0 rad/s)
- [x] Min duty cycle threshold (80% for motor startup)
- [x] Timeout watchdog (stop if no commands for 1s)
- [x] Encoder tick delta clamping (noise rejection)
- [x] GPIO access validation before initialization
- [ ] Battery voltage monitoring
- [ ] Over-current protection
- [ ] Physical emergency stop button

---

## 🚧 Phase 5: Localization and Mapping (IN PROGRESS)

### Static Map Testing
- [ ] Install nav2 packages
- [ ] Create simple test map (hallway, room)
- [ ] Configure AMCL for localization
- [ ] Test position estimation with RPLIDAR

### SLAM
- [ ] Install slam_toolbox package
- [ ] Configure SLAM parameters
- [ ] Map test environment
- [ ] Save and load maps
- [ ] Document SLAM workflow in docs/SLAM.md

---

## 📋 Phase 6: Navigation

### Nav2 Setup
- [ ] Install Nav2 packages
- [ ] Configure costmaps (global, local)
- [ ] Set up planners (NavFn, DWB)
- [ ] Configure recovery behaviors
- [ ] Test navigation to waypoints
- [ ] Tune parameters for smooth motion

### Autonomous Behavior
- [ ] Waypoint following
- [ ] Patrol behavior
- [ ] Obstacle avoidance testing
- [ ] Dynamic re-planning

---

## 📋 Phase 7: Advanced Features (Future)

### Sensor Fusion
- [ ] IMU + wheel odometry fusion (robot_localization)
- [ ] Visual odometry from RealSense
- [ ] Multi-sensor Kalman filter

### Computer Vision
- [ ] Object detection (YOLO, TensorFlow)
- [ ] AprilTag detection
- [ ] Person tracking

### Manipulation (if arm added)
- [ ] MoveIt2 integration
- [ ] Gripper control
- [ ] Pick and place tasks

---

## 🐛 Known Issues

### To Fix
- [ ] Remove duplicate cmd_vel publishing in teleop (timer + immediate on keypress)
- [ ] Calculate accurate URDF inertia values (currently using placeholders)

### To Investigate
- [ ] WiFi power management on Pi (potential disconnects)
- [ ] USB power budget with all sensors connected
- [ ] Add ROS 2 integration tests (node communication, topic verification)

---

## 📚 Learning Resources

### Completed Tutorials
1. ✅ Articulated Robotics - What you need?
2. ✅ Articulated Robotics - Networking
3. ✅ Articulated Robotics - Installing ROS
4. ✅ Articulated Robotics - ROS Overview
5. ✅ Articulated Robotics - Packages

### Planned Tutorials
- [ ] Articulated Robotics - The Transform System (tf2)
- [ ] Articulated Robotics - Describing robots with URDF
- [ ] Articulated Robotics - Simulating with Gazebo
- [ ] Nav2 official tutorials
- [ ] SLAM Toolbox tutorials

---

## 💡 Ideas and Experiments

- Battery life testing with all sensors active
- WiFi vs Ethernet performance comparison
- ROS bag recording for offline analysis
- Web interface for remote monitoring
- LED status indicators
- Audio feedback (speaker/buzzer)

---

## Notes

- Prioritize safety features before closed-loop motor control
- Test each sensor individually before full system integration
- Document all wiring, pin assignments, and configurations
- Keep URDF and TF tree consistent across all nodes
