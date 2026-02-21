# Hoverbot Setup, Operations, and Troubleshooting Guide

---

## Part 1: Initial Setup (One-Time)

### 1.1 Operating System Setup

**Development PC:**
1. Install Ubuntu 22.04 Desktop, set hostname to `dev`
2. Update: `sudo apt update && sudo apt upgrade -y`
3. Install tools: `sudo apt install -y git curl wget nano vim openssh-server`

**Robot Computer (Raspberry Pi):**
1. Flash Ubuntu 22.04 Server using Raspberry Pi Imager (set hostname `hoverbot`, enable SSH, configure WiFi)
2. SSH in: `ssh ubuntu@<PI_IP_ADDRESS>` (default password: `ubuntu`)
3. Update: `sudo apt update && sudo apt upgrade -y`
4. Set hostname: `sudo hostnamectl set-hostname hoverbot && sudo reboot`

### 1.2 Network Configuration

**Find IP addresses:**
```bash
hostname -I  # Run on each machine
# dev example: 192.168.86.37
# hoverbot example: 192.168.86.33
```

**Configure `/etc/hosts` on dev:**
```
192.168.86.33 hoverbot
```

**Configure `/etc/hosts` on hoverbot:**
```
192.168.86.37 dev
```

**Verify:** `ping hoverbot` from dev, `ping dev` from hoverbot.

**Set up passwordless SSH (on dev):**
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
ssh-copy-id ryan@hoverbot
```

**Optional SSH config (`~/.ssh/config` on dev):**
```
Host hoverbot
    HostName hoverbot
    User ryan
    ForwardAgent yes
    ForwardX11 yes
```

### 1.3 ROS 2 Installation

**On both machines:**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
```

**On dev:** `sudo apt install ros-humble-desktop -y`
**On hoverbot:** `sudo apt install ros-humble-ros-base -y`

**On both:**
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

**Additional packages on dev:**
```bash
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rqt ros-humble-rqt-graph
```

**Additional packages on hoverbot:**
```bash
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher
```

**Configure `~/.bashrc` on dev:**
```bash
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
# DO NOT set ROS_DOMAIN_ID -- use default (0)
export ROS_LOCALHOST_ONLY=0
```

**Configure `~/.bashrc` on hoverbot:**
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
# DO NOT set ROS_DOMAIN_ID -- use default (0)
export ROS_LOCALHOST_ONLY=0
```

Apply: `source ~/.bashrc`

### 1.4 Workspace and Package Setup

**On dev:**
```bash
mkdir -p ~/dev_ws/src && cd ~/dev_ws
colcon build --symlink-install && source install/setup.bash
cd src
git clone git@github.com:Dasovon/my_package.git my_robot_bringup
cd ~/dev_ws && colcon build --symlink-install && source install/setup.bash
```

**On hoverbot:**
```bash
mkdir -p ~/robot_ws/src && cd ~/robot_ws
colcon build --symlink-install && source install/setup.bash
cd src
git clone git@github.com:Dasovon/my_package.git my_package
cd ~/robot_ws && colcon build --symlink-install && source install/setup.bash
```

**Note:** Package is named `my_robot_bringup` in `package.xml`, but directory names differ (dev: `my_robot_bringup`, hoverbot: `my_package`). ROS uses the package name, not the directory name.

### 1.5 Verify Setup

**On hoverbot:** `ros2 launch my_robot_bringup talker.launch.py`
**On dev:** `ros2 topic echo /chatter` -- should see "Hello World" messages.

**Post-setup checklist:**
- Ubuntu 22.04 on both machines
- ROS 2 Humble installed
- `ping hoverbot` works from dev
- Passwordless SSH configured
- `echo $ROS_DOMAIN_ID` shows blank or 0 on both (do NOT set it)
- Workspaces build successfully
- Cross-machine topic communication verified

---

## Part 2: Daily Operations

### Quick Reference

| | Robot (hoverbot) | Dev Machine |
|---|---|---|
| **Hardware** | Raspberry Pi 4 | Ubuntu 22.04 Desktop |
| **IP** | 192.168.86.33 | 192.168.86.37 |
| **Workspace** | `~/robot_ws` | `~/dev_ws` |
| **ROS install** | `ros-humble-ros-base` | `ros-humble-desktop` |
| **ROS_DOMAIN_ID** | 0 (default) | 0 (default) |

### 2.1 Power On and Connect

1. Insert microSD card (if removed)
2. Connect RPLIDAR to USB
3. Connect motor power (6-9V battery to L298N)
4. Connect Pi 5V power supply
5. Wait for boot (green LED settles)

```bash
ssh hoverbot
# If hostname doesn't resolve: ssh ryan@192.168.86.33
```

### 2.2 Pre-Flight Checks

```bash
# Verify ROS environment
echo $ROS_DOMAIN_ID        # Expected: blank or 0 (do NOT set this)
echo $ROS_LOCALHOST_ONLY    # Expected: 0

# Pull latest code
cd ~/robot_ws/src/my_package
git pull origin main

# Build
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

If dependencies are missing: `rosdep install --from-paths src --ignore-src -r -y`

**Hardware checks (on hoverbot):**
```bash
# GPIO access
python3 -c "import RPi.GPIO; print('GPIO OK')"

# RPLIDAR device (use by-id path, NOT ttyUSBx)
ls /dev/serial/by-id/

# IMU
i2cdetect -y 1  # should show device at 0x28
```

If GPIO shows `Permission denied`: `sudo usermod -a -G gpio $USER` then log out/in.

### 2.3 Launch the Robot

```bash
ros2 launch my_robot_bringup full_bringup.launch.py
```

This starts:
1. **Robot State Publisher** -- URDF + TF tree
2. **Motor Controller** -- `/cmd_vel` → `/odom` + `odom→base_footprint` TF
3. **RPLIDAR** -- `/scan` (with auto-respawn at 7s delay)
4. **BNO055 IMU** -- `/imu/data`
5. **Static TF** -- `base_link` → `imu_link`
6. **EKF** -- fuses `/odom` + `/imu/data` → `/odometry/filtered`
7. **Lidar Watchdog** -- stops RPLIDAR motor when `/scan` has no subscribers; auto power cycles USB on crash

If you see `RuntimeError: Failed to add edge detection`, another process has the GPIO pins. Stop other ROS/motor processes, wait a few seconds, and relaunch. If it persists: `sudo reboot`.

**IMPORTANT — always kill stale nodes before relaunching:**
```bash
ps aux | grep -E "rplidar_composition|motor_controller.py|bno055|ekf_node|robot_state_pub|static_transform|lidar_watchdog" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null
```
Stale processes from previous launches fight over GPIO pins and cause motors to stop responding.

**Verify nodes:**
```bash
ros2 node list
# Expected: /robot_state_publisher /motor_controller /rplidar_node /bno055 /imu_tf /ekf_node /lidar_watchdog
```

**Verify topics and rates:**
```bash
ros2 topic list
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom          # ~50 Hz
ros2 topic hz /imu/data      # ~100 Hz
ros2 topic hz /odometry/filtered  # ~30 Hz
```

Key topics:

| Topic | Source | Type |
|-------|--------|------|
| `/cmd_vel` | Teleop / Nav | `geometry_msgs/msg/Twist` |
| `/odom` | Motor controller | `nav_msgs/msg/Odometry` |
| `/odometry/filtered` | EKF | `nav_msgs/msg/Odometry` |
| `/scan` | RPLIDAR | `sensor_msgs/msg/LaserScan` |
| `/imu/data` | BNO055 | `sensor_msgs/msg/Imu` |
| `/tf` | Motor controller + RSP | `tf2_msgs/msg/TFMessage` |

**SLAM (run on dev machine):**
```bash
ros2 launch my_robot_bringup slam.launch.py
```
Then open rviz2, Fixed Frame: `map`, add Map (`/map`) and LaserScan (`/scan`).

### 2.4 Driving the Robot

**Keyboard teleop (from dev):**
```bash
ros2 launch my_robot_bringup teleop_keyboard.launch.py
```

| Key | Action | linear.x | angular.z |
|-----|--------|----------|-----------|
| W / Up | Forward | 0.3 m/s | 0.0 |
| S / Down | Backward | -0.3 m/s | 0.0 |
| A / Left | Turn Left | 0.0 | 0.8 rad/s |
| D / Right | Turn Right | 0.0 | -0.8 rad/s |
| Space | Stop | 0.0 | 0.0 |
| Q | Quit | - | - |

**Manual velocity commands:**
```bash
# Forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**Monitor odometry:**
```bash
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

**RViz visualization (dev):**
```bash
rviz2
```
Add displays: RobotModel (`/robot_description`), LaserScan (`/scan`), TF, Odometry (`/odom`).

### 2.5 Running Unit Tests

Tests validate motor controller math and don't require hardware (GPIO is mocked).

```bash
# Using colcon (from workspace root)
colcon test --packages-select my_robot_bringup
colcon test-result --verbose

# Or using pytest directly
python3 -m pytest test/test_motor_controller.py -v
```

| Test Area | Description |
|-----------|-------------|
| `velocity_to_duty()` | Maps velocity to PWM duty cycle |
| `clamp_tick_delta()` | Validates encoder tick deltas |
| `yaw_to_quaternion()` | Converts yaw to quaternion |
| Differential drive kinematics | Forward, turning, stationary |
| Odometry integration | Pose updates from encoder ticks |

### 2.6 Simulation (Dev Only)

```bash
# Launch Gazebo
ros2 launch my_robot_bringup launch_sim.launch.py

# Drive simulated robot (second terminal)
ros2 launch my_robot_bringup teleop_keyboard.launch.py
```

### 2.7 Shutdown

1. Press Ctrl+C in each terminal running a launch file
2. `sudo shutdown -h now`
3. Wait for green LED to stop flashing before disconnecting power
4. Disconnect battery from L298N

---

## Part 3: Troubleshooting

### Network and Connectivity

**Nodes not discovering each other:**
1. Check `echo $ROS_DOMAIN_ID` on both machines (should be blank or 0)
2. `ping hoverbot` / `ping dev`
3. `ros2 daemon stop && ros2 daemon start` on both
4. `sudo ufw status` -- should be inactive or allow multicast
5. Verify `/etc/hosts` entries

**Topics visible but no messages:**
1. `ros2 topic info /topic_name -v` -- check QoS compatibility
2. `ros2 topic info /chatter` -- verify publisher/subscriber counts
3. `ping -c 100 hoverbot` -- check latency <100ms, <5% loss

**SSH connection issues:**
1. `sudo systemctl status ssh` on hoverbot
2. Check `~/.ssh/` key permissions (private key: 600)
3. `ssh-copy-id ryan@hoverbot`

### Build and Package Issues

**colcon build fails:**
1. `rosdep install --from-paths src --ignore-src -r -y`
2. Clean build: `rm -rf build/ install/ log/ && colcon build --symlink-install`
3. Check `CMakeLists.txt` -- `install()` before `ament_package()`
4. Check `package.xml` -- all `exec_depend` installed

**Launch file not found:**
1. `source ~/dev_ws/install/setup.bash` (or `robot_ws`)
2. `ros2 pkg list | grep my_robot_bringup`
3. Verify install directive in `CMakeLists.txt`

### ROS 2 Configuration

**ROS_DOMAIN_ID causing cross-machine issues:**
Do NOT set `ROS_DOMAIN_ID` in `.bashrc` on either machine. Use the default (0).
If it's there, remove it: `nano ~/.bashrc` and delete the line.
Correct `.bashrc` order:
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=0
```

### Motor and Encoder Issues

**Motors don't start:**
1. Check battery voltage (6-9V)
2. Check min_duty_cycle in config (try 90%)
3. Verify GPIO connections (see [HARDWARE.md](HARDWARE.md#gpio-pin-mapping))
4. Test: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"`

**Wrong direction:**
Check Motor A inversion in `motor_controller.py` `set_motor_a()` -- IN1/IN2 should be swapped.

**Motors jerk or stutter:**
1. Battery voltage sagging
2. Loose connections
3. Try reducing PWM frequency to 100 Hz

**Encoders not counting:**
1. Check 5V power to encoder boards
2. Check GPIO pin connections (H1/H2)
3. Manually spin wheels and watch tick counts

**Odometry drifts wrong direction:**
1. Check encoder inversion parameters (`encoder_left_inverted`, `encoder_right_inverted`)
2. Verify quadrature decoding logic
3. Check `odom_frame` and `base_frame` parameters

### Raspberry Pi Issues

**WiFi disconnects after idle:**
```bash
sudo iwconfig wlan0 power off
# Make permanent in /etc/rc.local: /sbin/iwconfig wlan0 power off
```
Alternative: use Ethernet.

**SD card corruption:**
- Use A2-rated SD card
- Always `sudo shutdown -h now` before unplugging
- Backup: `sudo dd if=/dev/sdX of=~/hoverbot_backup.img bs=4M status=progress`

**GPIO permission denied:**
```bash
sudo usermod -a -G gpio $USER
# Log out and back in
```

### Git Issues

**Push/pull fails (permission denied):**
1. `ssh -T git@github.com` -- test authentication
2. Verify remote: `git remote -v` (should be `git@github.com:Dasovon/my_package.git`)
3. Switch to SSH: `git remote set-url origin git@github.com:Dasovon/my_package.git`

**Merge conflicts:**
```bash
git status           # View conflicted files
nano <filename>      # Resolve conflicts (remove <<<<<<< markers)
git add <filename> && git commit -m "Resolved merge conflict"
```

### Performance Issues

**High CPU on Pi:**
1. `top` -- check for >50% CPU processes
2. Reduce sensor rates (RPLIDAR, camera)
3. `sudo systemctl disable bluetooth avahi-daemon`

**Slow network:**
1. Use 5GHz WiFi instead of 2.4GHz
2. Consider Ethernet for reliability

### Diagnostic Commands

```bash
# Network
ping <hostname>
ifconfig
iwconfig

# ROS 2
ros2 node list
ros2 topic list
ros2 topic echo /topic
ros2 topic hz /topic
ros2 topic info /topic -v
ros2 daemon stop && ros2 daemon start

# System
df -h
free -h
top
dmesg | tail -50
```

---

## Getting Help

1. **GitHub Issues:** https://github.com/Dasovon/my_package/issues
2. **ROS Answers:** https://answers.ros.org/
3. **Articulated Robotics Discord**

---

## References

- [Articulated Robotics Tutorials](https://articulatedrobotics.xyz/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Ubuntu 22.04 Server Guide](https://ubuntu.com/server/docs)
