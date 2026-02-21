# Hoverbot - ROS 2 Mobile Robot Project

**Status:** Base ROS 2 system configured ✅ | Motor control implemented ✅ | Sensors integrated ✅ | SLAM working ✅ | Navigation planned 📋

A battery-powered mobile robot that currently runs on a small DC gearmotor chassis with L298N control and RPLIDAR + wheel encoders, before moving to a hoverboard drive base. Running ROS 2 Humble on Ubuntu 22.04.

---

## Hardware

- **Robot Computer:** Raspberry Pi 4/5 (hostname: `hoverbot`)
- **Development PC:** Ubuntu 22.04 desktop (hostname: `dev`)
- **Drive System (current):** Small DC gearmotor chassis + L298N
- **Drive System (next):** Hoverboard motor controller (planned)
- **Sensors:**
  - RPLIDAR A1 (2D lidar) - *Integrated*
  - Wheel encoders (Hall effect, 288 ticks/rev) - *Integrated*
  - BNO055 IMU - *Integrated*
  - Intel RealSense D435i depth camera - *Planned*

See [docs/HARDWARE.md](docs/HARDWARE.md) for complete bill of materials and specifications.

---

## System Status

### ✅ Completed
- [x] Ubuntu 22.04 + ROS 2 Humble on both machines
- [x] Network configuration (hostname resolution, SSH, domain ID)
- [x] Cross-machine ROS 2 communication verified
- [x] Workspace setup (`dev_ws`, `robot_ws`)
- [x] Package structure with launch files and config
- [x] Git workflow for multi-machine development
- [x] TF tree and URDF robot description
- [x] L298N motor controller node with encoder support
- [x] Keyboard teleoperation node (WASD)
- [x] Wheel base calibration (0.236m effective)
- [x] BNO055 IMU integration (I2C, NDOF mode, 100Hz)
- [x] EKF sensor fusion (wheel odom + IMU)
- [x] RPLIDAR lidar watchdog (auto motor stop/start, auto USB power cycle)
- [x] SLAM mapping working (slam_toolbox, tuned for quality)
- [x] Gazebo simulation setup

### 🚧 In Progress
- [ ] Save and reuse a complete room map
- [ ] IMU calibration (BNO055 NDOF self-cal)

### 📋 Planned
- [ ] Save and reuse maps
- [ ] Nav2 autonomous navigation
- [ ] RealSense camera integration
- [ ] Hoverboard motor migration

See [docs/TODO.md](docs/TODO.md) for detailed roadmap.

---

## Quick Start

### Prerequisites
- Two Ubuntu 22.04 machines on same network
- ROS 2 Humble installed
- Git and SSH configured

### Clone and Build
**On development PC:**
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone git@github.com:Dasovon/my_package.git my_robot_bringup
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

**On robot computer:**
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone git@github.com:Dasovon/my_package.git my_package
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### Test Cross-Machine Communication
**On robot (hoverbot):**
```bash
ros2 launch my_robot_bringup talker.launch.py
```

**On dev PC:**
```bash
ros2 launch my_robot_bringup listener.launch.py
```

You should see "Hello World" messages flowing from hoverbot to dev.

---

## Documentation

- **[HARDWARE.md](docs/HARDWARE.md)** - Hardware specs, wiring, GPIO pins, motor config, BOM
- **[GUIDE.md](docs/GUIDE.md)** - Setup, daily operations, and troubleshooting
- **[TODO.md](docs/TODO.md)** - Project roadmap and next steps

---

## Repository Structure

```
my_robot_bringup/
├── README.md                    # This file
├── package.xml                  # ROS 2 package metadata
├── CMakeLists.txt               # Build configuration
├── LICENSE                      # MIT License
├── config/                      # Parameter files
│   ├── motor_controller.yaml    # Motor controller parameters
│   └── rplidar.yaml             # RPLIDAR configuration
├── launch/                      # Launch files
│   ├── full_bringup.launch.py   # Full robot bringup (RSP + motors + lidar)
│   ├── motor_control.launch.py  # Motor controller bringup
│   ├── rplidar.launch.py        # RPLIDAR bringup
│   ├── rsp.launch.py            # Robot state publisher
│   ├── teleop_keyboard.launch.py # Keyboard teleoperation
│   ├── launch_sim.launch.py     # Gazebo simulation
│   ├── talker.launch.py         # Demo publisher node
│   ├── listener.launch.py       # Demo subscriber node
│   └── service_example.launch.py # Demo service node
├── my_robot_bringup/            # Python nodes
│   ├── motor_controller.py      # L298N motor controller node
│   ├── teleop_keyboard.py       # Keyboard teleoperation node
│   └── lidar_watchdog.py        # RPLIDAR motor stop/start + USB power cycle
├── urdf/                        # Robot description files
│   ├── robot.urdf.xacro         # Main robot URDF
│   ├── robot_core.xacro         # Core robot structure
│   └── lidar.xacro              # LIDAR mount description
├── worlds/                      # Gazebo world files
│   ├── empty.world              # Empty simulation world
│   └── obstacles.world          # World with obstacles
└── docs/                        # Documentation
    ├── HARDWARE.md              # Hardware specs, wiring, motor config
    ├── GUIDE.md                 # Setup, operations, troubleshooting
    └── TODO.md                  # Project roadmap
```

---

## Network Configuration

- **Development PC:** `dev` (set per network)
- **Robot Computer:** `hoverbot` (192.168.86.33)
- **ROS Domain ID:** 0 (default — do NOT set in .bashrc)
- **Connection:** Passwordless SSH configured

Both machines on same subnet (192.168.86.0/24) with hostname resolution via `/etc/hosts`.

---

## Tutorials Completed

Based on [Articulated Robotics](https://articulatedrobotics.xyz/) tutorial series:

1. ✅ [What do you need? (Prerequisites)](https://beta.articulatedrobotics.xyz/tutorials/ready-for-ros/what-you-need-for-ros)
2. ✅ [Networking](https://beta.articulatedrobotics.xyz/tutorials/ready-for-ros/networking)
3. ✅ [Installing ROS](https://beta.articulatedrobotics.xyz/tutorials/ready-for-ros/installing-ros)
4. ✅ [ROS Overview](https://beta.articulatedrobotics.xyz/tutorials/ready-for-ros/ros-overview)
5. ✅ [Packages](https://beta.articulatedrobotics.xyz/tutorials/ready-for-ros/packages)
6. 🚧 [The Transform System (tf2)](https://beta.articulatedrobotics.xyz/tutorials/ready-for-ros/tf) - *Via RPLIDAR integration*

---

## Contributing

This is a personal learning project, but feedback and suggestions are welcome via GitHub issues.

---

## License

MIT License - See LICENSE file for details.

---

## Contact

**Maintainer:** Ryan  
**Email:** trickey19@gmail.com  
**GitHub:** https://github.com/Dasovon/my_package
