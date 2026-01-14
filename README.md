# Hoverbot - ROS 2 Mobile Robot Project

**Status:** Base ROS 2 system configured ✅ | Sensor integration in progress 🚧

A battery-powered mobile robot built on a hoverboard drive base with RPLIDAR, BNO055 IMU, and Intel RealSense depth camera. Running ROS 2 Humble on Ubuntu 22.04.

---

## Hardware

- **Robot Computer:** Raspberry Pi 4/5 (hostname: `hoverbot`)
- **Development PC:** Ubuntu 22.04 desktop (hostname: `dev`)
- **Drive System:** Hoverboard motor controller (planned)
- **Sensors:**
  - RPLIDAR A1 (2D lidar) - *Integration in progress*
  - BNO055 IMU - *Planned*
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

### 🚧 In Progress
- [ ] RPLIDAR A1 integration
- [ ] TF tree and URDF
- [ ] RViz visualization

### 📋 Planned
- [ ] BNO055 IMU integration
- [ ] RealSense camera integration
- [ ] Motor controller bringup
- [ ] SLAM and navigation
- [ ] Autonomous behavior

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

- **[SETUP.md](docs/SETUP.md)** - Complete setup guide from scratch
- **[HARDWARE.md](docs/HARDWARE.md)** - Bill of materials and specifications
- **[TODO.md](docs/TODO.md)** - Project roadmap and next steps
- **[TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** - Common issues and solutions

---

## Repository Structure

```
my_robot_bringup/
├── README.md                    # This file
├── package.xml                  # ROS 2 package metadata
├── CMakeLists.txt              # Build configuration
├── config/                      # Parameter files
│   └── my_controllers.yaml
├── launch/                      # Launch files
│   ├── talker.launch.py
│   ├── listener.launch.py
│   └── service_example.launch.py
└── docs/                        # Documentation
    ├── SETUP.md
    ├── HARDWARE.md
    ├── TODO.md
    └── TROUBLESHOOTING.md
```

---

## Network Configuration

- **Development PC:** `dev` (192.168.86.37)
- **Robot Computer:** `hoverbot` (192.168.86.33)
- **ROS Domain ID:** 42
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
