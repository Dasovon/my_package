# Hardware Bill of Materials

Complete hardware specifications and bill of materials for the Hoverbot mobile robot.
Current development runs on a small DC gearmotor chassis with L298N motor control before the hoverboard drive upgrade.

---

## Computing Hardware

### Development PC ("dev")
- **Type:** Desktop/Laptop
- **OS:** Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- **CPU:** x86_64 (Intel/AMD)
- **RAM:** 8GB+ recommended
- **Storage:** 50GB+ free space
- **Network:** WiFi or Ethernet
- **Hostname:** dev
- **IP Address:** (set per network)

### Robot Computer ("hoverbot")
- **Model:** Raspberry Pi 4 or 5
- **RAM:** 4GB minimum, 8GB recommended
- **OS:** Ubuntu 22.04.5 LTS Server (ARM64)
- **Storage:** 32GB+ microSD card (Class 10, A2 rated recommended)
- **Power:** 5V 3A USB-C power supply
- **Network:** WiFi or Ethernet
- **Hostname:** hoverbot
- **IP Address:** 192.168.86.33
- **Disk Space:** 49GB free (after OS and ROS installation)

---

## Sensors

### RPLIDAR A1 - 2D Laser Scanner
- **Status:** Installed, integration next
- **Type:** 360° 2D laser range scanner
- **Range:** 0.15m - 12m
- **Sample Rate:** 8000 samples/second
- **Scan Rate:** 5.5 Hz (typical)
- **Angular Resolution:** 1°
- **Interface:** USB (via UART adapter)
- **Power:** 5V via USB (no separate power needed)
- **ROS Driver:** `rplidar_ros` (ROS 2 package)
- **Frame ID:** `laser` or `laser_frame`

### BNO055 - 9-Axis IMU
- **Status:** Planned
- **Type:** 9-DOF Absolute Orientation Sensor
- **Sensors:** 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer
- **Output:** Quaternion, Euler angles, linear acceleration, gravity vector
- **Interface:** I2C or UART
- **Update Rate:** Up to 100Hz
- **ROS Driver:** `bno055` package or custom node
- **Frame ID:** `imu_link`

### Intel RealSense D435i - Depth Camera
- **Status:** Planned
- **Type:** Stereo depth camera with IMU
- **Depth Range:** 0.3m - 3m (typical), up to 10m max
- **Resolution:** 1280x720 @ 30fps (depth), 1920x1080 @ 30fps (RGB)
- **Field of View:** 87° × 58° (depth), 69° × 42° (RGB)
- **Interface:** USB 3.0 (USB-C connector)
- **Power:** 5V via USB (2.5W typical)
- **ROS Driver:** `realsense-ros` (official ROS 2 package)
- **Frame IDs:** `camera_link`, `camera_depth_frame`, `camera_color_frame`

---

## Actuators

### Small DC Gearmotor Chassis (Current)
- **Status:** In use
- **Motor Type:** DG01D-E with encoders
- **Motor Driver:** L298N dual H-bridge
- **Wheel Diameter:** 65mm
- **Wheelbase:** 165mm

### Hoverboard Motor Controller (Next Platform)
- **Status:** Planned, not yet integrated
- **Type:** Dual brushless DC motor controller (from hoverboard)
- **Interface:** UART (bidirectional control)
- **Voltage:** 36V nominal battery input
- **Current:** 15A per motor (typical)
- **Safety Features:** Current limiting, voltage monitoring, emergency stop
- **Control Mode:** Velocity commands via `/cmd_vel` topic
- **Odometry:** Wheel encoders for `/odom` topic

---

## Power System

### Battery (Planned)
- **Type:** Lithium-ion or LiFePO4
- **Voltage:** 36V nominal (10S Li-ion)
- **Capacity:** 10Ah+ recommended
- **BMS:** Built-in battery management system required
- **Connectors:** XT60 or Anderson Powerpole

### Power Distribution
- **Robot Computer:** 5V 3A USB-C (from buck converter)
- **RPLIDAR:** 5V via USB (from Pi or separate regulator)
- **RealSense:** 5V via USB (from Pi USB 3.0 port)
- **IMU:** 3.3V/5V (from Pi GPIO or I2C)
- **Motors:** 36V direct from battery

### Voltage Regulation (Planned)
- **36V → 5V:** Buck converter (3A+ rating for Pi + sensors)
- **36V → 12V:** Optional (for accessories)
- **Fuses:** Inline fuses for all power branches

---

## Mechanical Hardware

### Chassis
- **Type:** Custom CAD design
- **Material:** TBD (acrylic, aluminum, 3D printed parts)
- **Mounting:** RPLIDAR on top center, cameras at front, IMU near center of mass

### Drive Base
- **Type:** Hoverboard drive platform (differential drive)
- **Wheels:** Two driven wheels (hoverboard motors)
- **Casters:** Passive caster(s) for stability
- **Wheel Diameter:** ~6.5 inches (typical hoverboard wheel)
- **Wheelbase:** ~450mm (typical hoverboard width)

---

## Network Hardware

### Router/Access Point
- **Type:** Home WiFi router or dedicated travel router
- **Standard:** 802.11ac or better
- **Band:** 2.4GHz and/or 5GHz
- **Network:** 192.168.86.0/24 (current home network)

### Cables
- **SSH/Development:** Ethernet cable (optional, using WiFi currently)
- **USB:** USB-A to Micro-USB (RPLIDAR), USB-C (RealSense)

---

## Development Tools

### Software
- **ROS Version:** ROS 2 Humble
- **IDE:** VS Code with Remote-SSH extension
- **Version Control:** Git + GitHub
- **Build Tool:** colcon
- **Visualization:** RViz2

### Accessories
- **Keyboard/Mouse:** For robot setup (can be removed after SSH configured)
- **Monitor:** HDMI (temporary, for Pi setup)
- **SD Card Reader:** For flashing Pi OS

---

## Estimated Costs (USD)

| Item | Quantity | Unit Price | Total |
|------|----------|------------|-------|
| Raspberry Pi 4/5 (4GB) | 1 | $55-75 | $55-75 |
| MicroSD Card (32GB+) | 1 | $10-15 | $10-15 |
| RPLIDAR A1 | 1 | $99 | $99 |
| BNO055 IMU | 1 | $30-40 | $30-40 |
| RealSense D435i | 1 | $299 | $299 |
| Hoverboard (used) | 1 | $50-150 | $50-150 |
| Battery (36V 10Ah) | 1 | $100-200 | $100-200 |
| Buck Converters | 2 | $10-20 | $20-40 |
| Chassis Materials | - | $50-100 | $50-100 |
| Cables, connectors, misc | - | $30-50 | $30-50 |
| **Total** | | | **$743-1068** |

*Prices are estimates and may vary by supplier and location.*

---

## Datasheets and References

- **RPLIDAR A1:** [Slamtec RPLIDAR A1 Manual](https://www.slamtec.com/en/Lidar/A1Spec)
- **BNO055:** [Bosch BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- **RealSense D435i:** [Intel RealSense D435i Datasheet](https://www.intelrealsense.com/depth-camera-d435i/)
- **Raspberry Pi 4:** [Official Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)

---

## Safety Considerations

### Electrical
- ⚠️ **36V battery requires proper BMS and handling**
- ⚠️ **Reverse polarity protection on all power connections**
- ⚠️ **Inline fuses to prevent overcurrent damage**
- ⚠️ **Proper wire gauge for current loads**

### Mechanical
- ⚠️ **Emergency stop button (physical or software)**
- ⚠️ **Current limiting on motors to prevent damage**
- ⚠️ **Test bench setup before full assembly (wheels off ground)**

### Software
- ⚠️ **Velocity limits in code**
- ⚠️ **Watchdog timer for motor commands**
- ⚠️ **Safe startup/shutdown procedures**

---

## Future Additions (Wishlist)

- Lidar upgrade: RPLIDAR S2/S3 or Ouster OS0
- GPS module for outdoor navigation
- Additional cameras (stereo vision, fisheye)
- Gripper or manipulator arm
- LED status indicators
- Speaker for audio feedback
