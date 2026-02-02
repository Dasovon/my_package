# Hardware Requirements

## Required Hardware

### Computing Platform
- **Raspberry Pi 4** (4GB+ RAM recommended)
- **Operating System:** Ubuntu 22.04 LTS (64-bit)
- **ROS 2:** Humble

### Motor Control
- **Motor Driver:** L298N dual H-bridge
- **Motors:** DG01D-E with Hall effect encoders (1:48 gear ratio)
- **Power:** 6-9V battery for motors, separate 5V for Raspberry Pi

### Sensors
- **RPLIDAR A1:** 2D laser scanner (USB connection)
- **BNO055 IMU:** 9-DOF inertial measurement unit (I2C, address 0x28)
- **Intel RealSense D435:** RGB-D camera (USB 3.0)

## Software Dependencies

### System-Level Dependencies
```bash
# Required for motor control
sudo apt install python3-rpi.gpio

# Or install via pip
pip3 install RPi.GPIO --break-system-packages

```## Machine-Specific Dependencies

### Robot (Raspberry Pi)
The robot hardware needs:
- Motor control: `RPi.GPIO`
- Sensor drivers: `rplidar_ros`
- Core ROS 2: `rclpy`, message packages, `tf2_ros`
- Robot description: `xacro`, `robot_state_publisher`

**NOT needed on robot:**
- `gazebo_ros` (simulation - only on dev machine)
- `rviz2` (visualization - only on dev machine)

### Development Machine
The dev machine needs everything, including:
- All robot dependencies (for building/testing)
- `gazebo_ros` for simulation
- `rviz2` for visualization

**Install simulation/viz on dev machine:**
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-rviz2
```

### ROS 2 Dependencies
All ROS 2 dependencies are declared in `package.xml` and can be installed with:
```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

## GPIO Requirements

The `motor_controller` node requires direct GPIO access and will only run on Raspberry Pi hardware. It will fail gracefully with an error message on other platforms.

**GPIO Pins Used:**
- Motor A Enable: GPIO 17
- Motor A Direction: GPIO 27, GPIO 22
- Motor A Encoder: GPIO 23, GPIO 24
- Motor B Enable: GPIO 13
- Motor B Direction: GPIO 19, GPIO 26
- Motor B Encoder: GPIO 25, GPIO 5

## Development Without Hardware

For development on non-Raspberry Pi systems:
1. Use simulation (`launch_sim.launch.py` for Gazebo)
2. Test URDF and visualization in RViz
3. Develop launch files and configuration
4. Deploy to hardware for motor/sensor testing

## Troubleshooting

### "RPi.GPIO not available" Error
- **Solution:** Install RPi.GPIO or run on Raspberry Pi hardware
- **Check:** `python3 -c "import RPi.GPIO; print('GPIO OK')"`

### Permission Denied on GPIO Access
- **Solution:** Add user to `gpio` group: `sudo usermod -a -G gpio $USER`
- **Reboot required** after adding to group