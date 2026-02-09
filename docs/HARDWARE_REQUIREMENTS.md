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

**GPIO Pins Used (BCM numbers, configurable via ROS parameters):**

Motor A (Left Wheel):
- Enable (PWM): BCM 17 / physical pin 11 (`motor_enable_a_pin`)
- Direction IN1: BCM 27 / physical pin 13 (`motor_in1_pin`)
- Direction IN2: BCM 22 / physical pin 15 (`motor_in2_pin`)
- Encoder H1: BCM 23 / physical pin 16 (`encoder_left_a_pin`)
- Encoder H2: BCM 24 / physical pin 18 (`encoder_left_b_pin`)

Motor B (Right Wheel):
- Enable (PWM): BCM 13 / physical pin 33 (`motor_enable_b_pin`)
- Direction IN3: BCM 19 / physical pin 35 (`motor_in3_pin`)
- Direction IN4: BCM 26 / physical pin 37 (`motor_in4_pin`)
- Encoder H1: BCM 25 / physical pin 22 (`encoder_right_a_pin`)
- Encoder H2: BCM 5 / physical pin 29 (`encoder_right_b_pin`)

Encoder configuration:
- `encoder_ticks_per_rev`: 288 (3 pulses × 2 edges × 48:1 gear ratio)
- `encoder_left_inverted`: True (compensates for mirrored motor mount)
- `encoder_right_inverted`: False
- `encoder_bouncetime_ms`: 1

All pin assignments are declared as ROS 2 parameters in `motor_controller.py` and can be overridden via `config/motor_controller.yaml`.

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