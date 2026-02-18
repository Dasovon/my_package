# Hoverbot Hardware Reference

Complete hardware specifications, wiring, motor configuration, and software dependencies.

---

## Robot Physical Specifications

### Chassis (URDF)
- **Base length:** 0.179 m
- **Base width:** 0.165 m
- **Base height:** 0.022 m
- **Base mass (URDF placeholder):** 5.0 kg

### Wheels
- **Wheel radius:** 0.032375 m
- **Wheel diameter:** 0.06475 m
- **Wheel width:** 0.058 m
- **Wheel base (physical center-to-center):** 0.165 m
- **Wheel base (effective, calibrated):** 0.236 m — use this in software
- **Wheel circumference:** 0.2034 m

### Encoders (DG01D-E)
- **Resolution:** 288 ticks per wheel revolution (3 pulses x 2 edges x 48:1 gear ratio)
- **Distance per tick:** 0.706 mm
- **Decoding method:** Interrupt-based quadrature via `GPIO.add_event_detect()`

---

## Computing Hardware

### Development PC ("dev")
- **OS:** Ubuntu 22.04 LTS Desktop (x86_64)
- **RAM:** 8GB+ recommended
- **Storage:** 50GB+ free space
- **Hostname:** dev
- **ROS install:** `ros-humble-desktop`
- **Workspace:** `~/dev_ws`

### Robot Computer ("hoverbot")
- **Model:** Raspberry Pi 4 or 5
- **RAM:** 4GB minimum, 8GB recommended
- **OS:** Ubuntu 22.04 LTS Server (ARM64)
- **Storage:** 32GB+ microSD card (Class 10, A2 rated)
- **Power:** 5V 3A USB-C
- **Hostname:** hoverbot
- **IP Address:** 192.168.86.33
- **ROS install:** `ros-humble-ros-base`
- **Workspace:** `~/robot_ws`

### Network
- **Subnet:** 192.168.86.0/24
- **ROS_DOMAIN_ID:** 0 (default — do NOT set it in .bashrc)
- **Router:** 802.11ac or better (2.4GHz and/or 5GHz)

---

## Sensors

### RPLIDAR A1 - 2D Laser Scanner
- **Status:** Integrated
- **Range:** 0.15m - 12m
- **Scan Mode:** Standard (2kHz sample rate, 10Hz scan rate)
- **Angular Resolution:** ~1 degree
- **Interface:** USB → CP2102 UART adapter
- **Serial Port:** `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` (ALWAYS use by-id path — ttyUSBx number changes on every power cycle)
- **ROS Driver:** `rplidar_ros`
- **Frame ID:** `laser_frame`
- **URDF orientation:** `rpy="0 0 ${pi}"` — cable faces back, 0° points away from cable, pi rotation aligns it forward. **Do not change this.**
- **Known issue:** USB power instability causes periodic crashes. Fix: powered USB hub. Workaround: `respawn=True` in rplidar.launch.py.

### BNO055 - 9-Axis IMU
- **Status:** Integrated (2026-02-17)
- **Sensors:** Accelerometer, gyroscope, magnetometer (9-DOF)
- **Interface:** I2C, bus 1, address 0x28
- **Operation Mode:** NDOF (0x0C) — full sensor fusion
- **Update Rate:** 100Hz
- **ROS Driver:** `ros-humble-bno055`
- **Frame ID:** `imu_link`
- **Wiring:** VIN→3.3V, GND→GND, SDA→GPIO2 (pin 3), SCL→GPIO3 (pin 5), ADR floating
- **Static TF:** `base_link` → `imu_link` at origin (0,0,0), identity rotation
- **Config:** `config/bno055.yaml`
- **Topics:** `/imu/data` (fused), `/imu/imu_raw`, `/imu/mag`, `/imu/calib_status`

### Intel RealSense D435i - Depth Camera
- **Status:** Planned
- **Depth Range:** 0.3m - 3m (typical), up to 10m
- **Resolution:** 1280x720 @ 30fps (depth), 1920x1080 @ 30fps (RGB)
- **Interface:** USB 3.0
- **ROS Driver:** `realsense-ros`

---

## Motor System (L298N + DG01D-E)

### Current Drive Platform
- **Motors:** DG01D-E DC gearmotors (6V nominal, 3-9V range, 1:48 gear ratio)
- **Driver:** L298N dual H-bridge
- **Motor power:** 6-9V battery
- **Logic power:** 5V from Pi (via L298N onboard regulator)
- **Common ground:** Battery GND connected to Pi GND

### L298N Limitations
- **Voltage drop:** ~2V (9V battery -> ~7V to motors)
- **Current limit:** 2A per channel (marginal for stall)
- **Efficiency:** ~60-70%

### Motor Direction Calibration

| Motor | Code Configuration |
|-------|-------------------|
| Motor A (Left) | **INVERTED** -- `set_motor_a()` has IN1/IN2 swapped in code |
| Motor B (Right) | **NORMAL** -- IN3/IN4 standard |

Physical motors are wired opposite each other. Software inversion compensates without rewiring.

### Critical Parameters

**Minimum Duty Cycle: 80%** -- DG01D-E motors require high breakaway torque. Below 80%, motors won't start reliably.

**Velocity Mapping:**
```python
# velocity_fraction = abs(velocity) / max_speed
# duty = min_duty + (velocity_fraction * (max_duty - min_duty))
# Any non-zero velocity maps to minimum 80% duty cycle
```

**Ramping:** 10% duty cycle change per iteration at 50 Hz control loop (0.16s to reach 80% from 0%).

---

## GPIO Pin Mapping

All pins are declared as ROS 2 parameters in `motor_controller.py` (with physical pin defaults) and overridden via `config/motor_controller.yaml` with BCM GPIO numbers. The code uses `GPIO.setmode(GPIO.BCM)`.

### Motor A (Left Wheel)

| Function | Physical Pin | BCM GPIO | ROS Parameter | L298N Terminal |
|----------|-------------|----------|---------------|----------------|
| Enable (PWM) | 11 | GPIO 17 | `motor_enable_a_pin` | ENA |
| Direction 1 | 13 | GPIO 27 | `motor_in1_pin` | IN1 |
| Direction 2 | 15 | GPIO 22 | `motor_in2_pin` | IN2 |

### Motor B (Right Wheel)

| Function | Physical Pin | BCM GPIO | ROS Parameter | L298N Terminal |
|----------|-------------|----------|---------------|----------------|
| Enable (PWM) | 33 | GPIO 13 | `motor_enable_b_pin` | ENB |
| Direction 1 | 35 | GPIO 19 | `motor_in3_pin` | IN3 |
| Direction 2 | 37 | GPIO 26 | `motor_in4_pin` | IN4 |

### Encoder Inputs (Interrupt-based Quadrature)

Encoders use `GPIO.add_event_detect()` on both channels for accurate quadrature decoding with a state machine lookup table.

| Function | Physical Pin | BCM GPIO | ROS Parameter |
|----------|-------------|----------|---------------|
| Left Encoder H1 | 16 | GPIO 23 | `encoder_left_a_pin` |
| Left Encoder H2 | 18 | GPIO 24 | `encoder_left_b_pin` |
| Right Encoder H1 | 22 | GPIO 25 | `encoder_right_a_pin` |
| Right Encoder H2 | 29 | GPIO 5 | `encoder_right_b_pin` |

### Encoder Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `encoder_ticks_per_rev` | 288 | 3 pulses x 2 edges x 48:1 gear ratio |
| `encoder_left_inverted` | True | Compensates for mirrored left motor mount |
| `encoder_right_inverted` | False | Normal direction |
| `encoder_bouncetime_ms` | 0 | Disabled — hall effect sensors don't need debounce |

### Encoder Power
- **VCC:** 3.3V — Physical pins 1 or 17
- **GND:** Any ground pin
- **NOTE:** Right encoder VCC (pin 1/17) and signal A (GPIO 25, pin 22) were found disconnected 2026-02-17 causing right wheel to report 0 ticks. Always verify encoder wiring if one wheel shows constant 0 in /joint_states.

---

## ROS 2 Motor Configuration

### Full Parameter Set (`config/motor_controller.yaml`)
```yaml
motor_controller:
  ros__parameters:
    wheel_base: 0.236              # Effective wheel base (calibrated)
    wheel_diameter: 0.06475        # Wheel diameter (m)
    encoder_ticks_per_rev: 288     # 3 pulses x 2 edges x 48:1 gear
    max_speed: 0.5                 # Maximum linear velocity (m/s)
    max_angular_speed: 2.0         # Maximum angular velocity (rad/s)
    min_duty_cycle: 80             # Minimum PWM for starting
    max_duty_cycle: 100            # Maximum PWM
    pwm_frequency: 1000            # PWM frequency (Hz)
    max_tick_delta: 1200           # Encoder tick delta clamp
    odom_frame: "odom"
    base_frame: "base_footprint"
    encoder_left_inverted: true
    encoder_right_inverted: false
    encoder_bouncetime_ms: 1
    enable_debug_logging: false
    # Pin assignments (BCM GPIO numbers)
    motor_enable_a_pin: 17
    motor_in1_pin: 27
    motor_in2_pin: 22
    motor_enable_b_pin: 13
    motor_in3_pin: 19
    motor_in4_pin: 26
    encoder_left_a_pin: 23
    encoder_left_b_pin: 24
    encoder_right_a_pin: 25
    encoder_right_b_pin: 5
```

### Safety Features
- **Watchdog timeout:** 1 second (auto-stop if no `/cmd_vel` commands)
- **Emergency stop:** Ctrl+C triggers GPIO cleanup
- **PWM ramping:** Prevents sudden motor jerks
- **Velocity clamping:** Within configured max limits
- **Encoder tick clamping:** Rejects noise spikes > `max_tick_delta`
- **GPIO validation:** Checks `/dev/gpiomem` or `/dev/mem` access before setup

### ROS 2 Topics

| Topic | Direction | Type | Rate |
|-------|-----------|------|------|
| `/cmd_vel` | Subscribe | `geometry_msgs/msg/Twist` | - |
| `/odom` | Publish | `nav_msgs/msg/Odometry` | 50 Hz |
| `/joint_states` | Publish | `sensor_msgs/msg/JointState` | 50 Hz |
| `/tf` | Publish | `tf2_msgs/msg/TFMessage` | 50 Hz |

TF tree: `odom` -> `base_footprint` -> `base_link` -> sensors

---

## Differential Drive Kinematics

**Wheel Velocities:**
```python
v_left  = linear.x - (angular.z * wheel_base / 2.0)
v_right = linear.x + (angular.z * wheel_base / 2.0)
```

| Command | linear.x | angular.z | v_left | v_right | Result |
|---------|----------|-----------|--------|---------|--------|
| Forward | 0.2 | 0.0 | 0.2 | 0.2 | Straight forward |
| Backward | -0.2 | 0.0 | -0.2 | -0.2 | Straight backward |
| Rotate Left | 0.0 | 0.8 | -0.072 | 0.072 | Spin counter-clockwise |
| Arc Left | 0.2 | 0.3 | 0.173 | 0.227 | Curved forward-left |

---

## Power System (Planned - Hoverboard Platform)

### Battery
- **Type:** Li-ion or LiFePO4, 36V nominal (10S)
- **Capacity:** 10Ah+ recommended
- **BMS:** Required

### Voltage Regulation
- **36V -> 5V:** Buck converter (3A+ for Pi + sensors)
- **36V -> 12V:** Optional (accessories)
- **Fuses:** Inline for all power branches

### Power Distribution
- **Pi:** 5V 3A USB-C (from buck converter)
- **RPLIDAR:** 5V via USB
- **RealSense:** 5V via USB 3.0
- **IMU:** 3.3V/5V (from Pi GPIO/I2C)
- **Motors:** 36V direct from battery

---

## Software Dependencies

### On Robot (Raspberry Pi)
```bash
sudo apt install python3-rpi.gpio
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-bno055
```
- Motor control: `RPi.GPIO`
- Sensor drivers: `rplidar_ros`, `bno055`
- Sensor fusion: `robot_localization` (EKF)
- Core ROS 2: `rclpy`, message packages, `tf2_ros`
- Robot description: `xacro`, `robot_state_publisher`

### On Dev Machine (additionally)
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-rviz2
sudo apt install ros-humble-slam-toolbox
```

### All ROS 2 Dependencies
```bash
cd ~/robot_ws  # or ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Safety Considerations

### Electrical
- 36V battery requires proper BMS and handling
- Reverse polarity protection on all power connections
- Inline fuses to prevent overcurrent damage
- Proper wire gauge for current loads

### Mechanical
- Emergency stop button (physical or software)
- Current limiting on motors
- Test bench setup before full assembly (wheels off ground)

### Software
- Velocity limits in code
- Watchdog timer for motor commands
- Safe startup/shutdown procedures

---

## Migration Path to Hoverboard Motors

**Hardware changes:**
- Remove L298N, connect hoverboard motor controller via UART
- Configure serial protocol

**Software changes:**
- Replace GPIO motor control with serial commands in `motor_controller.py`
- Add hall sensor encoder feedback reading
- Update velocity mapping for hoverboard motor characteristics

**Parameters to update:**
- `max_speed` (likely higher)
- `min_duty_cycle` (may not need 80%)
- `wheel_base` (measure actual distance)

**Keep unchanged:**
- Differential drive kinematics math
- ROS 2 node structure and `/cmd_vel` interface
- Keyboard teleop

---

## Estimated Costs (USD)

| Item | Qty | Price | Total |
|------|-----|-------|-------|
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

---

## Datasheets and References

- [L298N Datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [DG01D-E Motor Specs](https://cdn.sparkfun.com/assets/8/3/b/e/4/DS-16413-DG01D-E_Motor_with_Encoder.pdf)
- [RPLIDAR A1 Manual](https://www.slamtec.com/en/Lidar/A1Spec)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [RPi.GPIO Documentation](https://pypi.org/project/RPi.GPIO/)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
