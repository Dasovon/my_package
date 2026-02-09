# L298N Motor Control - Temporary Test Setup

**Status:** WORKING - Interim solution for motor testing  
**Date:** January 26, 2026  
**Platform:** Raspberry Pi 4 (hoverbot)  
**Motors:** DG01D-E (6V nominal, 3-9V range)  
**Future:** Will migrate to hoverboard motor platform

---

## Overview

Temporary motor control setup using L298N dual H-bridge driver to test ROS 2 differential drive integration before switching to hoverboard motors. This setup validates the motor control architecture and keyboard teleop workflow.

---

## Hardware Configuration

### L298N Dual H-Bridge Driver

**Power:**
- Motor power: 6-9V battery
- Logic power: 5V from Pi (via L298N onboard regulator)
- Common ground: Battery GND connected to Pi GND

**Limitations:**
- Voltage drop: ~2V (inefficient for low-voltage motors)
- Current limit: 2A per channel (marginal for motor stall current)
- Efficiency: ~60-70%

**Note:** L298N chosen for availability/testing only. Not optimal for DG01D-E motors.

---

## Pin Mapping (BCM GPIO Numbering)

All pins are declared as ROS 2 parameters in `motor_controller.py` and can be overridden via `config/motor_controller.yaml`. The code defaults use physical pin numbers; the YAML config provides BCM GPIO numbers used at runtime (`GPIO.setmode(GPIO.BCM)`).

### Motor A (Left Wheel)

| Function | Pi Physical Pin | BCM GPIO | ROS Parameter | L298N Terminal |
|----------|----------------|----------|---------------|----------------|
| Enable (PWM) | 11 | GPIO 17 | `motor_enable_a_pin` | ENA |
| Direction 1 | 13 | GPIO 27 | `motor_in1_pin` | IN1 |
| Direction 2 | 15 | GPIO 22 | `motor_in2_pin` | IN2 |

### Motor B (Right Wheel)

| Function | Pi Physical Pin | BCM GPIO | ROS Parameter | L298N Terminal |
|----------|----------------|----------|---------------|----------------|
| Enable (PWM) | 33 | GPIO 13 | `motor_enable_b_pin` | ENB |
| Direction 1 | 35 | GPIO 19 | `motor_in3_pin` | IN3 |
| Direction 2 | 37 | GPIO 26 | `motor_in4_pin` | IN4 |

### Encoder Inputs (Quadrature, interrupt-based)

| Function | Pi Physical Pin | BCM GPIO | ROS Parameter |
|----------|----------------|----------|---------------|
| Left Encoder H1 | 16 | GPIO 23 | `encoder_left_a_pin` |
| Left Encoder H2 | 18 | GPIO 24 | `encoder_left_b_pin` |
| Right Encoder H1 | 22 | GPIO 25 | `encoder_right_a_pin` |
| Right Encoder H2 | 29 | GPIO 5 | `encoder_right_b_pin` |

### Encoder Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `encoder_ticks_per_rev` | 288 | 3 pulses × 2 edges × 48:1 gear ratio |
| `encoder_left_inverted` | True | Compensates for mirrored motor mount |
| `encoder_right_inverted` | False | Normal direction |
| `encoder_bouncetime_ms` | 1 | GPIO edge detection debounce (ms) |

---

## Motor Calibration

**Direction Configuration:**

| Motor | Wiring | Code Configuration |
|-------|--------|-------------------|
| Motor A (Left) | Normal | **INVERTED** (IN1/IN2 swapped in code) |
| Motor B (Right) | Normal | **NORMAL** (IN3/IN4 standard) |

**Why Motor A is inverted:**
- Physical motors wired opposite to each other
- Compensated in software (`set_motor_a()` has reversed GPIO logic)
- Easier than rewiring hardware

**Verification:**
- `linear.x > 0` → Both wheels forward → Robot drives forward ✅
- `angular.z > 0` → Left backward, Right forward → Robot turns left ✅

---

## Critical Parameters

### Minimum Duty Cycle: 80%

**Why:** DG01D-E motors require high breakaway torque
- Below 80%: Motors won't start (static friction too high)
- At 80%: Reliable starting
- Manual push test at 30%: Motors maintain rotation but won't start

**Implemented in:** `motor_controller.py` → `velocity_to_duty()` function

### Velocity Mapping
```python
# Maps velocity (m/s) to duty cycle (%)
# velocity_fraction = abs(velocity) / max_speed
# duty = min_duty + (velocity_fraction * (max_duty - min_duty))

# Example:
# max_speed = 0.5 m/s
# velocity = 0.2 m/s
# velocity_fraction = 0.2 / 0.5 = 0.4
# duty = 80 + (0.4 * (100 - 80)) = 80 + 8 = 88%
```

**Key point:** Any non-zero velocity maps to minimum 80% duty cycle

### Ramping Rate: 10% per iteration

**Control loop:** 50 Hz (0.02 second interval)
**Ramp rate:** 10% duty cycle change per iteration
**Time to reach 80% from 0%:** ~8 iterations = 0.16 seconds

**Why ramping:**
- Smooth acceleration (motor-friendly)
- Reduces current spikes
- Prevents drivetrain shock

---

## ROS 2 Configuration

### Node: motor_controller

**Package:** `my_robot_bringup`  
**Executable:** `motor_controller.py`  
**Subscribes:** `/cmd_vel` (geometry_msgs/msg/Twist)

**Parameters (from `config/motor_controller.yaml`):**
```yaml
motor_controller:
  ros__parameters:
    wheel_base: 0.165              # Distance between wheels (m)
    wheel_diameter: 0.06475        # Wheel diameter (m)
    encoder_ticks_per_rev: 288     # 3 pulses × 2 edges × 48:1 gear
    max_speed: 0.5                 # Maximum linear velocity (m/s)
    max_angular_speed: 2.0         # Maximum angular velocity (rad/s)
    min_duty_cycle: 80             # Minimum PWM for starting
    max_duty_cycle: 100            # Maximum PWM
    pwm_frequency: 1000            # PWM frequency (Hz)
    max_tick_delta: 1200           # Encoder tick delta clamp
    encoder_left_inverted: true    # Left encoder direction inversion
    encoder_right_inverted: false  # Right encoder direction
    encoder_bouncetime_ms: 1       # Edge detection debounce (ms)
    enable_debug_logging: false    # Motor duty cycle debug output
```

**Safety Features:**
- Watchdog timeout: 1 second (auto-stop if no commands)
- Emergency stop: Ctrl+C triggers GPIO cleanup
- Soft ramping: Prevents motor shock

---

## Keyboard Teleop

**Node:** `teleop_keyboard.py`  
**Publishes:** `/cmd_vel` at 10 Hz

**Controls:**

| Key | Action | linear.x | angular.z |
|-----|--------|----------|-----------|
| W / ↑ | Forward | 0.3 m/s | 0.0 rad/s |
| S / ↓ | Backward | -0.3 m/s | 0.0 rad/s |
| A / ← | Turn Left | 0.0 m/s | 0.8 rad/s |
| D / → | Turn Right | 0.0 m/s | -0.8 rad/s |
| Space | Stop | 0.0 m/s | 0.0 rad/s |
| Q | Quit | - | - |

**Parameters:**
- `linear_speed: 0.3` (conservative for testing)
- `angular_speed: 0.8` (rotation velocity)

---

## Usage

### Launch Motor Controller

**Terminal 1 (hoverbot):**
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch my_robot_bringup motor_control.launch.py
```

### Launch Keyboard Teleop

**Terminal 2 (hoverbot):**
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 run my_robot_bringup teleop_keyboard.py
```

**Drive with W/A/S/D keys**

### Test with Manual Commands
```bash
# Forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

---

## Files Created

### Python Nodes
```
my_robot_bringup/
├── motor_controller.py      # Motor control node
└── teleop_keyboard.py        # Keyboard teleop node
```

### Configuration
```
config/
└── motor_controller.yaml     # Motor parameters
```

### Launch Files
```
launch/
└── motor_control.launch.py   # Launches motor_controller node
```

### CMakeLists.txt Updates
```cmake
# Install Python scripts
install(PROGRAMS
  my_robot_bringup/motor_controller.py
  my_robot_bringup/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## Testing Checklist

✅ Both motors spin when commanded  
✅ Forward command drives straight  
✅ Backward command reverses straight  
✅ Rotation command turns in place  
✅ Arc motion works (forward + rotation)  
✅ Watchdog stops motors after 1 second  
✅ Keyboard controls responsive  
✅ Emergency stop (Ctrl+C) works  
✅ GPIO cleanup on shutdown  

---

## Known Issues & Limitations

### Issue 1: ~~No Wheel Encoders~~ (RESOLVED)

**Status:** Hall effect encoders now integrated (288 ticks/rev, 3 pulses × 2 edges × 48:1 gear)
**Implementation:** Interrupt-based quadrature decoding via `GPIO.add_event_detect()` on both channels
**Output:** Odometry published to `/odom` topic with TF transforms

### Issue 2: L298N Voltage Drop

**Problem:** ~2V loss reduces effective motor voltage  
**Example:** 9V battery → 7V to motors (after L298N drop)  
**Impact:** Requires higher battery voltage than motor nominal  
**Future:** Hoverboard motor controller more efficient

### Issue 3: Wheel Base Measurement

**Current:** `wheel_base: 0.165 m` measured
**Impact:** Rotation calculations should be accurate
**Note:** Verify measurement if robot doesn't turn correctly

### Issue 4: No Load Testing Only

**Current:** Wheels off ground (bench testing)  
**Missing:** Real load characteristics, traction, friction  
**Future:** Ground testing needed before navigation

---

## Troubleshooting

### Motors Don't Start

**Symptom:** Velocity command sent, but motors don't spin

**Check:**
1. Battery voltage (should be 6-9V)
2. PWM duty cycle reaching 80% minimum
3. GPIO permissions (`groups` should show `gpio` and `dialout`)
4. Physical wiring connections

**Test:**
```bash
# Send high-speed command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### Wrong Direction

**Symptom:** Forward command goes backward, or rotation wrong direction

**Check Motor A Inversion:**
```python
# In motor_controller.py, set_motor_a() should have:
if duty > 0:
    GPIO.output(self.IN1, GPIO.LOW)   # SWAPPED
    GPIO.output(self.IN2, GPIO.HIGH)  # SWAPPED
```

**If rotation direction wrong:** Swap motor A/B assignments in code

### Motors Jerk or Stutter

**Possible causes:**
1. Battery voltage sagging under load
2. Loose connections
3. PWM frequency too high (try reducing to 100 Hz)

### Teleop Not Publishing

**Symptom:** Keyboard prints messages but motors don't move

**Check:**
1. Motor controller node running (`ros2 node list`)
2. Topic connection (`ros2 topic info /cmd_vel`)
3. Messages flowing (`ros2 topic echo /cmd_vel`)
4. `self.publish_cmd_vel()` called after each key handler

---

## Differential Drive Kinematics Reference

**Input:** `/cmd_vel` → `linear.x` (m/s) and `angular.z` (rad/s)

**Wheel Velocities:**
```python
v_left = linear.x - (angular.z * wheel_base / 2.0)
v_right = linear.x + (angular.z * wheel_base / 2.0)
```

**Examples:**

| Command | linear.x | angular.z | v_left | v_right | Result |
|---------|----------|-----------|--------|---------|--------|
| Forward | 0.2 | 0.0 | 0.2 | 0.2 | Straight forward |
| Backward | -0.2 | 0.0 | -0.2 | -0.2 | Straight backward |
| Rotate Left | 0.0 | 0.8 | -0.072 | 0.072 | Spin counter-clockwise |
| Arc Left | 0.2 | 0.3 | 0.173 | 0.227 | Curved forward-left |

---

## Migration Path to Hoverboard Motors

**When ready to switch:**

1. **Hardware:**
   - Remove L298N
   - Connect hoverboard motor controller via UART
   - Configure serial protocol (likely custom or ESC protocol)

2. **Software Changes:**
   - Replace `motor_controller.py` GPIO control with serial commands
   - Add encoder feedback reading (hall sensors)
   - Update velocity mapping (hoverboard motors different characteristics)
   - Publish `/odom` from encoder feedback

3. **Parameters to Update:**
   - `max_speed` (likely higher with hoverboard motors)
   - `min_duty_cycle` (may not need 80% minimum)
   - `wheel_base` (measure actual distance)
   - Add encoder parameters (ticks per revolution, gear ratio)

4. **Keep:**
   - Differential drive kinematics math
   - ROS 2 node structure
   - `/cmd_vel` interface
   - Keyboard teleop (no changes needed)

---

## Resources

**Hardware:**
- [L298N Datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [DG01D-E Motor Specs](https://cdn.sparkfun.com/assets/8/3/b/e/4/DS-16413-DG01D-E_Motor_with_Encoder.pdf)

**ROS 2:**
- [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [RPi.GPIO Documentation](https://pypi.org/project/RPi.GPIO/)

**Differential Drive:**
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)

---

## Status Summary

**Working:** ✅
- Motor control via ROS 2
- Keyboard teleoperation
- Differential drive kinematics
- Safety features (watchdog, emergency stop)
- Hall effect encoder feedback (288 ticks/rev, interrupt-based quadrature)
- Odometry publishing (`/odom` topic + TF)
- Encoder tick clamping (noise rejection)

**Not Implemented:**
- Closed-loop speed control (open-loop only)
- Ground testing (bench testing only)
- Battery voltage monitoring

**Next Steps:**
- Continue Articulated Robotics tutorial series
- Add remaining sensors (BNO055 IMU, RealSense camera)
- Integrate SLAM with RPLIDAR
- Eventually migrate to hoverboard motor platform

---

**End of L298N Motor Control Documentation**