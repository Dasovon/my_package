# Encoder Integration & Odometry Complete

**Date:** February 1, 2026  
**Status:** ✅ WORKING - Encoders reading, odometry publishing, motors responding  
**Next:** BNO055 IMU integration or SLAM with RPLIDAR

---

## What We Accomplished

### Hardware Integration
- ✅ Connected DG01D-E Hall effect encoders to Raspberry Pi GPIO
- ✅ Interrupt-based quadrature decoding (H1/H2 channels) via `GPIO.add_event_detect()`
- ✅ Motor direction calibrated (`encoder_left_inverted: True`, `encoder_right_inverted: False`)
- ✅ 288 ticks/rev (3 pulses × 2 edges × 48:1 gear ratio)

### Software Implementation
- ✅ Open-loop velocity control (min duty 80% for reliable starting)
- ✅ Encoder-based odometry calculation
- ✅ Odometry publishing (`/odom` topic + TF transform)
- ✅ TF tree: `odom` → `base_footprint` → `base_link` → sensors
- ✅ Differential drive kinematics (wheel base 165mm)

### Verified Working
- ✅ Motors respond to `/cmd_vel` commands
- ✅ Encoders count ticks during motion
- ✅ Odometry position updates as robot moves
- ✅ TF transforms published correctly
- ✅ Keyboard teleop functional (W/A/S/D controls)

---

## Final Configuration

### Robot Specifications
```yaml
Wheel base: 165mm (0.165 m)
Wheel diameter: 64.75mm (0.06475 m)
Wheel circumference: 203.4mm
Distance per encoder tick: 0.706mm
Encoder resolution: 288 ticks per wheel revolution (3 pulses × 2 edges × 48:1)
Gear ratio: 1:48
Motor voltage: 6-9V (currently ~7V with L298N losses)
```

### Motor Control Parameters
```yaml
min_duty_cycle: 80%     # Reliable starting threshold
max_duty_cycle: 100%
max_speed: 0.5 m/s
max_angular_speed: 2.0 rad/s
pwm_frequency: 1000 Hz
ramp_rate: 10% per iteration (50 Hz control loop)
```

### GPIO Pin Mapping

All pins are configurable via ROS 2 parameters (declared in `motor_controller.py`, overridden in `config/motor_controller.yaml`).

**Motor A (Left Wheel):**
- Enable: BCM 17 / physical pin 11 (`motor_enable_a_pin`)
- IN1: BCM 27 / physical pin 13 (`motor_in1_pin`)
- IN2: BCM 22 / physical pin 15 (`motor_in2_pin`)
- Encoder H1: BCM 23 / physical pin 16 (`encoder_left_a_pin`)
- Encoder H2: BCM 24 / physical pin 18 (`encoder_left_b_pin`)

**Motor B (Right Wheel):**
- Enable: BCM 13 / physical pin 33 (`motor_enable_b_pin`)
- IN3: BCM 19 / physical pin 35 (`motor_in3_pin`)
- IN4: BCM 26 / physical pin 37 (`motor_in4_pin`)
- Encoder H1: BCM 25 / physical pin 22 (`encoder_right_a_pin`)
- Encoder H2: BCM 5 / physical pin 29 (`encoder_right_b_pin`)

**Encoder Configuration:**
- `encoder_left_inverted`: True (compensates for mirrored left motor mount)
- `encoder_right_inverted`: False
- `encoder_bouncetime_ms`: 1 (GPIO edge detection debounce)

**Encoder Power:**
- +5V: Physical pins 2 or 4
- GND: Physical pins 6 and 9

---

## Key Learnings

### Why 80% Minimum Duty?
DG01D-E motors have high static friction. Below 60% duty, motors won't overcome breakaway torque. At 80%, reliable starting on every command.

### Encoder Inversion
Physical motors are wired opposite to each other on chassis. The left encoder is inverted in software (`encoder_left_inverted: True`) while the right encoder reads normally (`encoder_right_inverted: False`). Motor A direction is also compensated in code (`set_motor_a()` has reversed GPIO logic).

### Open-Loop vs Closed-Loop
**Current:** Open-loop control (no PID feedback)
- Encoders used for odometry only
- Simple, reliable, predictable behavior
- Good enough for teleoperation and basic navigation

**Future:** Closed-loop PID control
- Use encoder feedback to match commanded velocity exactly
- Requires tuning Kp, Ki, Kd gains
- Better for precise autonomous navigation
- Can add when needed for Nav2 integration

---

## ROS 2 Topics Published
```bash
/odom (nav_msgs/Odometry)
- Position: x, y, theta
- Velocity: linear.x, angular.z
- Published at 50 Hz

/tf (TF transform)
- odom → base_footprint
- Updated at 50 Hz
```

---

## Usage

### Launch Motor Controller
```bash
ssh hoverbot
ros2 launch my_robot_bringup motor_control.launch.py
```

### Keyboard Teleop
```bash
ros2 run my_robot_bringup teleop_keyboard.py
```

### Manual Velocity Commands
```bash
# Forward
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Rotate
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

### Monitor Odometry
```bash
ros2 topic echo /odom
ros2 topic echo /odom --field pose.pose.position.x
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## Files Modified

### New Files
- `ENCODER_ODOMETRY_COMPLETE.md` - This documentation

### Modified Files
- `my_robot_bringup/motor_controller.py` - Added encoder reading and odometry
- `config/motor_controller.yaml` - Added encoder parameters, increased min_duty to 80%
- `package.xml` - Added tf2_ros dependency

### Configuration Files
```yaml
# config/motor_controller.yaml
wheel_base: 0.165
wheel_diameter: 0.06475
encoder_ticks_per_rev: 288
min_duty_cycle: 80
```

---

## Next Steps

### Option 1: BNO055 IMU Integration
**Why:** Improve odometry accuracy with sensor fusion
- Install BNO055 ROS 2 driver
- Configure I2C communication (address 0x28)
- Calibrate IMU (accelerometer, gyro, magnetometer)
- Fuse IMU orientation with wheel odometry

**Time:** ~30-45 minutes

### Option 2: SLAM with RPLIDAR + Odometry
**Why:** Build maps and localize robot
- RPLIDAR already working (publishing `/scan`)
- Odometry now working (publishing `/odom` and TF)
- Ready for slam_toolbox or cartographer
- Can build maps while driving with teleop

**Time:** ~1 hour setup + testing

### Option 3: Continue Articulated Robotics Tutorial
**Next tutorial:** Mobile robot kinematics, Nav2 setup
- We've now covered most prerequisites
- Have URDF, simulation, sensors, odometry
- Ready for autonomous navigation

---

## Known Limitations (L298N Temporary Setup)

### L298N Issues
- ✅ Voltage drop: ~2V loss (9V battery → ~7V to motors)
- ✅ Efficiency: ~60-70% (wastes power as heat)
- ✅ Current limit: 2A per channel (marginal for stall)

### Future Migration to Hoverboard Motors
- Higher voltage (36V nominal)
- Higher current capability (10-20A)
- Built-in encoder support
- UART/CAN communication
- Better efficiency

**When:** After completing Articulated Robotics tutorial series and validating navigation stack with current setup.

---

## Troubleshooting Reference

### Motors Don't Move
- Check battery voltage (should be 6-9V)
- Check min_duty_cycle in config (try 90%)
- Verify GPIO connections
- Test with full speed: `ros2 topic pub /cmd_vel ... "{linear: {x: 0.5}}"`

### Encoders Not Counting
- Check 5V power to encoder boards
- Check GPIO pin connections (H1/H2 wires)
- Run test_encoders.py to verify
- Manually spin wheels and watch tick counts

### Odometry Drifts Wrong Direction
- One encoder wired backward
- Check quadrature decoding logic
- Verify encoder direction matches motor direction

### TF Transform Errors
- Check frame names match URDF
- Verify odom_frame and base_frame parameters
- Run: `ros2 run tf2_tools view_frames`

---

## Performance Baseline

**Odometry Accuracy (Bench Test):**
- Short distances (<1m): ±5cm error
- Rotation in place: ±3 degrees error
- Limited by wheel slip and open-loop control

**Control Response:**
- Time to min duty (80%): 0.1-0.2 seconds (ramping)
- Time to full speed: 0.2-0.3 seconds
- Watchdog timeout: 1.0 second

**Resource Usage (Raspberry Pi 4):**
- motor_controller CPU: ~10-15%
- Memory: ~40 MB
- Control loop: 50 Hz stable
- Encoder reading: interrupt-based (GPIO edge detection)

---

## Verification Checklist

✅ Motors respond to `/cmd_vel`  
✅ Forward command drives straight  
✅ Rotation commands turn in place  
✅ Encoders increment during motion  
✅ `/odom` topic publishes at 50 Hz  
✅ Position.x increases when driving forward  
✅ TF transform odom→base_footprint updates  
✅ `tf2_echo odom base_footprint` shows live transform  
✅ Keyboard teleop works (W/A/S/D/Space)  
✅ Watchdog stops motors after 1 second  
✅ Emergency stop (Ctrl+C) works  
✅ All code committed to Git  

**Status:** Encoder integration and odometry complete! Ready for sensor fusion or SLAM. 🚀