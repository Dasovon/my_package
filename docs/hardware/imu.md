# IMU (BNO055) — Troubleshooting & Reference

## Specs

| Parameter | Value |
|---|---|
| Model | Adafruit BNO055 |
| Sensors | Accelerometer, gyroscope, magnetometer (9-DOF) |
| Interface | I2C, bus 1 |
| Address | 0x28 (ADR pin floating / tied low) |
| Operation mode | NDOF (0x0C) — full sensor fusion |
| Update rate | 100 Hz |
| Frame ID | `imu_link` |
| ROS driver | `ros-humble-bno055` |

---

## Wiring

| BNO055 Pin | Pi Connection | Physical Pin |
|---|---|---|
| VIN | 3.3V | Pin 1 or 17 |
| GND | GND | Any GND pin |
| SDA | GPIO 2 | Pin 3 |
| SCL | GPIO 3 | Pin 5 |
| ADR | Floating | — (sets address to 0x28) |

**Note:** ADR pin floating = I2C address 0x28. If ADR is tied high = 0x29. Current config uses 0x28.

---

## TF / URDF

- Frame: `imu_link`, child of `base_link`
- Static TF: `base_link → imu_link` at origin (0, 0, 0), identity rotation
- TF published as a static transform from `full_bringup.launch.py`

---

## ROS Interface

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/msg/Imu` | ~75 Hz | Fused orientation + angular velocity + linear acceleration |
| `/imu/imu_raw` | `sensor_msgs/msg/Imu` | ~75 Hz | Raw (unfused) sensor data |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | ~75 Hz | Magnetometer |
| `/imu/calib_status` | `std_msgs/msg/String` | 0.1 Hz | Calibration status string |

**Topic naming:** The bno055 driver natively publishes to `/imu/imu` (not `/imu/data`). A topic remapping in `full_bringup.launch.py` redirects it to `/imu/data` to match ROS convention and the EKF config. Do not remove this remapping.

**EKF fusion:** `/imu/data` is fused with `/odom` by `robot_localization` EKF node. EKF config: `config/ekf.yaml`. EKF does NOT publish TF (`publish_tf: false`).

**Config:** `config/bno055.yaml` — contains I2C bus, address, and saved calibration offsets loaded on startup.

---

## How to Test

**Prerequisites:** Pi bringup running.

### 1. Verify I2C device is detected (on Pi)
```bash
ssh ryan@192.168.86.33 "i2cdetect -y 1"
```
Expected: `0x28` appears in the address table. If missing, check wiring.

### 2. Check /imu/data is publishing
```bash
ros2 topic hz /imu/data
```
Expected: ~100 Hz.

### 3. Check orientation quaternion
```bash
ros2 topic echo /imu/data --field orientation --once
```
With robot flat on ground: `w ≈ 1.0`, `x ≈ 0`, `y ≈ 0`, `z ≈ 0`.

### 4. Tilt test — verify orientation changes
```bash
ros2 topic echo /imu/data --field orientation
```
Tilt the robot forward/back and left/right. `x` and `y` quaternion components should change. If values don't change, the IMU is frozen or the driver crashed.

### 5. Check calibration status
```bash
ros2 topic echo /imu/calib_status
```
Returns a byte (0–255). Upper nibble = system cal, lower nibble = component cal. Value of `255` (0xFF) = fully calibrated. Value of `0` = uncalibrated. The saved offsets in `config/bno055.yaml` load automatically on startup — you should not need to calibrate from scratch each boot.

### 6. Verify angular velocity (gyro)
```bash
ros2 topic echo /imu/data --field angular_velocity
```
While stationary: all values near 0. While rotating: `z` component should change (positive = counter-clockwise when viewed from above).

---

## Expected Good State

- `/imu/data` at 100 Hz
- Orientation quaternion responds to physical tilting
- Calibration status non-zero on startup (offsets loaded from config)
- `angular_velocity.z` changes during rotation
- `/imu/calib_status` reaches 255 after a few seconds of movement

---

## Known Issues & Fixes

### /imu/data not publishing — bno055 process running at 30% CPU but silent
**Cause:** Stale Fast-RTPS shared memory files in `/dev/shm/` from previously killed processes. The driver configures the sensor successfully ("configuration complete" in log) but then fails silently when trying to publish — `"Failed to publish: publisher's context is invalid"`. This happens when bno055 is killed with `kill -9` and leaves SHM files behind.
**Fix:** Clean SHM files and restart bringup:
```bash
ssh ryan@192.168.86.33 "rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps*"
# then restart bringup
```
**Diagnostic:** If bno055 log ends at "configuration complete" with no further output and `ros2 topic hz /imu/data` shows nothing, this is the cause. Confirmed by running bno055 in foreground and seeing the publish error.

### /imu/data not publishing — general check
**Check in order:**
1. `ssh ryan@192.168.86.33 "i2cdetect -y 1"` — is 0x28 visible?
2. `ssh ryan@192.168.86.33 "ps aux | grep bno055 | grep -v grep"` — is the driver running?
3. Check bringup log: `ssh ryan@192.168.86.33 "tail -30 /tmp/bringup.log"`
4. Check for stale SHM: `ssh ryan@192.168.86.33 "ls /dev/shm/fastrtps*"` — if present, clean them.

If 0x28 is missing from i2cdetect, it's a wiring problem (VIN, SDA, SCL, or GND loose).

### Two bno055 instances running simultaneously → I2C state corruption
**Cause:** Killing bno055 with `kill -9` while bringup is still running, then starting a new instance. Both processes fight over I2C, leaving the bus in a bad state.
**Symptom:** bno055 configures fine, driver runs at high CPU, but `/imu/data` never publishes even after SHM cleanup.
**Fix:** Full Pi reboot. There is no software-only recovery once I2C state is corrupted this way.
**Prevention:** Always kill the entire bringup, not individual nodes.

### Verify sensor is healthy (when driver is running but not publishing)
Read registers directly to confirm hardware is OK:
```bash
ssh ryan@192.168.86.33 "python3 -c \"
import smbus, time
bus = smbus.SMBus(1)
mode = bus.read_byte_data(0x28, 0x3D)
print(f'Op mode: 0x{mode:02X} (NDOF=0x0C, upper bits=status)')
status = bus.read_byte_data(0x28, 0x39)
print(f'Sys status: 0x{status:02X} (fusion running=5)')
calib = bus.read_byte_data(0x28, 0x35)
print(f'Calib: 0x{calib:02X} (sys/gyro/accel/mag, 3=full)')
\""
```
Expected healthy output: op mode `0x8C` (= NDOF + status bits), sys status `0x05`, calib `0xFC` or better.

### Orientation values frozen / not responding to tilt
**Cause:** BNO055 driver crashed or I2C bus locked up.
**Fix:** Restart Pi bringup. If i2cdetect still shows 0x28, do a full Pi reboot to reset the I2C bus.

### Calibration status always 0 after boot
**Cause:** Calibration offsets not loading from `config/bno055.yaml`, or offsets are stale/corrupt.
**Fix:** Move the robot in a figure-8 pattern for ~30 seconds. The BNO055 NDOF mode self-calibrates at runtime. To save new offsets, read `/imu/calib_status` bytes when at 255 and update `config/bno055.yaml`.

### EKF output worse than raw odometry
**Cause:** IMU `z` angular velocity and odometry `z` angular velocity conflicting (EKF tuning).
**Note:** EKF `publish_tf: false` — motor controller owns `odom → base_footprint` TF. EKF publishes `/odometry/filtered` only. If EKF diverges, disable it and use raw `/odom` for navigation.

### i2cdetect shows 0x28 but driver fails to open
**Cause:** I2C permissions. The bno055 driver needs access to `/dev/i2c-1`.
**Fix:**
```bash
ssh ryan@192.168.86.33 "sudo usermod -aG i2c ryan"
```
Then log out and back in (or reboot).

### Wrong I2C address (0x29 instead of 0x28)
**Cause:** ADR pin is tied high.
**Fix:** Leave ADR floating (for 0x28). If using 0x29, update `config/bno055.yaml` accordingly.
