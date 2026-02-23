# RPLIDAR A1 — Troubleshooting & Reference

## Specs

| Parameter | Value |
|---|---|
| Model | RPLIDAR A1 |
| Range | 0.15 m – 12 m |
| Scan rate | 10 Hz |
| Sample rate | 2 kHz (Standard mode) |
| Angular resolution | ~1° |
| Interface | USB via CP2102 UART adapter |
| Frame ID | `laser_frame` |
| ROS driver | `rplidar_ros` |

---

## Connection

**Always use the by-id path.** The `ttyUSBx` number changes on every power cycle.

```
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

USB device for power cycling:
```
/sys/bus/usb/devices/1-1.2/authorized
```

---

## URDF / TF

- Frame: `laser_frame`, child of `base_link`
- URDF orientation: `rpy="0 0 ${pi}"` — cable faces back, 0° scans directly away from cable, π rotation aligns forward scan with robot forward. **Do not change this.**
- Static TF published by RSP (robot_state_publisher) from URDF

---

## ROS Interface

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | 360° scan |

**Launch config:** `launch/full_bringup.launch.py` includes `rplidar.launch.py` with:
```python
scan_mode: 'Standard'     # CRITICAL — do not use Sensitivity mode
respawn: True
respawn_delay: 7.0        # Allows watchdog USB power cycle to complete
```

**Lidar watchdog** (`lidar_watchdog.py`): monitors `/scan`. If no subscribers, toggles lidar motor off (saves battery). If scan stops arriving, power cycles USB automatically.

Sudoers rule required for USB power cycle (on Pi):
```
ryan ALL=(ALL) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/1-1.2/authorized
```

---

## How to Test

**Prerequisites:** Pi bringup running.

### 1. Confirm /scan is publishing
```bash
ros2 topic hz /scan
```
Expected: ~10 Hz.

### 2. Check scan data looks valid
```bash
ros2 topic echo /scan --field ranges --once
```
Expected: 360 float values, most between 0.15 and 12.0. `inf` is valid (out of range). `nan` in large quantities indicates a problem.

### 3. Visualize in rviz2
```bash
rviz2 -d ~/dev_ws/src/my_package/config/nav2.rviz
```
Add a LaserScan display → topic `/scan` → frame `laser_frame`. Should see a ring of points around the robot matching the room.

### 4. Verify scan direction (forward = correct)
Points in front of the robot in rviz should correspond to objects physically in front of it. If they appear behind, the URDF `rpy` rotation is wrong.

### 5. Check lidar health via bringup log
```bash
ssh ryan@192.168.86.33 "tail -20 /tmp/bringup.log"
```
No health errors = good. Error codes 80008000/80008002 = USB power instability (see Known Issues).

---

## Expected Good State

- `/scan` publishing at 10 Hz (may start at 6–7 Hz for the first few seconds; normal warmup)
- 360 valid range readings per scan
- Scan ring in rviz matches room geometry
- No health errors in bringup log
- Lidar motor spinning audibly (distinct whirring sound)

---

## Known Issues & Fixes

### Health errors 80008000 / 80008001 / 80008002
**Cause:** USB power instability. The CP2102 adapter draws current spikes the Pi's USB port struggles with.
**Fix (automatic):** Lidar watchdog detects this and power cycles USB via `authorized` toggle.
**Fix (manual from dev):**
```bash
ssh ryan@192.168.86.33 "echo '0' | sudo tee /sys/bus/usb/devices/1-1.2/authorized && sleep 2 && echo '1' | sudo tee /sys/bus/usb/devices/1-1.2/authorized"
```
**Fix (hardware):** Powered USB hub eliminates this entirely.

### USB authorized-toggle doesn't fully reset the lidar
**Cause:** Software toggle doesn't fully reset the CP2102 hardware state.
**Symptom:** After a software toggle, the driver may report `"Standard mode not supported"` even though Standard mode is correctly configured. This means the hardware didn't reset cleanly.
**Fix:** Physical unplug and replug of the USB cable. Most reliable option.

### Lidar starts then immediately stops / timeout loop on startup
**Cause:** Same USB power issue. The `respawn_delay: 7.0` in `rplidar.launch.py` gives the watchdog time to finish its USB cycle before the driver restarts.
**Fix:** Physical unplug/replug, then let bringup restart the driver automatically.

### 3 rplidar processes on the same port
**Cause:** Previous crash left zombie processes; bringup respawned a new one; watchdog may have added another.
**Check:**
```bash
ssh ryan@192.168.86.33 "lsof /dev/ttyUSB0"
```
If 3 PIDs shown, kill the 2 oldest:
```bash
ssh ryan@192.168.86.33 "kill -9 <pid1> <pid2>"
```

### Sensitivity scan mode — DO NOT USE
**Cause:** Sensitivity mode (4kHz sample rate) overwhelms the UART buffer and causes driver crashes.
**Fix:** Always use `scan_mode: 'Standard'` in rplidar.launch.py. This is already set correctly.

### Scan shows no points / /scan not publishing
**Check in order:**
1. Is the lidar motor spinning? (listen for whirring)
2. `ros2 topic hz /scan` — is it publishing at all?
3. `ssh ryan@192.168.86.33 "lsof /dev/serial/by-id/..."` — is the serial port open?
4. Check bringup log for errors: `ssh ryan@192.168.86.33 "tail -30 /tmp/bringup.log"`

### Scan ring rotated / wrong orientation in rviz
**Cause:** URDF `rpy` for laser_frame is wrong, or TF is stale.
**Check:** The correct URDF value is `rpy="0 0 ${pi}"`. Do not change this. If rviz shows the ring rotated, verify RSP is running and publishing `/tf_static`.

### Sudoers rule missing (watchdog can't power cycle)
Run on Pi:
```bash
sudo python3 -c "open('/etc/sudoers.d/lidar-power-cycle','w').write('ryan ALL=(ALL) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/1-1.2/authorized\n')"
```
