# Encoders — Troubleshooting & Reference

## Specs

| Parameter | Value |
|---|---|
| Type | Hall effect, quadrature (2-channel) |
| Pulses per motor rev | 3 |
| Edges per pulse | 2 (rising + falling) |
| Gear ratio | 1:48 |
| **Ticks per wheel rev** | **3 × 2 × 48 = 288** |
| Distance per tick | ~0.706 mm (wheel circumference / 288) |
| Decoding | Interrupt-based quadrature via `GPIO.add_event_detect()` |
| Debounce | None — Hall effect sensors don't bounce |

---

## GPIO Wiring

All BCM numbers. VCC is **3.3V** (not 5V).

| Function | BCM GPIO | Physical Pin |
|---|---|---|
| Left Encoder H1 (A) | 23 | 16 |
| Left Encoder H2 (B) | 24 | 18 |
| Right Encoder H1 (A) | 25 | 22 |
| Right Encoder H2 (B) | 5 | 29 |
| VCC (both encoders) | — | Pin 1 or 17 (3.3V) |
| GND (both encoders) | — | Any ground pin |

**Direction inversion:**
- `encoder_left_inverted: true` — compensates for mirrored left motor mount
- `encoder_right_inverted: false` — normal

---

## ROS Interface

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | Wheel positions in radians |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | Computed from encoder ticks |

**Key config (`config/motor_controller.yaml`):**
```yaml
encoder_ticks_per_rev: 288
encoder_left_inverted: true
encoder_right_inverted: false
encoder_bouncetime_ms: 1     # Effectively 0 — Hall effect doesn't need it
max_tick_delta: 1200         # Rejects noise spikes above this value per cycle
```

---

## How to Test

**Prerequisites:** Pi bringup running. Wheels must be free to rotate (robot on ground or lifted).

### 1. Check /joint_states while stationary
```bash
ros2 topic echo /joint_states
```
Expected: `position` values for both wheels slowly accumulating (or static), `velocity` near 0.0.

### 2. Spin wheels by hand and watch tick counts
Lift the robot slightly, spin each wheel by hand and watch `/joint_states`.
- Left wheel spin → `left_wheel_joint` position changes
- Right wheel spin → `right_wheel_joint` position changes

### 3. Drive forward and verify both wheels count up
```bash
ros2 topic echo /joint_states --field position
```
Drive forward with teleop. Both values should increase. If one stays at 0.0, see Known Issues.

### 4. Verify odometry updates
```bash
ros2 topic echo /odom --field pose.pose.position
```
Drive in a straight line ~1 m. `/odom` x should read approximately 1.0.

### 5. Check for encoder noise/spikes
```bash
ros2 topic echo /joint_states --field velocity
```
While stationary, velocities should be 0.0 or near-zero. Large random spikes indicate wiring noise.

---

## Expected Good State

- Both wheels show changing `position` in `/joint_states` when driven
- Velocities in `/joint_states` match commanded velocity direction (positive = forward)
- `/odom` x increases when driving forward, decreases when reversing
- No tick spike errors in bringup log
- Driving 1 m forward → `/odom` x ≈ 1.0 m (within ~10%)

---

## Known Issues & Fixes

### One wheel shows 0.0 forever in /joint_states
**Cause:** VCC wire disconnected from that encoder.
**Fix:** Right encoder VCC (pin 1 or 17, 3.3V) was found disconnected in a previous session. Check all encoder VCC and signal wires. A wire pulled from the connector looks connected but isn't making contact.

**Diagnostic:**
```bash
ros2 topic echo /joint_states --field position
```
Watch both values. The one that never changes is the faulty encoder.

### Encoder counts wrong direction (odom drifts backward when driving forward)
**Cause:** `encoder_left_inverted` or `encoder_right_inverted` wrong.
**Fix:** Flip the `inverted` param for the misbehaving wheel in `config/motor_controller.yaml`.

### Ticks spike to huge values randomly
**Cause:** Electrical noise on encoder signal lines, or `max_tick_delta` too high.
**Fix:** `max_tick_delta: 1200` in config rejects spikes. If still occurring, add ferrite beads or shorten encoder cables. Do not add software debounce — Hall effect sensors don't need it and debounce causes missed edges.

### Odometry drifts significantly after 1 m
**Cause:** Wheel diameter or tick count wrong, or wheel slippage.
**Check:** Verify `encoder_ticks_per_rev: 288` and `wheel_diameter: 0.06475` in config. Measure actual wheel circumference and compare.

### Right encoder H1 (GPIO 25) always reads 0 even with VCC connected
**Cause:** Signal wire (physical pin 22 → GPIO 25) disconnected at Pi or encoder board.
**Fix:** Reseat or replace the wire for GPIO 25.
