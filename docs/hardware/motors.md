# Motors — Troubleshooting & Reference

## Specs

| Parameter | Value |
|---|---|
| Motor model | DG01D-E DC gearmotor |
| Voltage | 6V nominal, 3–9V range |
| Gear ratio | 1:48 |
| Driver | L298N dual H-bridge |
| Motor power supply | 6–9V battery |
| Logic power | 5V from Pi (via L298N onboard regulator) |
| Common ground | Battery GND connected to Pi GND |

**L298N note:** ~2V voltage drop (9V battery → ~7V to motors). 60–70% efficient. 2A per channel current limit.

---

## GPIO Wiring

All BCM numbers. Set via `config/motor_controller.yaml`.

### Motor A — Left Wheel (INVERTED in software)

| Function | BCM GPIO | Physical Pin | L298N Terminal |
|---|---|---|---|
| Enable (PWM) | 17 | 11 | ENA |
| IN1 | 27 | 13 | IN1 |
| IN2 | 22 | 15 | IN2 |

### Motor B — Right Wheel (normal)

| Function | BCM GPIO | Physical Pin | L298N Terminal |
|---|---|---|---|
| Enable (PWM) | 13 | 33 | ENB |
| IN3 | 19 | 35 | IN3 |
| IN4 | 26 | 37 | IN4 |

**Motor A is inverted in software** — IN1/IN2 are swapped in `motor_controller.py` to compensate for mirrored physical wiring. Do not rewire; do not remove the software inversion.

---

## ROS Interface

| Topic | Direction | Type | Rate |
|---|---|---|---|
| `/cmd_vel` | Subscribe | `geometry_msgs/msg/Twist` | — |
| `/odom` | Publish | `nav_msgs/msg/Odometry` | 50 Hz |
| `/joint_states` | Publish | `sensor_msgs/msg/JointState` | 50 Hz |
| `/tf` | Publish | TF `odom → base_footprint` | 50 Hz |

**Key config (`config/motor_controller.yaml`):**
```yaml
min_duty_cycle: 90       # CRITICAL — do not lower
max_duty_cycle: 100
pwm_frequency: 1000
wheel_base: 0.236        # Effective (calibrated), not physical 0.165
wheel_diameter: 0.06475
max_speed: 0.5           # m/s
max_angular_speed: 2.0   # rad/s
```

**Safety features:**
- 1-second watchdog: motors stop automatically if no `/cmd_vel` received
- PWM ramping: 10% change per 50 Hz tick (prevents jerks)
- Velocity clamping within configured limits
- SIGTERM handler: `kill <pid>` triggers GPIO cleanup (motors stop cleanly)
- `gpio-motor-stop` systemd service: sets all motor pins OUTPUT LOW at boot, before bringup starts

---

## How to Test

**Prerequisites:** Pi bringup running (`full_bringup.launch.py`). Always use teleop — never send test velocity commands from Claude Code.

### 1. Verify motor controller is running
```bash
ssh ryan@192.168.86.33 "ps aux | grep motor_controller | grep -v grep"
```
Expected: one process visible.

### 2. Check /cmd_vel is being received
On dev machine:
```bash
source ~/dev_ws/install/setup.bash
ros2 topic echo /cmd_vel
```
Drive with teleop and verify messages appear.

### 3. Drive forward/backward
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- Press `i` — robot should move forward, both wheels spinning in same direction
- Press `,` — robot should move backward
- Press `j` / `l` — robot should rotate left / right in place

### 4. Verify /odom updates while moving
```bash
ros2 topic echo /odom --field pose.pose.position
```
x and y should change while driving.

---

## Expected Good State

- Both wheels spin smoothly at 90%+ duty cycle
- Robot drives straight (not veering) at `linear.x = 0.2`
- `/odom` x position increases while driving forward
- Motors stop within 1 second of releasing teleop key (watchdog)
- No GPIO errors in bringup log

---

## Known Issues & Fixes

### Motors don't move at low speed commands
**Cause:** DG01D-E requires high breakaway torque. Below 90% duty cycle, motors are underpowered.
**Fix:** `min_duty_cycle` is set to 90 in `config/motor_controller.yaml`. Any non-zero velocity maps to at least 90% PWM. Do not lower this value.

### Robot veers left or right when driving straight
**Cause:** Wheel base calibration off, or one motor slightly stronger.
**Fix:** Adjust `wheel_base` in `config/motor_controller.yaml`. Larger value → less turning effect per angular command. Current calibrated value: 0.236 m (physical is 0.165 m).

### Motors keep running after Nav2 / teleop is killed
**Cause:** Motor controller holds last velocity command. Watchdog only fires if no message is received for 1s — if the last message was non-zero and then nothing arrives, it stops. But if a publisher is still sending, it won't.
**Fix:**
```bash
ssh ryan@192.168.86.33 "source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash && ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'"
```
Kill that publisher once Nav2 should take control again.

### Motors start moving immediately when bringup launches (no teleop running)
**Cause:** A stale `ros2 topic pub` process is running on the dev machine, publishing to `/cmd_vel`. When motor_controller starts and subscribes, it immediately receives the command.
**Check:**
```bash
source ~/dev_ws/install/setup.bash && ros2 topic info /cmd_vel
```
If Publisher count > 0 with no teleop/Nav2 running, find and kill the publisher:
```bash
ps aux | grep 'topic pub.*cmd_vel' | grep -v grep
kill -9 <pid>
```

### Motors start on boot before bringup (GPIO floating)
**Cause:** L298N breakout boards pull ENA/ENB HIGH by default. Before `motor_controller` runs and sets up GPIO, the IN pins are floating inputs — noise can drive the H-bridge.
**Fix:** The `gpio-motor-stop` systemd service (`/home/ryan/gpio_motor_stop.py`) sets all motor pins OUTPUT LOW at boot. If motors move before bringup on a fresh boot, check the service is active:
```bash
ssh ryan@192.168.86.33 "systemctl is-active gpio-motor-stop.service"
```

### Motors keep spinning after motor_controller is killed with kill -9
**Cause:** `kill -9` (SIGKILL) cannot be caught — GPIO cleanup never runs. Pins hold their last driven state.
**Fix:** Run the GPIO safety script directly:
```bash
ssh ryan@192.168.86.33 "python3 /home/ryan/gpio_motor_stop.py"
```
Note: `kill <pid>` (SIGTERM) is handled correctly since the SIGTERM fix — only `kill -9` has this issue.

### Motor controller not found / GPIO error on startup
**Fix:** Restart full bringup. Never restart motor_controller alone outside the launch framework — DDS discovery is unreliable when restarted manually.
```bash
ssh ryan@192.168.86.33 "source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash && nohup ros2 launch my_robot_bringup full_bringup.launch.py > /tmp/bringup.log 2>&1 &"
```

### One wheel doesn't move
**Likely cause:** L298N channel fault, or enable pin not configured correctly.
**Check:** Swap enable pins in config temporarily to isolate whether it's the Pi GPIO or the L298N channel.

### Motors jerk on startup
**Cause:** Ramp rate too aggressive or min_duty_cycle too high relative to load.
**Note:** This is expected at first command — ramping is 10%/tick at 50 Hz, so 0→90% takes ~180ms.
