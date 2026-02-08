# Startup and Testing Guide

How to power on both machines, verify connectivity, build the workspace, and run tests. This guide assumes initial setup (OS, ROS 2, networking, SSH) is already complete per [SETUP.md](SETUP.md).

---

## Table of Contents

1. [Quick Reference](#quick-reference)
2. [Starting the Robot (hoverbot)](#starting-the-robot-hoverbot)
3. [Starting the Dev Machine](#starting-the-dev-machine)
4. [Verifying the Connection](#verifying-the-connection)
5. [Building the Workspace](#building-the-workspace)
6. [Running Unit Tests](#running-unit-tests)
7. [Hardware Verification on hoverbot](#hardware-verification-on-hoverbot)
8. [Launching the Full Robot Stack](#launching-the-full-robot-stack)
9. [Teleop and Live Testing](#teleop-and-live-testing)
10. [Simulation Testing (Dev Only)](#simulation-testing-dev-only)
11. [Cross-Machine Communication Tests](#cross-machine-communication-tests)
12. [Shutting Down](#shutting-down)

---

## Quick Reference

| | Robot (hoverbot) | Dev Machine |
|---|---|---|
| **Hardware** | Raspberry Pi 4/5 | Ubuntu 22.04 Desktop |
| **IP** | 192.168.86.33 | 192.168.86.37 |
| **Workspace** | `~/robot_ws` | `~/dev_ws` |
| **ROS install** | `ros-humble-ros-base` | `ros-humble-desktop` |
| **ROS_DOMAIN_ID** | 42 | 42 |

---

## Starting the Robot (hoverbot)

### 1. Power on

1. Insert the microSD card (if removed).
2. Connect the RPLIDAR to a USB port.
3. Connect the motor power supply (6-9V battery to L298N).
4. Connect the Pi's 5V power supply.
5. Wait for the Pi to finish booting. The green activity LED will flash during boot and settle when ready.

### 2. SSH in from the dev machine

```bash
ssh hoverbot
```

If the hostname doesn't resolve, use the IP directly:

```bash
ssh ryan@192.168.86.33
```

### 3. Verify environment variables

```bash
echo $ROS_DOMAIN_ID        # Should print: 42
echo $ROS_LOCALHOST_ONLY    # Should print: 0
```

If either is missing, source the shell profile:

```bash
source ~/.bashrc
```

### 4. Pull latest code

```bash
cd ~/robot_ws/src/my_package
git pull origin main
```

---

## Starting the Dev Machine

### 1. Boot normally

Log in to your Ubuntu 22.04 desktop.

### 2. Open a terminal and verify environment

```bash
echo $ROS_DOMAIN_ID        # Should print: 42
echo $ROS_LOCALHOST_ONLY    # Should print: 0
```

If either is missing:

```bash
source ~/.bashrc
```

### 3. Pull latest code

```bash
cd ~/dev_ws/src/my_robot_bringup
git pull origin main
```

---

## Verifying the Connection

Run these checks before doing any cross-machine work.

### Ping test

From the dev machine:

```bash
ping -c 5 hoverbot
```

Expected: replies with < 20 ms latency, 0% packet loss.

### SSH test

```bash
ssh hoverbot "hostname && echo 'SSH OK'"
```

Expected output:

```
hoverbot
SSH OK
```

### ROS 2 daemon restart (if nodes aren't discovering each other)

Run on **both** machines:

```bash
ros2 daemon stop
ros2 daemon start
```

---

## Building the Workspace

Always rebuild after pulling new code.

### On hoverbot

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### On dev

```bash
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

If the build fails with missing dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Then rebuild.

---

## Running Unit Tests

Unit tests validate the motor controller math (velocity mapping, odometry, kinematics) and do not require hardware.

### Using colcon (recommended)

Run from the workspace root on either machine:

```bash
cd ~/dev_ws    # or ~/robot_ws on hoverbot
colcon test --packages-select my_robot_bringup
```

View the results:

```bash
colcon test-result --verbose
```

### Using pytest directly

```bash
cd ~/dev_ws/src/my_robot_bringup    # or ~/robot_ws/src/my_package
python3 -m pytest test/test_motor_controller.py -v
```

### What the tests cover

| Test area | Description |
|---|---|
| `velocity_to_duty()` | Maps linear velocity to PWM duty cycle |
| `clamp_tick_delta()` | Validates encoder tick deltas |
| `yaw_to_quaternion()` | Converts yaw angle to quaternion |
| Differential drive kinematics | Forward, turning, and stationary cases |
| Odometry integration | Pose updates from encoder ticks |

All tests should pass on any machine — they mock the GPIO layer and don't need a Raspberry Pi.

---

## Hardware Verification on hoverbot

Run these checks after booting the Pi and before launching the full stack.

### Check GPIO access

```bash
python3 -c "import RPi.GPIO; print('GPIO OK')"
```

If you see `Permission denied`, add your user to the gpio group:

```bash
sudo usermod -a -G gpio $USER
```

Then log out and back in.

### Check RPLIDAR connection

```bash
ls /dev/ttyUSB0
```

If the device isn't present, unplug and replug the RPLIDAR USB cable. Verify the serial port permissions:

```bash
sudo chmod 666 /dev/ttyUSB0
```

### Check motor power

Make sure the L298N motor driver is receiving power from the battery. The onboard LED on the L298N should be lit. If the motors don't respond later during teleop, check:

- Battery charge level
- Wiring between the L298N and the Pi GPIO header (see [ROBOT_PHYSICAL_CHARACTERISTICS.md](ROBOT_PHYSICAL_CHARACTERISTICS.md))

---

## Launching the Full Robot Stack

### On hoverbot — start all robot nodes

```bash
ros2 launch my_robot_bringup full_bringup.launch.py
```

This launches three subsystems:

1. **Robot State Publisher** — publishes the URDF and TF tree
2. **Motor Controller** — subscribes to `/cmd_vel`, publishes `/odom`
3. **RPLIDAR** — publishes `/scan` from the laser scanner

### Verify nodes are running

On hoverbot (or from dev):

```bash
ros2 node list
```

Expected nodes:

```
/robot_state_publisher
/motor_controller
/rplidar_node
```

### Verify topics are publishing

```bash
ros2 topic list
```

Key topics to check:

| Topic | Source | Type |
|---|---|---|
| `/cmd_vel` | Teleop / Nav | `geometry_msgs/msg/Twist` |
| `/odom` | Motor controller | `nav_msgs/msg/Odometry` |
| `/scan` | RPLIDAR | `sensor_msgs/msg/LaserScan` |
| `/robot_description` | RSP | `std_msgs/msg/String` |
| `/tf` | RSP + Motor controller | `tf2_msgs/msg/TFMessage` |

Check message rates:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
```

---

## Teleop and Live Testing

### Option A: Launch teleop from dev

```bash
ros2 launch my_robot_bringup teleop_keyboard.launch.py
```

This opens an xterm window for keyboard input. Use arrow keys or WASD (depending on the node implementation) to drive the robot.

### Option B: Publish cmd_vel manually

Send a single velocity command from any terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

The robot should drive forward briefly. Send a stop command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Monitor odometry in real time

```bash
ros2 topic echo /odom
```

### Visualize in RViz (dev machine)

```bash
rviz2
```

Add displays for:
- **RobotModel** (set description topic to `/robot_description`)
- **LaserScan** (topic `/scan`)
- **TF** (to see the coordinate frames)
- **Odometry** (topic `/odom`)

---

## Simulation Testing (Dev Only)

To test without the physical robot, use Gazebo on the dev machine.

### Launch simulation

```bash
ros2 launch my_robot_bringup launch_sim.launch.py
```

This starts Gazebo with the obstacles world and spawns the robot model.

### Drive the simulated robot

In a second terminal:

```bash
ros2 launch my_robot_bringup teleop_keyboard.launch.py
```

### Verify simulation topics

```bash
ros2 topic list
ros2 topic echo /odom
```

---

## Cross-Machine Communication Tests

These verify that ROS 2 DDS discovery is working between hoverbot and dev.

### Test 1: Talker / Listener

**On hoverbot:**

```bash
ros2 launch my_robot_bringup talker.launch.py
```

**On dev:**

```bash
ros2 topic echo /chatter
```

You should see `Hello World` messages. Press Ctrl+C to stop.

### Test 2: Node discovery

With any node running on hoverbot, run on dev:

```bash
ros2 node list
```

Nodes from hoverbot should appear in the list.

### Test 3: Service call

**On dev:**

```bash
ros2 launch my_robot_bringup service_example.launch.py
```

In another terminal:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
```

Expected response: `sum: 25`.

### Test 4: rqt_graph (dev)

```bash
rqt_graph
```

Verify that nodes from both machines appear and topics connect them correctly.

---

## Shutting Down

### Stop ROS nodes

Press Ctrl+C in each terminal running a launch file.

### Shut down hoverbot safely

```bash
sudo shutdown -h now
```

Wait for the green LED to stop flashing before disconnecting power. Pulling power without a clean shutdown risks SD card corruption.

### Disconnect motor power

After the Pi is off, disconnect the battery from the L298N to avoid draining it.

---

## Troubleshooting

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for solutions to common issues including:

- Nodes not discovering each other across machines
- RPLIDAR not detected
- GPIO permission errors
- Build failures
- WiFi disconnects on the Pi
