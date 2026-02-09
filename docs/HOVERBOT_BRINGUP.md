# Hoverbot Software Bring-Up (Step by Step)

This guide walks through powering on the hoverbot, connecting from the dev machine, and starting **all on-robot software** (state publisher, motor controller, and RPLIDAR). It assumes initial OS + ROS setup is complete (see [SETUP.md](SETUP.md)).

---

## 1) Power on the hoverbot

1. Insert the microSD card (if removed).
2. Connect the **RPLIDAR** to a USB port on the Pi.
3. Connect the **motor power** supply (6–9V battery to the L298N).
4. Connect the **Pi 5V** power supply.
5. Wait for the Pi to finish booting (LED activity settles when ready).

---

## 2) SSH into hoverbot

From the dev machine:

```bash
ssh hoverbot
```

If the hostname does not resolve:

```bash
ssh ryan@192.168.86.33
```

---

## 3) Verify the ROS environment

```bash
echo $ROS_DOMAIN_ID        # Expected: 42
echo $ROS_LOCALHOST_ONLY    # Expected: 0
```

If either is missing:

```bash
source ~/.bashrc
```

---

## 4) Update the code on hoverbot

```bash
cd ~/robot_ws/src/my_package
git pull origin main
```

---

## 5) Build the hoverbot workspace

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

Tip: paste the command **once**. If you accidentally paste it twice (e.g., `--symlink-installcolcon build`), `colcon` will report unrecognized arguments.

If you see warnings about missing paths in `COLCON_PREFIX_PATH`/`AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH` right after a clean build, they usually mean your shell still has older values from a different workspace. Re-source this workspace after the build (`source ~/robot_ws/install/setup.bash`) or open a fresh terminal and try again.

If dependencies are missing:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

On hoverbot, you may see:

```
my_robot_bringup: Cannot locate rosdep definition for [python3-rpi.gpio]
```

This is expected on the Pi. It uses the OS package (`python3-rpi.gpio`) and is already satisfied, so you can ignore the warning and continue.

Note: that error line is output from `rosdep`. Don’t paste it back into the shell—`my_robot_bringup: Cannot locate rosdep...` is not a command.

Then rebuild.

---

## 6) Quick hardware checks (recommended)

### GPIO access

```bash
python3 -c "import RPi.GPIO; print('GPIO OK')"
```

If you see `Permission denied`:

```bash
sudo usermod -a -G gpio $USER
```

Log out and back in.

### RPLIDAR device

```bash
ls /dev/ttyUSB0
```

If missing, unplug/replug the USB cable. If permissions fail:

```bash
sudo chmod 666 /dev/ttyUSB0
```

---

## 7) Launch the full hoverbot stack

```bash
ros2 launch my_robot_bringup full_bringup.launch.py
```

If the motor controller exits with GPIO warnings or `RuntimeError: Failed to add edge detection`, another process likely has the GPIO pins in use (or they were left in a bad state). Stop any other ROS/motor processes, wait a few seconds, and relaunch. If it persists, reboot the Pi to clear GPIO state:

```bash
sudo reboot
```

If you see the launch error `process has died ... motor_controller.py` with exit code 1, scroll up in the same terminal for the real cause (often a GPIO edge-detect failure). Resolve that root error and relaunch.

If the error repeats, confirm only one motor controller is running (`ros2 node list`) before relaunching.

This starts:

1. **Robot State Publisher** (URDF + TF)
2. **Motor Controller** (`/cmd_vel` → `/odom`)
3. **RPLIDAR** (`/scan`)

---

## 8) Confirm everything is running

In a second terminal on hoverbot (or from the dev machine):

```bash
ros2 node list
```

Expected nodes:

```
/robot_state_publisher
/motor_controller
/rplidar_node
```

Check topics:

```bash
ros2 topic list
```

Optional: verify publish rates:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
```

---

## 9) (Optional) Drive the robot

From the dev machine:

```bash
ros2 launch my_robot_bringup teleop_keyboard.launch.py
```

Or send a single command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## 10) Shut down safely

Stop launch files with **Ctrl+C**, then:

```bash
sudo shutdown -h now
```

Disconnect motor power after the Pi is fully off.
