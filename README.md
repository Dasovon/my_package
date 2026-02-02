# my_robot_bringup

ROS 2 differential drive robot bringup package with L298N motor control.

## Structure

```
my_package/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── motor_controller.yaml   # Motor and encoder parameters
├── launch/
│   ├── motor_control.launch.py # Motor controller node
│   ├── rsp.launch.py           # Robot state publisher
│   └── launch_sim.launch.py    # Gazebo simulation
├── my_robot_bringup/
│   ├── motor_controller.py     # Differential drive controller
│   └── teleop_keyboard.py      # Keyboard teleoperation
├── urdf/
│   ├── robot.urdf.xacro        # Main robot description
│   ├── robot_core.xacro        # Chassis and wheels
│   └── lidar.xacro             # LIDAR mount
└── worlds/
    ├── empty.world             # Empty Gazebo world
    └── obstacles.world         # World with obstacles
```

## Hardware

- **Motors**: DG01D-E with 1:48 gear ratio, 576 encoder ticks/rev
- **Driver**: L298N dual H-bridge
- **Platform**: Raspberry Pi with GPIO access

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

## Usage

### Physical Robot

**Terminal 1** - Launch motor controller:
```bash
ros2 launch my_robot_bringup motor_control.launch.py
```

**Terminal 2** - Launch keyboard teleop:
```bash
ros2 run my_robot_bringup teleop_keyboard.py
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| W / Up | Forward |
| S / Down | Backward |
| A / Left | Turn Left |
| D / Right | Turn Right |
| Space | Stop |
| Q | Quit |

### Simulation

```bash
ros2 launch my_robot_bringup launch_sim.launch.py
```

### Robot State Publisher (visualization)

```bash
ros2 launch my_robot_bringup rsp.launch.py
```

## Configuration

Edit `config/motor_controller.yaml` to adjust:
- Wheel dimensions and base width
- GPIO pin assignments
- Speed limits and PWM settings

## License

MIT
