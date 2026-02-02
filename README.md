# my_robot_bringup

ROS 2 robot bringup package template.

## Structure

```
my_package/
├── CMakeLists.txt
├── package.xml
├── config/           # Configuration files
├── launch/           # Launch files
│   ├── talker.launch.py
│   └── listener.launch.py
└── README.md
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

## Usage

Launch the demo talker:
```bash
ros2 launch my_robot_bringup talker.launch.py
```

Launch the demo listener:
```bash
ros2 launch my_robot_bringup listener.launch.py
```

## License

MIT
