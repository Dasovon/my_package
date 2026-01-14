# Complete Setup Guide - Hoverbot ROS 2 System

This guide walks through setting up the complete ROS 2 development environment from scratch, including networking, workspaces, and package configuration.

**Target:** Base ROS 2 system ready for sensor integration  
**Time Required:** 2-3 hours  
**Difficulty:** Beginner-Intermediate

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Operating System Setup](#operating-system-setup)
3. [Network Configuration](#network-configuration)
4. [ROS 2 Installation](#ros-2-installation)
5. [Workspace Setup](#workspace-setup)
6. [Package Installation](#package-installation)
7. [Verification](#verification)

---

## Hardware Requirements

### Development PC ("dev")
- Ubuntu 22.04 LTS desktop
- 8GB+ RAM recommended
- 50GB+ free disk space
- WiFi or Ethernet connection
- **Hostname:** dev
- **IP:** 192.168.86.37 (DHCP or static)

### Robot Computer ("hoverbot")
- Raspberry Pi 4/5 (4GB+ RAM recommended)
- Ubuntu 22.04 LTS Server
- 32GB+ microSD card
- WiFi or Ethernet connection
- **Hostname:** hoverbot
- **IP:** 192.168.86.33 (DHCP or static)

### Network
- Both machines on same subnet (e.g., 192.168.86.0/24)
- Router with DHCP (or static IPs configured)
- Low latency preferred (<20ms ping)

---

## Operating System Setup

### Development PC

1. **Install Ubuntu 22.04 Desktop**
   - Download from [ubuntu.com](https://ubuntu.com/download/desktop)
   - Create bootable USB
   - Install with default options
   - Set hostname to `dev` during installation

2. **Update system:**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

3. **Install essential tools:**
   ```bash
   sudo apt install -y git curl wget nano vim openssh-server
   ```

### Robot Computer (Raspberry Pi)

1. **Flash Ubuntu 22.04 Server**
   - Download Ubuntu Server 22.04 for Raspberry Pi
   - Use Raspberry Pi Imager
   - Set hostname to `hoverbot`
   - Enable SSH
   - Configure WiFi during flash (or use Ethernet)

2. **Boot and update:**
   ```bash
   # SSH into Pi from dev PC
   ssh ubuntu@<PI_IP_ADDRESS>
   # Default password: ubuntu (you'll be prompted to change it)
   
   sudo apt update
   sudo apt upgrade -y
   ```

3. **Set permanent hostname:**
   ```bash
   sudo hostnamectl set-hostname hoverbot
   sudo reboot
   ```

---

## Network Configuration

### Step 1: Find IP Addresses

**On dev:**
```bash
hostname -I
# Example output: 192.168.86.37
```

**On hoverbot:**
```bash
hostname -I
# Example output: 192.168.86.33
```

### Step 2: Configure Hostname Resolution

**On dev, edit `/etc/hosts`:**
```bash
sudo nano /etc/hosts
```

Add this line:
```
192.168.86.33 hoverbot
```

**On hoverbot, edit `/etc/hosts`:**
```bash
sudo nano /etc/hosts
```

Add this line:
```
192.168.86.37 dev
```

### Step 3: Verify Connectivity

**From dev:**
```bash
ping hoverbot
# Should see replies with ~5-15ms latency
```

**From hoverbot:**
```bash
ping dev
# Should see replies
```

### Step 4: Set Up Passwordless SSH

**On dev:**
```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t ed25519 -C "your_email@example.com"
# Press Enter to accept defaults

# Copy key to hoverbot
ssh-copy-id ryan@hoverbot
# Enter password when prompted

# Test passwordless login
ssh hoverbot
# Should log in without password
```

### Step 5: Configure SSH Config (Optional but Recommended)

**On dev, create/edit `~/.ssh/config`:**
```bash
nano ~/.ssh/config
```

Add:
```
Host hoverbot
    HostName hoverbot
    User ryan
    ForwardAgent yes
    ForwardX11 yes
```

Now you can just type `ssh hoverbot` without username.

---

## ROS 2 Installation

### On Both Machines (dev and hoverbot)

1. **Set up ROS 2 repository:**
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install ROS 2 Humble:**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   
   # On dev: Full desktop install
   sudo apt install ros-humble-desktop -y
   
   # On hoverbot: Base install (no GUI tools)
   sudo apt install ros-humble-ros-base -y
   ```

3. **Install development tools:**
   ```bash
   sudo apt install -y python3-colcon-common-extensions python3-rosdep
   ```

4. **Initialize rosdep (one-time setup):**
   ```bash
   sudo rosdep init
   rosdep update
   ```

5. **Install additional packages:**
   
   **On dev:**
   ```bash
   sudo apt install -y \
     ros-humble-xacro \
     ros-humble-joint-state-publisher \
     ros-humble-joint-state-publisher-gui \
     ros-humble-rqt \
     ros-humble-rqt-graph
   ```
   
   **On hoverbot:**
   ```bash
   sudo apt install -y \
     ros-humble-xacro \
     ros-humble-joint-state-publisher
   ```

### Configure Bash Environment

**On dev, edit `~/.bashrc`:**
```bash
nano ~/.bashrc
```

Add these lines at the end:
```bash
# ROS 2 Configuration
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

**On hoverbot, edit `~/.bashrc`:**
```bash
nano ~/.bashrc
```

Add these lines at the end:
```bash
# ROS 2 Setup
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

**Apply changes:**
```bash
source ~/.bashrc
```

### Verify ROS 2 Installation

**On both machines:**
```bash
echo $ROS_DOMAIN_ID
# Should output: 42

ros2 pkg list | wc -l
# Dev should show ~287 packages
# Hoverbot should show ~186 packages
```

---

## Workspace Setup

### On Dev PC

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

### On Hoverbot

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Package Installation

### Set Up GitHub SSH (if not already done)

**On dev:**
```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "your_email@example.com"

# Add to GitHub
cat ~/.ssh/id_ed25519.pub
# Copy output and add to GitHub → Settings → SSH Keys
```

**On hoverbot:**
```bash
# Same process
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub
# Add to GitHub
```

### Clone and Build Package

**On dev:**
```bash
cd ~/dev_ws/src
git clone git@github.com:Dasovon/my_package.git my_robot_bringup
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

**On hoverbot:**
```bash
cd ~/robot_ws/src
git clone git@github.com:Dasovon/my_package.git my_package
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

**Note:** Package is named `my_robot_bringup` in `package.xml`, but directory names differ (dev: `my_robot_bringup`, hoverbot: `my_package`). ROS uses package name, not directory name.

---

## Verification

### Test 1: Cross-Machine Node Discovery

**On hoverbot:**
```bash
ros2 launch my_robot_bringup talker.launch.py
```

**On dev:**
```bash
ros2 node list
# Should show: /talker

ros2 topic echo /chatter
# Should see "Hello World" messages
```

Press Ctrl+C to stop.

### Test 2: Launch Listener

**On dev (with talker still running on hoverbot):**
```bash
ros2 launch my_robot_bringup listener.launch.py
```

**Should see:**
```
[INFO] [listener]: I heard: [Hello World: 123]
[INFO] [listener]: I heard: [Hello World: 124]
...
```

### Test 3: Computation Graph Visualization

**On dev:**
```bash
rqt_graph
```

**Should show:**
- `/talker` node
- `/listener` node
- `/chatter` topic connecting them

### Test 4: Service Example

**On dev:**
```bash
ros2 launch my_robot_bringup service_example.launch.py

# In another terminal
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
```

**Should return:**
```
response:
  sum: 25
```

---

## Post-Setup Checklist

✅ Ubuntu 22.04 on both machines  
✅ ROS 2 Humble installed  
✅ Hostname resolution working (`ping hoverbot` from dev)  
✅ Passwordless SSH configured  
✅ `ROS_DOMAIN_ID=42` set on both machines  
✅ Workspaces build successfully  
✅ Cross-machine topic communication verified  
✅ `rqt_graph` shows node connections  

---

## Troubleshooting

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for common issues and solutions.

---

## Next Steps

With base system configured, you're ready for:
1. **RPLIDAR integration** - 2D laser scanner
2. **TF tree setup** - Coordinate transforms
3. **URDF creation** - Robot description
4. **RViz visualization** - Sensor data display

See [TODO.md](TODO.md) for detailed roadmap.

---

## References

- [Articulated Robotics Tutorials](https://articulatedrobotics.xyz/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Ubuntu 22.04 Server Guide](https://ubuntu.com/server/docs)
