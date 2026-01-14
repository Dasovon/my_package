# Troubleshooting Guide

Common issues and solutions for the Hoverbot ROS 2 system.

---

## Network and Connectivity Issues

### Nodes Not Discovering Each Other

**Symptoms:** `ros2 node list` only shows local nodes, can't see nodes on other machine

**Solutions:**

1. **Check ROS_DOMAIN_ID match:**
   ```bash
   # On both machines
   echo $ROS_DOMAIN_ID
   # Should both output: 42
   ```

2. **Verify network connectivity:**
   ```bash
   ping hoverbot  # From dev
   ping dev       # From hoverbot
   # Should see replies with low latency (<20ms)
   ```

3. **Restart ROS daemon:**
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

4. **Check firewall:**
   ```bash
   sudo ufw status
   # Should be inactive, or allow ROS multicast traffic
   ```

5. **Verify /etc/hosts configuration:**
   ```bash
   cat /etc/hosts | grep hoverbot  # On dev
   cat /etc/hosts | grep dev       # On hoverbot
   ```

---

### Topics Visible But No Messages Received

**Symptoms:** `ros2 topic list` shows topic, but `ros2 topic echo` prints nothing

**Solutions:**

1. **Check QoS compatibility:**
   ```bash
   ros2 topic info /topic_name -v
   # Verify publisher and subscriber QoS match
   ```

2. **Verify publisher/subscriber counts:**
   ```bash
   ros2 topic info /chatter
   # Should show: Publisher count: 1, Subscription count: 1
   ```

3. **Test network latency:**
   ```bash
   ping -c 100 hoverbot
   # Should be <100ms, <5% packet loss
   ```

4. **Check for multicast blocking:**
   - Rare issue, usually on corporate/school networks
   - Try connecting both machines via Ethernet or dedicated router

---

### SSH Connection Issues

**Symptoms:** Can't SSH into hoverbot

**Solutions:**

1. **Verify SSH service is running:**
   ```bash
   # On hoverbot
   sudo systemctl status ssh
   # Should show "active (running)"
   ```

2. **Check SSH key permissions:**
   ```bash
   # On dev
   ls -l ~/.ssh/
   # id_ed25519 should be 600, id_ed25519.pub should be 644
   ```

3. **Re-copy SSH key:**
   ```bash
   ssh-copy-id ryan@hoverbot
   ```

---

## Build and Package Issues

### colcon build Fails

**Symptoms:** `colcon build` returns errors

**Solutions:**

1. **Check for missing dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Clean build:**
   ```bash
   cd ~/dev_ws
   rm -rf build/ install/ log/
   colcon build --symlink-install
   ```

3. **Check CMakeLists.txt syntax:**
   - Ensure `install()` directives are before `ament_package()`
   - Verify all referenced directories exist

4. **Check package.xml dependencies:**
   - All `exec_depend` packages must be installed
   - Run: `ros2 pkg list | grep <dependency_name>`

---

### Launch File Not Found

**Symptoms:** `ros2 launch my_robot_bringup file.launch.py` says "package not found"

**Solutions:**

1. **Source workspace:**
   ```bash
   source ~/dev_ws/install/setup.bash  # On dev
   source ~/robot_ws/install/setup.bash # On hoverbot
   ```

2. **Verify package built:**
   ```bash
   ros2 pkg list | grep my_robot_bringup
   # Should output: my_robot_bringup
   ```

3. **Check install directive in CMakeLists.txt:**
   ```cmake
   install(DIRECTORY launch
     DESTINATION share/${PROJECT_NAME}
   )
   ```

4. **Verify files installed:**
   ```bash
   ls install/my_robot_bringup/share/my_robot_bringup/launch/
   # Should show your launch files
   ```

---

## ROS 2 Configuration Issues

### ROS_DOMAIN_ID Not Persisting

**Symptoms:** `echo $ROS_DOMAIN_ID` returns nothing or wrong value

**Solutions:**

1. **Check .bashrc order:**
   ```bash
   cat ~/.bashrc
   # export ROS_DOMAIN_ID=42 must come AFTER source /opt/ros/humble/setup.bash
   ```

2. **Correct order:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/dev_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   export ROS_LOCALHOST_ONLY=0
   ```

3. **Re-source .bashrc:**
   ```bash
   source ~/.bashrc
   ```

4. **Open new terminal to test:**
   ```bash
   echo $ROS_DOMAIN_ID
   # Should output: 42
   ```

---

### Auto-Sourcing Not Working

**Symptoms:** Must manually source workspace in every new terminal

**Solutions:**

1. **Add to .bashrc:**
   ```bash
   nano ~/.bashrc
   # Add these lines at the end
   source /opt/ros/humble/setup.bash
   source ~/dev_ws/install/setup.bash  # Or ~/robot_ws/install/setup.bash
   ```

2. **Test in new terminal:**
   ```bash
   # Open new terminal
   ros2 topic list
   # Should work without manual sourcing
   ```

---

## Raspberry Pi Specific Issues

### WiFi Disconnects After Idle

**Symptoms:** SSH drops after 30s-5min of inactivity

**Solutions:**

1. **Disable WiFi power management:**
   ```bash
   # On hoverbot
   sudo iwconfig wlan0 power off
   ```

2. **Make permanent:**
   ```bash
   sudo nano /etc/rc.local
   # Add before "exit 0":
   /sbin/iwconfig wlan0 power off
   ```

3. **Alternative - use Ethernet:**
   - More stable for robot operations
   - Eliminates WiFi power management issues

---

### SD Card Corruption

**Symptoms:** Pi won't boot, filesystem errors

**Prevention:**

1. **Use high-quality SD card (A2 rated)**
2. **Proper shutdown:**
   ```bash
   sudo shutdown -h now
   # Wait for green LED to stop blinking before unplugging
   ```

3. **Backup regularly:**
   ```bash
   # On dev (with hoverbot shutdown and SD card in reader)
   sudo dd if=/dev/sdX of=~/hoverbot_backup.img bs=4M status=progress
   ```

---

## Git and Version Control Issues

### Git Push/Pull Fails

**Symptoms:** Permission denied or authentication error

**Solutions:**

1. **Check SSH key added to GitHub:**
   ```bash
   cat ~/.ssh/id_ed25519.pub
   # Copy and verify it's in GitHub → Settings → SSH Keys
   ```

2. **Test GitHub SSH:**
   ```bash
   ssh -T git@github.com
   # Should output: "Hi <username>! You've successfully authenticated..."
   ```

3. **Verify remote URL:**
   ```bash
   cd ~/dev_ws/src/my_robot_bringup
   git remote -v
   # Should show: git@github.com:Dasovon/my_package.git
   ```

4. **Switch to SSH URL if using HTTPS:**
   ```bash
   git remote set-url origin git@github.com:Dasovon/my_package.git
   ```

---

### Merge Conflicts

**Symptoms:** Git reports conflicts during pull

**Solutions:**

1. **View conflicts:**
   ```bash
   git status
   # Shows files with conflicts
   ```

2. **Edit conflicted files:**
   ```bash
   nano <filename>
   # Look for <<<<<<< HEAD markers
   # Keep desired changes, remove conflict markers
   ```

3. **Mark as resolved:**
   ```bash
   git add <filename>
   git commit -m "Resolved merge conflict"
   ```

4. **Prevention - commit before pulling:**
   ```bash
   git add .
   git commit -m "Save local changes"
   git pull
   ```

---

## Performance Issues

### High CPU Usage on Raspberry Pi

**Solutions:**

1. **Check running nodes:**
   ```bash
   top
   # Look for processes using >50% CPU
   ```

2. **Reduce sensor rates:**
   - Lower RPLIDAR scan rate
   - Reduce camera resolution/frame rate
   - Limit topic publishing rates

3. **Disable unnecessary services:**
   ```bash
   sudo systemctl disable bluetooth
   sudo systemctl disable avahi-daemon
   ```

---

### Slow Network Performance

**Solutions:**

1. **Check latency:**
   ```bash
   ping -c 100 hoverbot
   # Should be <20ms average, <5% loss
   ```

2. **Use 5GHz WiFi instead of 2.4GHz**
   - Less interference
   - Higher bandwidth

3. **Consider Ethernet:**
   - Most reliable for robot operations
   - No wireless interference

---

## Diagnostic Commands Reference

### Network Diagnostics
```bash
ping <hostname>              # Test connectivity
traceroute <hostname>        # Show network path
ifconfig                     # Show network interfaces
iwconfig                     # Show WiFi status
```

### ROS 2 Diagnostics
```bash
ros2 node list               # Show active nodes
ros2 topic list              # Show active topics
ros2 topic echo /topic       # Display messages
ros2 topic hz /topic         # Check message rate
ros2 topic info /topic -v    # Detailed topic info
ros2 daemon stop && ros2 daemon start  # Restart daemon
```

### System Diagnostics
```bash
df -h                        # Disk space
free -h                      # RAM usage
top                          # Process monitor
dmesg | tail -50             # Kernel messages
systemctl status <service>   # Service status
```

---

## Getting Help

If issues persist:

1. **Check GitHub Issues:** https://github.com/Dasovon/my_package/issues
2. **ROS Answers:** https://answers.ros.org/
3. **Articulated Robotics Discord:** Community support
4. **Create detailed issue report:**
   - Exact error message
   - Commands run before error
   - Output of diagnostic commands
   - ROS distro and Ubuntu version
