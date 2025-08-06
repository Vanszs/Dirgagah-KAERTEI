# 🛠️ KAERTEI 2025 FAIO - Troubleshooting Guide

## 🎯 **Quick Problem Resolution**

### **🚨 Installation Issues**

#### **Problem: ROS 2 Key Errors (Ubuntu/Debian)**
```bash
# Error: "The following signatures couldn't be verified"
# Solution:
sudo apt-key del 421C365BD9FF1F717815A3895523# 1. Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/kaertei_drone/install/setup.bash

# 2. Add to shell profile
echo "source ~/kaertei_drone/install/setup.bash" >> ~/.bashrc1FA116
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

#### **Problem: yay Not Found (Arch Linux)**
```bash
# Error: "yay: command not found"
# Solution: Install AUR helper
git clone https://aur.archlinux.org/yay.git /tmp/yay
cd /tmp/yay && makepkg -si
```

#### **Problem: rosdep Fails on Arch Linux**
```bash
# Error: "rosdep: command not found" or dependency errors
# Solution: This is expected - dependencies handled by pacman
rosdep install --from-paths src --ignore-src -r -y || true
echo "rosdep errors on Arch are normal - dependencies installed via pacman"
```

#### **Problem: GeographicLib Download Fails**
```bash
# Error: "get-geoids failed" or network timeout
# Solution: Manual download
sudo mkdir -p /usr/share/GeographicLib/
sudo wget -O /tmp/geoids.tar.bz2 https://sourceforge.net/projects/geographiclib/files/geoids-distrib/egm96-15.tar.bz2
sudo tar -xjf /tmp/geoids.tar.bz2 -C /usr/share/GeographicLib/
```

---

### **🔌 Hardware Connection Issues**

#### **Problem: Flight Controller Not Detected**
```bash
# Check connected devices
ls /dev/tty{USB,ACM}* 2>/dev/null

# If empty, check:
# 1. USB cable (try different cable)
# 2. USB port (try different port)  
# 3. Flight controller power
# 4. Cable quality (data cables, not charging-only)

# Add user to serial groups
# Ubuntu/Debian:
sudo usermod -a -G dialout $USER

# Arch Linux:
sudo usermod -a -G uucp $USER

# Logout and login after group changes
```

#### **Problem: Permission Denied on Serial Port**
```bash
# Error: "Permission denied: '/dev/ttyUSB0'"
# Solution:
sudo chmod 666 /dev/ttyUSB0  # Temporary fix
# OR
sudo usermod -a -G dialout $USER  # Permanent fix (Ubuntu)
sudo usermod -a -G uucp $USER     # Permanent fix (Arch)
# Then logout/login
```

#### **Problem: Camera Not Detected**
```bash
# Check cameras
ls /dev/video*
v4l2-ctl --list-devices

# If no cameras found:
# 1. Check USB connections
# 2. Check camera power (if external power needed)
# 3. Try different USB port
# 4. Check camera compatibility

# Install camera tools
# Ubuntu/Debian:
sudo apt install v4l-utils

# Arch Linux:
sudo pacman -S v4l-utils
```

---

### **🚀 Launch and Runtime Issues**

#### **Problem: MAVROS Connection Failed**
```bash
# Error: "FCU: DeviceError:serial:open: Permission denied"
# Check in this order:

# 1. Check device exists
ls /dev/ttyUSB0

# 2. Check permissions
ls -la /dev/ttyUSB0

# 3. Check user groups
groups $USER

# 4. Update config file
nano config/hardware_config.conf
# Update: connection_port = "/dev/ttyUSB0"  # or ACM0

# 5. Test direct connection
sudo chmod 666 /dev/ttyUSB0
./run_checkpoint_mission.sh debug mavros
```

#### **Problem: MAVLink Direct Mode Fails**
```bash
# Error: "Connection refused" or "No heartbeat"
# Solutions:

# 1. Check baud rate in config
nano config/hardware_config.conf
# Try different baud rates: 57600, 115200, 921600

# 2. Test with MAVProxy
pip3 install MAVProxy
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

# 3. Switch to MAVROS mode
./run_checkpoint_mission.sh debug mavros
```

#### **Problem: Node Launch Failures**
```bash
# Error: "Package 'kaertei_drone' not found"
# Solution: Build workspace
source /opt/ros/humble/setup.bash
cd ~/kaertei_drone
colcon build --packages-select kaertei_drone
source install/setup.bash

# Error: "No module named 'cv2'"
# Solution: Install Python dependencies
pip3 install opencv-python ultralytics pymavlink

# Error: Import errors for mavros_msgs
# Solution: Check MAVROS installation
ros2 pkg list | grep mavros
# If empty, reinstall MAVROS
./install_mavros.sh
```

---

### **🎮 Mission Execution Issues**

#### **Problem: Checkpoint Stuck or Not Progressing**
```bash
# Debug mode not responding to input
# Check:

# 1. Terminal focus (click on terminal)
# 2. Input method (press Enter after 'n')
# 3. Check for error messages in output

# Force next checkpoint:
# In debug mode, press: n + Enter

# Switch to auto mode for testing:
./run_checkpoint_mission.sh auto mavros
```

#### **Problem: GPS/Position Issues**
```bash
# Error: "No GPS fix" or "Invalid position"
# Solutions:

# 1. Check GPS connection and antenna
# 2. Wait for GPS lock (can take 2-5 minutes)
# 3. Check GPS configuration in QGroundControl
# 4. Verify waypoints in config:
nano config/hardware_config.conf
# Update GPS waypoints for your test location

# 5. Test GPS status:
ros2 topic echo /mavros/global_position/global  # MAVROS mode
# OR check MAVLink GPS status
```

#### **Problem: Vision/Camera Issues**
```bash
# Error: "Failed to open camera" or "No camera feed"
# Solutions:

# 1. Check camera connections
ls /dev/video*

# 2. Test camera directly
cheese  # Ubuntu GUI test
# OR
ffplay /dev/video0  # Command line test

# 3. Check OpenCV installation
python3 -c "import cv2; print(cv2.__version__)"

# 4. Adjust camera index in code if needed
# (Camera might be /dev/video1 instead of /dev/video0)
```

---

### **⚡ Performance Issues**

#### **Problem: Slow Response or High CPU Usage**
```bash
# Check system resources
htop
# OR
top

# Solutions:
# 1. Close unnecessary applications
# 2. Increase swap if low RAM
sudo swapon --show
sudo fallocate -l 2G /swapfile  # Create 2GB swap if needed

# 3. Use lightweight desktop environment
# 4. Disable visual effects

# 5. Check for background processes
ps aux | grep -E "(gazebo|rviz|rqt)"
```

#### **Problem: Network/Communication Lag**
```bash
# Check ROS 2 network
ros2 topic hz /mavros/state  # Should show ~20Hz

# Check system network
ping -c 4 127.0.0.1

# Solutions:
# 1. Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 2. Check for network conflicts
# 3. Use wired connection if possible
```

---

### **🔧 Configuration Issues**

#### **Problem: Config File Not Found or Invalid**
```bash
# Error: "Config file not found" or "Invalid configuration"
# Solution:

# 1. Check file exists
ls -la config/hardware_config.conf

# 2. Reset to defaults
cp config/hardware_config.conf.backup config/hardware_config.conf

# 3. Validate config syntax
cat config/hardware_config.conf | grep -E "^[^#].*="

# 4. Common config fixes:
nano config/hardware_config.conf
# Check for:
# - Correct file paths
# - Valid IP addresses  
# - Proper boolean values (true/false)
# - No trailing spaces
```

#### **Problem: Environment Variables Not Set**
```bash
# Error: "ROS_DOMAIN_ID not set" or "PYTHONPATH missing"
# Solution:

# 1. Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/kaertei_drone/install/setup.bash

# 2. Add to shell profile
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/kaertei_drone/install/setup.bash" >> ~/.bashrc

# 3. Check current environment
printenv | grep ROS
```

---

### **📊 Diagnostic Commands**

#### **System Health Check**
```bash
# Run master installer diagnostic
./install_kaertei.sh
# Select option 7 (View System Status)

# Manual checks
ros2 doctor  # ROS 2 health check
ros2 topic list  # Active topics
ros2 node list   # Active nodes
colcon list      # Workspace packages
```

#### **Hardware Diagnostic**
```bash
# Master installer hardware check
./install_kaertei.sh  
# Select option 8 (Hardware Detection)

# Manual hardware checks
lsusb  # USB devices
lspci  # PCI devices  
dmesg | tail -20  # Recent hardware messages
systemctl status  # System services
```

#### **Network Diagnostic**
```bash
# ROS 2 network discovery
ros2 multicast send  # Send test multicast
ros2 multicast receive  # Listen for multicast

# Check network interfaces
ip addr show
ping -c 4 8.8.8.8  # Internet connectivity
```

---

### **🆘 Emergency Procedures**

#### **Competition Day Emergency Reset**
```bash
# Complete system reset (5 minutes)
cd ~/kaertei_drone/

# 1. Kill all processes
pkill -f ros2
pkill -f mavros
pkill -f python3

# 2. Reset USB devices
sudo rmmod usbserial
sudo modprobe usbserial

# 3. Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 4. Quick relaunch
source /opt/ros/humble/setup.bash
source ~/kaertei_drone/install/setup.bash
./run_checkpoint_mission.sh debug mavros
```

#### **Fallback Mode (If MAVROS Fails)**
```bash
# Switch to direct MAVLink mode
./run_checkpoint_mission.sh debug mavlink

# If that fails, minimal Python test:
python3 scripts/test_mavlink_connection.py
```

#### **Last Resort: Manual Control**
```bash
# Launch QGroundControl for manual flight
qgroundcontrol
# OR
# Use RC transmitter for manual override
```

---

### **📞 Getting Help**

#### **Log Collection for Support**
```bash
# Collect all relevant logs
mkdir -p ~/drone_logs/$(date +%Y%m%d_%H%M%S)
cd ~/drone_logs/$(date +%Y%m%d_%H%M%S)

# System logs
dmesg > dmesg.log
journalctl -u ros2* > ros2.log 2>/dev/null || true
ros2 topic list > topics.log 2>/dev/null || true
ros2 node list > nodes.log 2>/dev/null || true

# Configuration
cp ~/kaertei_drone/config/hardware_config.conf .

# Hardware info
lsusb > hardware.log
lspci >> hardware.log
ls -la /dev/tty* >> hardware.log

echo "Logs collected in: $(pwd)"
```

#### **Common Error Patterns**
| Error Message | Quick Fix |
|---------------|-----------|
| `Permission denied: '/dev/ttyUSB0'` | `sudo usermod -a -G dialout $USER` |
| `Package 'kaertei_drone' not found` | `colcon build --packages-select kaertei_drone` |
| `No module named 'cv2'` | `pip3 install opencv-python` |
| `FCU: DeviceError:serial:open` | Check USB cable and permissions |
| `rosdep: command not found` | Normal on Arch Linux - ignore |
| `The following signatures couldn't be verified` | Update ROS 2 GPG keys |

---

## 🎯 **Prevention Tips**

1. **Always test hardware connections before competition**
2. **Keep backup USB cables and SD cards**
3. **Document your specific hardware configuration**
4. **Practice emergency procedures**
5. **Test both MAVROS and MAVLink modes**
6. **Verify GPS coordinates for competition venue**
7. **Check camera focus and exposure settings**
8. **Monitor system resources during extended runs**

**Remember**: When in doubt, restart the system and use debug mode! 🚁
