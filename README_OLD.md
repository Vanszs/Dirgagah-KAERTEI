# KAERTEI 2025 FAIO Drone System

<div align="center">

<img src="logo-krti25.png" alt="KAERTEI 2025" width="200"/>

### *Production-Ready Autonomous Hexacopter*
### Ubuntu 22.04 & Docker - Zero to Competition

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04_LTS-orange?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker&logoColor=white)](https://docker.com/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python&logoColor=white)](https://python.org/)

<img src="HumbleHawksbill_TransparentBG-NoROS.png" alt="ROS 2 Humble" width="150"/>

</div>

---

## 📝 **Overview**

Sistem drone hexacopter autonomous untuk **KAERTEI 2025 FAIO Competition**. Complete system dengan **26-checkpoint mission**, optimized untuk **Ubuntu 22.04** dan **Docker** deployment.

### 🎯 **Key Features**
- 🚀 **3-Step Setup**: Zero to competition ready
- 🎯 **26 Debug Checkpoints**: Step-by-step mission validation
- 🐧 **Ubuntu 22.04 Optimized**: Full ROS 2 Humble integration
- 🐳 **Docker Ready**: Universal deployment
- ⚡ **Just Commands**: Simple interface for all operations  
- 🎥 **Multi-Camera Vision**: YOLOv8 + OpenCV processing
- 🧭 **GPS + Indoor Navigation**: Hybrid positioning system
- 🔧 **Hardware Integration**: Auto-detection Pixhawk, cameras, sensors

---

## 🚀 **Quick Start**

### **⚡ Ubuntu Setup (Recommended)**
```bash
# 1. Clone & Auto-Setup
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp
just setup            # Complete installation (auto-detects system)

# 2. Validate System  
just test             # Full system validation

# 3. Competition Commands
just debug            # Debug mission (step-by-step)
just run              # Autonomous mission (competition mode)
```

### **🎯 Essential Commands**
| Command | Purpose | When to Use |
|---------|---------|-------------|
| `just setup` | Complete installation | First time only |
| `just test` | System validation | Before competition |
| `just status` | Quick system check | Anytime |
| `just hardware` | Hardware detection | Check connections |
| `just debug` | Step-by-step mission | Development/Testing |
| `just run` | Full autonomous | Competition |
| `just stop` | Emergency stop | Emergency only |

### **🔧 Quick Shortcuts**
```bash
just t     # Quick test
just s     # Quick status  
just d     # Quick debug
just r     # Quick run
just h     # Quick help
```

### **🆘 Competition Day Emergency**
```bash
just emergency    # Reset all systems
just doctor       # Full diagnostics
just help         # Competition guide
```

---
just mission-auto     # Autonomous competition mode
```

### **🐳 Docker Mode (Alternative)**
```bash
# 1. Build Container
just docker-build

# 2. Start Container
just docker-start

# 3. Run Mission in Container  
just docker-mission   # or: just mission (auto-detects Docker)
```

### **🔧 Manual Setup (Legacy)**
```bash
# Ubuntu 22.04 manual setup (if auto-install fails)
./setup_ubuntu22.sh   # Legacy installer
just ubuntu-test      # Ubuntu-specific validation
just ubuntu-mission   # Ubuntu-specific mission
```

### **🎯 Auto-Detection Features**
- **🏗️ Architecture**: ARM64 (Raspberry Pi) vs x86_64 (Desktop)
- **🖥️ Platform**: Ubuntu version validation & optimization  
- **📦 Dependencies**: Hardware-specific package selection
- **🔧 Permissions**: Automatic hardware access configuration
- **🍓 Raspberry Pi**: No NVIDIA/CUDA, GPIO support, memory optimization

---

## 📋 **System Requirements**

| Component | Ubuntu 22.04 | Docker |
|-----------|--------------|--------|
| **OS** | Ubuntu 22.04 LTS | Any OS with Docker |
| **CPU** | Intel/AMD x64, 2+ cores | Intel/AMD x64, 2+ cores |
| **RAM** | 4GB (8GB recommended) | 6GB (8GB recommended) |
| **Storage** | 20GB free | 25GB free |
| **Hardware** | Pixhawk, 3x cameras, GPIO | Same + USB passthrough |

---

## ⚡ **Essential Commands**

| Command | Purpose | Use Case |
|---------|---------|----------|
| `just mission-debug` | Debug mission (26 checkpoints) | Competition practice |
| `just mission-auto` | Full autonomous mission | Competition run |
| `just test-all` | System validation | Post-installation |
| `just status` | System health check | Quick diagnostics |
| `just hardware-check` | Hardware detection | Setup verification |
| `just competition-ready` | Full readiness check | Pre-competition |

---

## 🛠️ **Complete Installation Guide**

### 🐧 **Ubuntu 22.04 Installation (Recommended)**

#### **Prerequisites**
```bash
# Verify Ubuntu version
lsb_release -a
# Should show: Ubuntu 22.04.x LTS

# Update system
sudo apt update && sudo apt upgrade -y
```

#### **Automated Setup**
```bash
# Run complete setup (installs everything)
chmod +x setup_ubuntu22.sh
./setup_ubuntu22.sh

# What this installs:
# ✅ ROS 2 Humble Hawksbill
# ✅ MAVROS with GeographicLib datasets
# ✅ Python packages: OpenCV, PyMAVLink, YOLOv8, PyTorch
# ✅ Just command runner
# ✅ Hardware permissions (dialout, video groups)
# ✅ udev rules for flight controllers
# ✅ Competition workspace build
```

#### **Post-Installation Validation**
```bash
# Quick system check
just status

# Comprehensive validation
just test-all

# Hardware detection
just hardware-check
```

### 🐳 **Docker Installation (Universal)**

#### **Prerequisites**
```bash
# Install Docker (Ubuntu)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Logout and login again

# Install Docker Compose
sudo apt install -y docker-compose
```

#### **Build & Run**
```bash
# Build Docker environment
docker-compose build

# Run interactive container
docker-compose up -d
docker exec -it kaertei2025_hexacopter bash

# Inside container - run mission
source /opt/ros/humble/setup.bash
just mission-debug
```

---

## 🎯 **Competition System - 26 Checkpoint Mission**

### **🎯 Mission Flow Overview**
```
📍 INDOOR PHASE (Checkpoints 1-16) - 62% of mission
├── 🚁 INIT & TAKEOFF (1-2)
│   ├── Initialize flight controller & ARM
│   └── Takeoff to 0.6m altitude
│
├── 📦 ITEM COLLECTION (3-8) 
│   ├── Item 1: Front Camera → Align → Pickup with Front Magnet
│   └── Item 2: Back Camera → Align → Pickup with Back Magnet
│
├── 🎪 DROP OPERATIONS (9-14)
│   ├── Navigate & Turn → Search Dropzone
│   ├── Drop Item 1 (Front) → Ascend
│   └── Drop Item 2 (Back) 
│
└── 🚪 EXIT TRANSITION (15-16)
    ├── Find Exit Gate (Top Camera)
    └── Ascend to 3.0m (Outdoor Mode)

📍 OUTDOOR PHASE (Checkpoints 17-26) - 38% of mission  
├── 🛰️ GPS WAYPOINT 1 (17-19)
│   ├── AUTO mode to Waypoint 1
│   ├── Manual search for outdoor item
│   └── Pickup outdoor item (Front Magnet only)
│
├── 🛰️ GPS WAYPOINT 2 (20-23) 
│   ├── Ascend → AUTO to Waypoint 2
│   ├── Search outdoor dropzone
│   └── Drop outdoor item
│
└── 🏁 FINAL RETURN (24-26)
    ├── Ascend to cruise altitude
    ├── AUTO to Waypoint 3 & Landing
    └── Mission COMPLETED
```

### **📋 Complete 26-Checkpoint Mission Breakdown**

#### **🏠 Indoor Phase (Checkpoints 1-16)**

| No | Checkpoint Name | Action Description | Hardware | Flight Mode | Altitude |
|----|-----------------|-------------------|----------|-------------|----------|
| **1** | `INIT` | 🚁 Initialize systems & ARM flight controller | Flight Controller | STABILIZED | Ground |
| **2** | `TAKEOFF` | 🚀 Takeoff to indoor altitude | Motors | POSITION | 1.0m |
| **3** | `SEARCH_ITEM_1_FRONT` | 🔍 Move forward, search with front camera | Front Camera | POSITION | 1.0m |
| **4** | `ALIGN_ITEM_1` | 🎯 Computer vision alignment with Item 1 | Front Camera + CV | POSITION | 1.0m |
| **5** | `PICKUP_ITEM_1` | 📦 Descend, activate front electromagnet | Front Magnet | POSITION | 0.3m → 1.0m |
| **6** | `SEARCH_ITEM_2_BACK` | 🔍 Continue forward, switch to back camera | Back Camera | POSITION | 1.0m |
| **7** | `ALIGN_ITEM_2` | 🎯 Computer vision alignment with Item 2 | Back Camera + CV | POSITION | 1.0m |
| **8** | `PICKUP_ITEM_2` | 📦 Descend, activate back electromagnet | Back Magnet | POSITION | 0.3m → 1.0m |
| **9** | `NAVIGATE_TURN_DIRECTION` | 🔄 Execute configured turn maneuver | Navigation | POSITION | 1.0m |
| **10** | `SEARCH_DROPZONE` | 🎪 Search for colored dropzone baskets | Front Camera + CV | POSITION | 1.0m |
| **11** | `DROP_ITEM_1_FRONT` | 📦 Release front item into target basket | Front Magnet | POSITION | 0.3m |
| **12** | `ASCEND_AFTER_DROP_1` | ⬆️ Ascend after first drop completed | Motors | POSITION | 1.0m |
| **13** | `ALIGN_DROP_2_BACK` | 🎯 Switch to back camera, align for drop | Back Camera + CV | POSITION | 1.0m |
| **14** | `DROP_ITEM_2_BACK` | 📦 Release back item into target basket | Back Magnet | POSITION | 0.3m |
| **15** | `FIND_EXIT` | 🚪 Locate exit gate using top camera | Top Camera + CV | POSITION | 1.0m |
| **16** | `ASCEND_TO_OUTDOOR` | ⬆️ Climb to outdoor flight altitude | Motors | POSITION | 3.0m |

#### **🌤️ Outdoor Phase (Checkpoints 17-26)**

| No | Checkpoint Name | Action Description | Hardware | Flight Mode | Altitude |
|----|-----------------|-------------------|----------|-------------|----------|
| **17** | `AUTO_WAYPOINT_1` | 🛰️ GPS autonomous flight to Waypoint 1 | GPS + Flight Controller | AUTO | 3.0m |
| **18** | `MANUAL_SEARCH_OUTDOOR` | 🔍 Switch to POSITION, manual search | Front Camera | POSITION | 3.0m |
| **19** | `PICKUP_OUTDOOR` | 📦 Pickup outdoor item (front magnet only) | Front Camera + Magnet | POSITION | 1.0m → 3.0m |
| **20** | `ASCEND_TO_WAYPOINT_2` | ⬆️ Gain altitude for GPS navigation | Motors | POSITION | 3.0m |
| **21** | `AUTO_WAYPOINT_2` | 🛰️ GPS autonomous flight to Waypoint 2 | GPS + Flight Controller | AUTO | 3.0m |
| **22** | `MANUAL_SEARCH_DROP_OUTDOOR` | 🎪 Search for outdoor dropzone target | Front Camera | POSITION | 3.0m |
| **23** | `DROP_OUTDOOR` | 📦 Release outdoor item at target | Front Magnet | POSITION | 1.0m |
| **24** | `ASCEND_TO_WAYPOINT_3` | ⬆️ Final ascent for return journey | Motors | POSITION | 3.0m |
| **25** | `AUTO_WAYPOINT_3_LANDING` | 🏁 GPS flight to landing zone | GPS + Flight Controller | AUTO → LAND | 3.0m → 0m |
| **26** | `COMPLETED` | 🎉 Mission complete, all systems safe | All Systems | DISARMED | Ground |

### **🎯 Mission Statistics & Key Features**

| **Metric** | **Value** | **Details** |
|------------|-----------|-------------|
| **Total Checkpoints** | 26 | Complete autonomous mission |
| **Indoor Tasks** | 16 checkpoints (62%) | Item pickup & drop operations |
| **Outdoor Tasks** | 10 checkpoints (38%) | GPS navigation & outdoor operations |
| **Items Handled** | 3 total | 2 indoor items + 1 outdoor item |
| **Flight Modes** | 4 modes | STABILIZED → POSITION → AUTO → LAND |
| **Camera Usage** | 3 cameras | Front (primary), Back (indoor), Top (exit) |
| **Magnet Control** | 2 electromagnets | Front (all phases), Back (indoor only) |
| **Altitude Levels** | 4 levels | Ground, 0.3m (pickup), 1.0m (indoor), 3.0m (outdoor) |
| **Estimated Duration** | 15-20 minutes | Competition mode with minimal delays |

### **🔧 Mission Control Modes**

#### **🐛 Debug Mode** (`just debug`)
```bash
# Step-by-step mission execution
# Type 'next' + Enter to advance each checkpoint
# Perfect for development and testing
# Full logging and human oversight
```

#### **🚀 Autonomous Mode** (`just run`)  
```bash
# Fully automatic 26-checkpoint execution
# Competition-ready with optimized timing
# Minimal delays, maximum performance
```

### **Debug Features**
- **Type `next`** to proceed to next checkpoint
- **Real-time status** monitoring during mission
- **Step-by-step validation** for each checkpoint
- **Emergency abort** capability with `abort` command

---

## 🔧 **Hardware Configuration**

### **🔗 System Architecture**
```
┌─────────────────────┐    ┌─────────────────────┐
│    PIXHAWK 4        │    │  RASPBERRY PI 5     │
│                     │    │                     │
│ ✅ 6x Motors (ESC)  │    │ ✅ 3x USB Cameras   │
│ ✅ GPS Module       │    │ ✅ 3x LiDAR Sensors │
│ ✅ IMU/Compass      │    │ ✅ 2x Relay Control │
│ ✅ MAVLink/USB      │    │ ✅ GPIO Management  │
└─────────────────────┘    └─────────────────────┘
         │                           │
         └──── USB Connection ───────┘
```

### **🚁 Flight Controller (Pixhawk4)**
**Purpose**: Pure flight control and GPS navigation
- **Motors**: 6x brushless motors via ESC
- **GPS**: External GPS module for waypoint navigation
- **Communication**: MAVLink over USB to Pi 5
- **Flight Modes**: STABILIZED → POSITION → AUTO → LAND

```bash
# Pixhawk4 connection test
ls /dev/tty* | grep -E "(USB|ACM)"
# Expected: /dev/ttyUSB0 or /dev/ttyACM0

# Test MAVLink communication
just test-mavros
```

### **💻 Raspberry Pi 5 - Payload Controller**
**Purpose**: All sensors, cameras, and actuators

#### **📷 Camera System (3x USB)**
- **Front Camera**: `/dev/video0` - Item detection & pickup
- **Back Camera**: `/dev/video2` - Item 2 detection (indoor only)  
- **Top Camera**: `/dev/video4` - Exit gate detection
- **Resolution**: 640x480 @ 30fps (MJPEG hardware decode)

```bash
# Camera system test
v4l2-ctl --list-devices
cheese /dev/video0  # Test front camera
```

#### **📡 LiDAR System (3x TF Mini Plus)**
- **Front LiDAR**: `/dev/ttyUSB1` - Forward obstacle detection
- **Left LiDAR**: `/dev/ttyUSB2` - Left side clearance
- **Right LiDAR**: `/dev/ttyUSB3` - Right side clearance
- **Range**: 0.1m to 12m, 100Hz sampling rate
- **Connection**: USB-to-Serial adapters

```bash
# LiDAR system test
ls /dev/ttyUSB* 
# Expected: ttyUSB0 (Pixhawk), ttyUSB1-3 (LiDAR)

# Test LiDAR communication
screen /dev/ttyUSB1 115200  # Front LiDAR
```

#### **🔌 GPIO Control (Raspberry Pi 5)**
- **Front Magnet Relay**: GPIO 18 (Active LOW)
- **Back Magnet Relay**: GPIO 19 (Active LOW)
- **Status LED**: GPIO 21
- **Error LED**: GPIO 20
- **Emergency Stop**: GPIO 16

```bash
# GPIO test
python3 -c "
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.HIGH)  # Relay OFF
print('Front magnet relay: OFF')
"
```

# Test camera system
just test-cameras
```

#### **3. GPIO Setup (Electromagnets)**
```bash
# For Raspberry Pi GPIO control
# Default pins: GPIO 18 (front magnet), GPIO 19 (back magnet)

# Test GPIO access
python3 -c "import RPi.GPIO; print('GPIO ready')"

# Configure in hardware_config.conf:
[electromagnets]
front_magnet_pin = 18
back_magnet_pin = 19
```

#### **4. Sensor Integration**
```bash
# ToF sensors for obstacle avoidance
# Usually connected via I2C

# Test sensor access
just test-sensors
```

### **Competition Configuration**
Before competition, update these critical settings in `config/hardware_config.conf`:

```ini
[competition_venue]
# ⚠️ CRITICAL: Update GPS coordinates for your venue
outdoor_pickup_latitude = -6.365000    # UPDATE THIS
outdoor_pickup_longitude = 106.825000  # UPDATE THIS
outdoor_drop_latitude = -6.364500      # UPDATE THIS
outdoor_drop_longitude = 106.825500    # UPDATE THIS

[flight_params]
indoor_altitude = 0.6      # Competition height (meters)
outdoor_altitude = 3.0     # Outdoor cruise height
turn_direction = "left"    # or "right" based on arena layout

[hardware]
pixhawk_port = "/dev/ttyACM0"    # Flight controller port
camera_front = 0                 # Front camera index
camera_back = 2                  # Back camera index
camera_top = 4                   # Top camera index
```

---

## 🚨 **Complete Troubleshooting Guide**

### **🎯 Quick Problem Resolution**

| Problem | Symptoms | Quick Fix |
|---------|----------|-----------|
| **ROS 2 not found** | `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| **Permission denied** | USB device access error | `sudo usermod -a -G dialout,video $USER` (logout/login) |
| **MAVROS timeout** | No flight controller response | Check USB connection, try different port |
| **Camera not detected** | No video devices | Check USB 3.0 connection, power supply |
| **Import errors** | Python module missing | `pip3 install -r requirements.txt` |
| **Docker permission** | Docker access denied | `sudo usermod -a -G docker $USER` (logout/login) |
| **Package not found** | `Package 'drone_mvp' not found` | `colcon build --packages-select drone_mvp` |
| **CV2 import fails** | `No module named 'cv2'` | `pip3 install opencv-python` |
| **FCU DeviceError** | Serial connection failed | Check USB cable and permissions |

---

### **🚨 Installation Issues**

#### **ROS 2 Key Errors (Ubuntu)**
```bash
# Error: "The following signatures couldn't be verified"
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

#### **GeographicLib Download Fails**
```bash
# Error: "get-geoids failed" or network timeout
# Manual download solution:
sudo mkdir -p /usr/share/GeographicLib/
sudo wget -O /tmp/geoids.tar.bz2 https://sourceforge.net/projects/geographiclib/files/geoids-distrib/egm96-15.tar.bz2
sudo tar -xjf /tmp/geoids.tar.bz2 -C /usr/share/GeographicLib/
```

#### **MAVROS Installation Issues**
```bash
# Error: GeographicLib datasets missing
sudo apt install ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

#### **OpenCV Installation Issues**
```bash
# Error: cv2 import fails
pip3 uninstall opencv-python opencv-contrib-python
pip3 install opencv-python==4.6.0.66
```

#### **Python Dependencies Conflicts**
```bash
# Error: Version conflicts or module import errors
# Create clean virtual environment:
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate
pip3 install -r requirements.txt
```

---

### **🔌 Hardware Connection Issues**

#### **Flight Controller Not Detected**
```bash
# Check connected devices first
ls /dev/tty{USB,ACM}* 2>/dev/null

# If empty, troubleshoot:
# 1. Check USB cable (use data cable, not charging-only)
# 2. Try different USB port
# 3. Check flight controller power/status LEDs
# 4. Test with QGroundControl

# Check USB devices
lsusb | grep -i -E "(ftdi|px4|pixhawk)"
dmesg | tail | grep tty

# Add user to correct groups:
# Ubuntu/Debian:
sudo usermod -a -G dialout $USER
# Arch Linux:
sudo usermod -a -G uucp $USER
# Then logout and login
```

#### **Permission Denied on Serial Port**
```bash
# Error: "Permission denied: '/dev/ttyUSB0'"
# Temporary fix:
sudo chmod 666 /dev/ttyUSB0

# Permanent fix:
sudo usermod -a -G dialout $USER  # Ubuntu
sudo usermod -a -G uucp $USER     # Arch
# Then logout/login for changes to take effect
```

#### **Camera Not Detected**
```bash
# Check cameras
ls /dev/video*
v4l2-ctl --list-devices

# If no cameras found:
# 1. Check USB connections (use USB 3.0 ports)
# 2. Check camera power (external power if needed)
# 3. Try different USB port
# 4. Test camera compatibility

# Install camera tools:
sudo apt install v4l-utils  # Ubuntu
sudo pacman -S v4l-utils    # Arch

# Test camera directly:
cheese /dev/video0          # GUI test
ffplay /dev/video0          # Command line test
```

#### **GPIO Access (Raspberry Pi)**
```bash
# Check GPIO permissions
sudo usermod -a -G gpio $USER

# Test GPIO directly:
echo 18 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio18/direction
echo 1 > /sys/class/gpio/gpio18/value

# For electromagnet control:
# Verify GPIO pins in hardware_config.conf
# Default: GPIO 18 (front), GPIO 19 (back)
```

---

### **🚀 Launch and Runtime Issues**

#### **MAVROS Connection Failed**
```bash
# Error: "FCU: DeviceError:serial:open: Permission denied"
# Troubleshoot systematically:

# 1. Verify device exists
ls /dev/ttyUSB0

# 2. Check permissions
ls -la /dev/ttyUSB0

# 3. Verify user groups
groups $USER

# 4. Update configuration
nano config/hardware_config.conf
# Update: connection_port = "/dev/ttyUSB0"  # or ttyACM0

# 5. Test direct connection
sudo chmod 666 /dev/ttyUSB0
just test-mavros
```

#### **MAVLink Direct Mode Fails**
```bash
# Error: "Connection refused" or "No heartbeat"
# Try different approaches:

# 1. Check baud rate in config
nano config/hardware_config.conf
# Try: 57600, 115200, 921600

# 2. Test with MAVProxy
pip3 install MAVProxy
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

# 3. Switch to MAVROS fallback
just mission-debug  # Uses MAVROS by default
```

#### **Node Launch Failures**
```bash
# Error: "Package 'drone_mvp' not found"
# Rebuild workspace:
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select drone_mvp
source install/setup.bash

# Error: "No module named 'cv2'"
pip3 install opencv-python ultralytics pymavlink

# Error: Import errors for mavros_msgs
ros2 pkg list | grep mavros
# If empty, reinstall MAVROS:
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

---

### **🎮 Mission Execution Issues**

#### **Checkpoint Stuck or Not Progressing**
```bash
# Debug mode not responding:
# 1. Ensure terminal has focus (click on terminal window)
# 2. Type 'next' and press Enter (not just 'n')
# 3. Check for error messages in output

# Force progression in debug mode:
# Type: next [Enter]

# Switch to autonomous mode for testing:
just mission-auto
```

#### **GPS/Position Issues**
```bash
# Error: "No GPS fix" or "Invalid position"
# Solutions:

# 1. Wait for GPS lock (2-5 minutes outdoor)
# 2. Check GPS antenna connection
# 3. Verify GPS status:
ros2 topic echo /mavros/global_position/global

# 4. Update competition coordinates:
nano config/hardware_config.conf
# Update GPS waypoints for your venue:
# outdoor_pickup_latitude = YOUR_LATITUDE
# outdoor_pickup_longitude = YOUR_LONGITUDE

# 5. Test GPS reception:
# Use QGroundControl to verify GPS status
```

#### **Vision/Camera Issues**
```bash
# Error: "Failed to open camera" or "No camera feed"
# Systematic troubleshooting:

# 1. Check camera devices
ls /dev/video*

# 2. Test cameras individually
ffplay /dev/video0  # Front camera
ffplay /dev/video2  # Back camera
ffplay /dev/video4  # Top camera

# 3. Check OpenCV
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"

# 4. Adjust camera indices if needed
# Edit hardware_config.conf:
# camera_front = 0  # Change if your camera is different index
# camera_back = 2
# camera_top = 4
```

---

### **⚡ Performance Issues**

#### **Slow Response or High CPU Usage**
```bash
# Monitor system resources
htop
top

# Solutions:
# 1. Close unnecessary applications
pkill -f firefox
pkill -f chrome

# 2. Increase swap if low RAM
sudo swapon --show
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 3. Optimize system:
# - Use lightweight desktop (XFCE, LXDE)
# - Disable visual effects
# - Close background applications
```

#### **Network/Communication Lag**
```bash
# Check ROS 2 communication
ros2 topic hz /mavros/state  # Should show ~20Hz

# Network diagnostics
ping -c 4 127.0.0.1
ifconfig

# Solutions:
# 1. Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 2. Check for network conflicts
# 3. Use wired connection if possible
# 4. Disable WiFi power management
```

---

### **🔧 Configuration Issues**

#### **Config File Problems**
```bash
# Error: "Config file not found" or "Invalid configuration"

# 1. Verify file exists
ls -la config/hardware_config.conf

# 2. Check file permissions
chmod 644 config/hardware_config.conf

# 3. Validate syntax
cat config/hardware_config.conf | grep -E "^[^#].*=" | head -10

# 4. Reset to defaults if corrupted
cp config/hardware_config.conf.example config/hardware_config.conf
```

#### **Environment Variables**
```bash
# Error: "ROS_DOMAIN_ID not set" or "PYTHONPATH missing"

# 1. Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. Make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 3. Verify environment
printenv | grep ROS
printenv | grep PYTHON
```

---

### **🆘 Emergency Procedures**

#### **Competition Day Emergency Reset (5 minutes)**
```bash
cd ~/ros2_ws/src/drone_mvp/

# 1. Kill all processes
sudo pkill -f ros2
sudo pkill -f mavros
sudo pkill -f python3

# 2. Reset USB devices
sudo rmmod usbserial
sudo modprobe usbserial

# 3. Reset permissions
sudo chmod 666 /dev/tty{USB,ACM}*

# 4. Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 5. Quick system validation
just status
just hardware-check

# 6. Relaunch mission
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
just mission-debug
```

#### **Fallback Modes**
```bash
# If MAVROS fails - try direct MAVLink:
just mission-debug  # Will auto-fallback to MAVLink

# If all fails - minimal test:
python3 -c "
import pymavlink.mavutil as mavutil
conn = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
print('MAVLink test OK' if conn else 'MAVLink failed')
"

# Last resort - manual control:
# Use QGroundControl or RC transmitter
```

---

### **📊 Diagnostic Commands**

#### **System Health Check**
```bash
# Complete system validation
just test-all

# Individual system checks
just status           # Quick overview
just hardware-check   # Hardware detection
just test-mavros     # Flight controller
just test-cameras    # Camera system

# ROS 2 diagnostics
ros2 doctor          # ROS 2 health check
ros2 topic list      # Active topics
ros2 node list       # Running nodes
ros2 pkg list | grep drone_mvp  # Package status
```

#### **Hardware Diagnostics**
```bash
# USB device detection
lsusb | grep -i -E "(ftdi|px4|pixhawk|camera)"

# Serial device check
ls -la /dev/tty{USB,ACM,AMA}* 2>/dev/null

# Camera device check
v4l2-ctl --list-devices
ls -la /dev/video*

# System messages
dmesg | tail -20
journalctl -u ros2* --since "10 minutes ago"
```

#### **Network Diagnostics**
```bash
# ROS 2 network discovery
ros2 multicast send    # Test multicast
ros2 multicast receive # Listen for responses

# Network interfaces
ip addr show
route -n

# Communication test
ros2 topic echo /mavros/state --once
```

---

### **📞 Getting Help & Log Collection**

#### **Collecting Support Logs**
```bash
# Create timestamped log directory
mkdir -p ~/kaertei_logs/$(date +%Y%m%d_%H%M%S)
cd ~/kaertei_logs/$(date +%Y%m%d_%H%M%S)

# System information
uname -a > system_info.txt
lsb_release -a >> system_info.txt
cat /proc/version >> system_info.txt

# ROS 2 environment
printenv | grep ROS > ros_env.txt
ros2 topic list > ros_topics.txt 2>/dev/null || echo "ROS 2 not running" > ros_topics.txt
ros2 node list > ros_nodes.txt 2>/dev/null || echo "No nodes running" > ros_nodes.txt

# Hardware information
lsusb > hardware.txt
lspci >> hardware.txt
ls -la /dev/tty* >> hardware.txt
ls -la /dev/video* >> hardware.txt

# Configuration files
cp ~/ros2_ws/src/drone_mvp/config/hardware_config.conf . 2>/dev/null || echo "Config not found"

# System logs
dmesg > dmesg.log
journalctl --since "1 hour ago" > system.log

echo "📋 Support logs collected in: $(pwd)"
echo "📎 Attach these files when requesting help"
```

#### **Error Message Quick Reference**
| Error Pattern | Likely Cause | Quick Fix |
|---------------|--------------|-----------|
| `Permission denied: '/dev/ttyUSB0'` | User not in dialout group | `sudo usermod -a -G dialout $USER` |
| `Package 'drone_mvp' not found` | Workspace not built | `colcon build --packages-select drone_mvp` |
| `No module named 'cv2'` | OpenCV not installed | `pip3 install opencv-python` |
| `FCU: DeviceError:serial:open` | Flight controller connection | Check USB cable and port |
| `rosdep: command not found` | Missing rosdep (normal on Arch) | Ignore - dependencies handled separately |
| `Failed to open camera` | Camera not available | Check `/dev/video*` and permissions |
| `No heartbeat received` | MAVLink communication issue | Check baud rate and connection |
| `GeographicLib datasets missing` | MAVROS datasets not installed | Run GeographicLib install script |

---

### **🎯 Prevention & Best Practices**

#### **Pre-Competition Checklist**
- [ ] **Test all hardware connections** 24 hours before competition
- [ ] **Backup working configuration** files
- [ ] **Document your specific hardware setup** (ports, indices)
- [ ] **Practice emergency procedures** with team
- [ ] **Test both MAVROS and MAVLink modes**
- [ ] **Verify GPS coordinates** for competition venue
- [ ] **Check camera focus and exposure** settings
- [ ] **Monitor system resources** during extended runs
- [ ] **Prepare backup hardware** (cables, SD cards, cameras)

#### **Competition Day Setup**
1. **Arrive early** for hardware setup and testing
2. **Run full system validation** before first flight
3. **Keep backup equipment** readily available
4. **Document any configuration changes** made on-site
5. **Test emergency stop procedures** before competition flights

**⚠️ Remember: When in doubt, restart the system and use debug mode!** 🚁

---

## 📊 **System Architecture**

### **Software Stack**
```
┌─────────────────────────────────────────┐
│     Mission Control (Python)           │ ← 26-Checkpoint FSM
├─────────────────────────────────────────┤
│        ROS 2 Nodes Network             │ ← Node Coordination  
├─────────────────────────────────────────┤
│           MAVLink Bridge               │ ← MAVROS Communication
├─────────────────────────────────────────┤
│        PX4 Firmware                    │ ← Flight Control Logic
├─────────────────────────────────────────┤
│      Pixhawk PX4 Standard              │ ← Hardware Control
└─────────────────────────────────────────┘
```

### **Node Architecture**
| Node | Technology | Function |
|------|------------|----------|
| `checkpoint_mission_node.py` | **Python FSM** | Main mission control (26 checkpoints) |
| `camera_control_node.py` | **OpenCV** | Multi-camera management |
| `vision_detector_node.py` | **YOLOv8** | Object detection system |
| `magnet_control_node.py` | **GPIO** | Electromagnet control |
| `sensor_monitor.py` | **I2C/ToF** | Sensor integration |

### **Hardware Architecture**
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Mission PC     │    │  Pixhawk PX4    │    │  Hexacopter     │
│  (ROS 2 Humble) │◄───┤  (ArduPilot)    │◄───┤  (6 Motors)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  3x Cameras     │    │  GPS/IMU/Sensor │    │  2x Electromagn │
│  (Vision Sys)   │    │  (Navigation)   │    │  (Pickup/Drop)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

---

## 🏆 **Competition Day Checklist**

### **Pre-Competition Setup**
- [ ] **Hardware connections tested** (`just hardware-check`)
- [ ] **GPS coordinates updated** (competition venue specific)
- [ ] **Flight altitude configured** (indoor: 0.6m, outdoor: 3.0m)
- [ ] **Camera indices verified** (front/back/top cameras)
- [ ] **Electromagnet pins configured** (GPIO 18, 19)
- [ ] **Mission simulation successful** (`just test-mission`)
- [ ] **Full system validation passed** (`just test-all`)
- [ ] **Emergency procedures reviewed** (`just emergency-stop`)
- [ ] **Backup configurations created** (`just backup-config`)

### **Competition Workflow**
1. **Setup**: `just setup` - Complete system setup
2. **Validate**: `just test-all` - Verify all systems ready  
3. **Practice**: `just mission-debug` - Practice with checkpoints
4. **Compete**: `just mission-auto` - Full autonomous run
5. **Emergency**: `just emergency-stop` - Safety abort

---

## 🤝 **Support**

- 🐛 **Issues**: [GitHub Issues](https://github.com/Vanszs/Dirgagah-KAERTEI/issues)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/Vanszs/Dirgagah-KAERTEI/discussions)
- 📖 **Wiki**: [Complete Documentation](https://github.com/Vanszs/Dirgagah-KAERTEI/wiki)

### **Quick Help Commands**
```bash
just help           # Essential commands guide
just help-mission   # Mission-specific help  
just help-hardware  # Hardware setup help
just --list         # Show all available commands
```

---

## 🎖️ **Credits**

### **Development Team**
- **KAERTEI 2025 Development Team**
- **Indonesian Drone Enthusiasts Community**

### **Technology Stack**
- **ROS 2 Humble Hawksbill** - Robot Operating System
- **Ubuntu 22.04 LTS** - Operating System
- **OpenCV** - Computer Vision Library
- **YOLOv8** - Object Detection Model
- **PyTorch** - Deep Learning Framework
- **MAVROS** - MAVLink ROS Interface
- **ArduPilot** - Flight Controller Firmware

---

<div align="center">

## 🏆 **Ready for KAERTEI 2025 Competition!**
*Built with ❤️ by Indonesian Drone Enthusiasts*

[![GitHub](https://img.shields.io/github/stars/Vanszs/Dirgagah-KAERTEI?style=social)](https://github.com/Vanszs/Dirgagah-KAERTEI)

</div>
