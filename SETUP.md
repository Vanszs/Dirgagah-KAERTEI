# KAERTEI 2025 FAIO - Complete Setup Guide

<div align="center">

<img src="logo-krti25.png" alt="KAERTEI 2025" width="200"/>

# KAERTEI 2025 FAIO Drone System
## *Ubuntu 22.04 & Docker - Zero to Competition Ready*

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04_LTS-orange?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker&logoColor=white)](https://docker.com/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python&logoColor=white)](https://python.org/)

<img src="HumbleHawksbill_TransparentBG-NoROS.png" alt="ROS 2 Humble" width="150"/>

*Production-Ready Autonomous Hexacopter System*

</div>

---

## 📝 **Project Overview**

Sistem drone hexacopter autonomous yang komprehensif untuk KAERTEI 2025 Divisi Fully Autonomous Indoor-Outdoor (FAIO). Dioptimalkan untuk **Ubuntu 22.04 LTS** dengan setup otomatis dari nol hingga siap kompetisi dalam **3 langkah sederhana**.

### 🎯 **Key Features**
- **✅ Ubuntu 22.04 Optimized**: Instalasi otomatis ROS 2 Humble + MAVROS
- **✅ Zero to Competition**: 3 langkah setup dari sistem kosong hingga siap lomba  
- **✅ 26 Checkpoint System**: Sistem debug lengkap dengan validasi otomatis
- **✅ Docker Support**: Universal deployment di semua platform
- **✅ Just Commands**: Interface sederhana untuk semua operasi
- **✅ Hardware Integration**: Auto-detection Pixhawk, kamera, sensor
- **✅ Vision System**: YOLOv8 + OpenCV multi-camera processing

---

## 🚀 **Quick Start (3 Steps)**

### **Option 1: Ubuntu 22.04 (Recommended)**
```bash
# Step 1: Clone & Setup
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp
./setup_ubuntu22.sh

# Step 2: Validate System
just test-all

# Step 3: Run Competition
just mission-debug    # Debug mode with checkpoints
just mission-auto     # Full autonomous mode
```

### **Option 2: Docker (Universal)**
```bash
# Step 1: Clone & Build
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp
docker-compose up --build

# Step 2: Run Mission
docker exec -it kaertei2025_hexacopter bash
just mission-debug
```

---

## 📋 **System Requirements**

### **Minimum Requirements**
| Component | Ubuntu 22.04 | Docker |
|-----------|--------------|--------|
| **OS** | Ubuntu 22.04 LTS | Any OS with Docker |
| **CPU** | Intel/AMD x64, 2+ cores | Intel/AMD x64, 2+ cores |
| **RAM** | 4GB (8GB recommended) | 6GB (8GB recommended) |
| **Storage** | 20GB free space | 25GB free space |
| **Network** | Internet for dependencies | Internet for Docker images |

### **Hardware Compatibility**
- ✅ **Flight Controller**: Pixhawk PX4/ArduPilot
- ✅ **Cameras**: USB 3.0 cameras (3x)
- ✅ **Sensors**: ToF distance sensors
- ✅ **GPIO**: Raspberry Pi GPIO for electromagnets
- ✅ **GPS**: U-blox or compatible GPS modules

---

## 🛠️ **Installation Guide**

### 🐧 **Ubuntu 22.04 Installation (Recommended)**

#### **Prerequisites**
```bash
# Verify Ubuntu version
lsb_release -a
# Should show: Ubuntu 22.04.x LTS

# Update system
sudo apt update && sudo apt upgrade -y

# Install basic tools
sudo apt install -y git curl wget build-essential
```

#### **Automated Setup**
```bash
# Clone repository
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp

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

# Ready for competition
just competition-ready
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
# Clone repository
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp

# Build Docker environment
docker-compose build

# Run interactive container
docker-compose up -d
docker exec -it kaertei2025_hexacopter bash

# Inside container - run mission
source /opt/ros/humble/setup.bash
just mission-debug
```

#### **Docker Features**
- 🔒 **Isolated Environment**: No system pollution
- 🌍 **Cross-Platform**: Windows, macOS, Linux
- 📦 **All Dependencies**: Pre-installed and configured
- 🔌 **Hardware Passthrough**: USB devices, cameras
- 🚀 **Quick Deployment**: One command deployment

---

## ⚡ **Just Commands Reference**

KAERTEI 2025 menggunakan **Just** command runner untuk mempermudah operasi:

### **Essential Commands**
| Command | Purpose | Use Case |
|---------|---------|----------|
| `just mission-debug` | Debug mission (step-by-step) | Competition practice |
| `just mission-auto` | Autonomous mission | Competition run |
| `just status` | System health check | Quick diagnostics |
| `just test-all` | Complete system test | Post-installation |
| `just hardware-check` | Hardware detection | Hardware setup |

### **Competition Commands**
| Command | Purpose |
|---------|---------|
| `just competition-ready` | Full readiness check |
| `just emergency-stop` | Emergency mission abort |
| `just recover-system` | Full system recovery |
| `just backup-config` | Backup configurations |

### **Development Commands**
| Command | Purpose |
|---------|---------|
| `just build` | Build ROS 2 workspace |
| `just clean` | Clean build files |
| `just setup` | Run setup process |
| `just validate` | System validation |

### **Docker Commands**
| Command | Purpose |
|---------|---------|
| `just docker-build` | Build Docker image |
| `just docker-run` | Run interactive container |
| `just docker-mission` | Run mission in Docker |
| `just docker-clean` | Clean Docker resources |

---

## 🎯 **Mission System Overview**

### **26-Checkpoint Competition System**
Sistem mission menggunakan FSM (Finite State Machine) dengan 26 checkpoint yang dapat di-debug satu per satu:

#### **Indoor Phase (Checkpoints 1-15)**
1. **INIT** - Initialize systems and arm drone
2. **TAKEOFF** - Takeoff to 0.6m altitude (competition optimized)
3. **SEARCH_ITEM_1_FRONT** - Activate front camera, search item 1
4. **ALIGN_ITEM_1** - Center item 1 in camera view
5. **PICKUP_ITEM_1** - Descend and pickup with front magnet
6. **SEARCH_ITEM_2_BACK** - Activate back camera, search item 2
7. **ALIGN_ITEM_2** - Center item 2 in camera view
8. **PICKUP_ITEM_2** - Descend and pickup with back magnet
9. **NAVIGATE_TURN** - Configurable turn direction
10. **SEARCH_DROPZONE** - Search for dropzone baskets
11. **DROP_ITEM_1_FRONT** - Drop front item first
12. **ASCEND_AFTER_DROP_1** - Ascend after first drop
13. **ALIGN_DROP_2_BACK** - Switch to back camera for second drop
14. **DROP_ITEM_2_BACK** - Drop back item
15. **FIND_EXIT** - Find exit gate with top camera

#### **Outdoor Phase (Checkpoints 16-26)**
16. **ASCEND_TO_OUTDOOR** - Ascend to 3m for outdoor phase
17. **AUTO_WAYPOINT_1** - AUTO mode to waypoint 1
18. **MANUAL_SEARCH_OUTDOOR** - MANUAL mode search for outdoor item
19. **PICKUP_OUTDOOR** - Pickup with front magnet only
20. **ASCEND_TO_WAYPOINT_2** - Ascend to 3m
21. **AUTO_WAYPOINT_2** - AUTO mode to waypoint 2
22. **MANUAL_SEARCH_DROP_OUTDOOR** - MANUAL mode search for dropzone
23. **DROP_OUTDOOR** - Drop outdoor item
24. **ASCEND_TO_WAYPOINT_3** - Ascend to 3m
25. **AUTO_WAYPOINT_3_LANDING** - AUTO mode to waypoint 3 + landing
26. **COMPLETED** - Mission completed!

### **Debug Commands During Mission**
- `next` - Proceed to next checkpoint
- `status` - Show system status
- `help` - Show available commands
- `abort` - Emergency abort and land

---

## 🔧 **Hardware Configuration**

### **Flight Controller Setup**
```bash
# Connect Pixhawk via USB
# Device appears as: /dev/ttyACM0 or /dev/ttyUSB0

# Test connection
ls -la /dev/tty{ACM,USB}*

# Test MAVROS communication
just test-mavros

# Configure connection in config file
nano config/hardware_config.conf
```

### **Camera System Setup**
```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera detection
just test-cameras

# Camera configuration:
# Front camera: /dev/video0 (usually)
# Back camera: /dev/video2 (usually)
# Top camera: /dev/video4 (usually)
```

### **GPIO Configuration (Raspberry Pi)**
```bash
# For electromagnet control
# Default GPIO pins: 18 (front magnet), 19 (back magnet)

# Test GPIO access
python3 -c "import RPi.GPIO; print('GPIO Ready')"

# Configure in hardware config
nano config/hardware_config.conf
```

---

## 🧪 **Testing & Validation**

### **System Health Check**
```bash
# Quick status check
just status

# Comprehensive system test
just test-all

# Hardware-specific tests
just test-mavros      # Flight controller
just test-cameras     # Camera system
just test-gpio        # Electromagnets
just test-sensors     # ToF sensors
```

### **Mission Testing**
```bash
# Simulation mode (no hardware)
python3 simulate_mission.py

# Hardware validation
python3 validate_system.py

# Competition readiness
just competition-ready
```

### **Debug Tools**
```bash
# System diagnosis
just doctor

# Hardware calibration
python3 calibrate_hardware.py

# Log analysis
tail -f logs/mission.log
```

---

## 🚨 **Troubleshooting**

### **Common Issues & Solutions**

| Problem | Symptoms | Solution |
|---------|----------|----------|
| **ROS 2 not found** | `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| **Permission denied** | USB device access error | `sudo usermod -a -G dialout,video $USER` (logout/login) |
| **MAVROS timeout** | No flight controller response | Check USB connection, try different port |
| **Camera not detected** | No video devices | Check USB 3.0 connection, power supply |
| **Import errors** | Python module missing | `pip3 install -r requirements.txt` |
| **Docker permission** | Docker access denied | `sudo usermod -a -G docker $USER` (logout/login) |

### **Emergency Recovery**
```bash
# Complete system recovery
just emergency-recovery

# Reset hardware connections
just reset-hardware

# Rebuild workspace
just clean && just build

# Factory reset configuration
just reset-config
```

### **Log Analysis**
```bash
# Mission logs
tail -f logs/mission.log

# ROS 2 logs
ls ~/.ros/log/

# System logs
journalctl -u drone-service
```

---

## 🏆 **Competition Configuration**

### **Critical Settings to Update**
Before competition, update these settings in `config/hardware_config.conf`:

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

### **Competition Day Checklist**
- [ ] Hardware connections tested
- [ ] GPS coordinates updated
- [ ] Flight altitude configured
- [ ] Camera indices verified
- [ ] Electromagnet pins configured
- [ ] Mission simulation successful
- [ ] Full system validation passed
- [ ] Emergency procedures reviewed

---

## 📚 **Architecture & Technology**

### **Software Stack**
| Layer | Technology | Purpose |
|-------|------------|---------|
| **Mission Control** | Python FSM | 26-checkpoint system |
| **ROS 2 Network** | Humble Hawksbill | Node coordination |
| **MAVLink Bridge** | MAVROS | Flight controller communication |
| **Computer Vision** | OpenCV + YOLOv8 | Object detection |
| **Hardware Control** | RPi.GPIO | Electromagnet control |
| **Flight Controller** | PX4/ArduPilot | Flight control |

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

### **Node Network**
- `checkpoint_mission_node.py` - Main mission FSM
- `camera_control_node.py` - Multi-camera management
- `vision_detector_node.py` - YOLOv8 object detection
- `magnet_control_node.py` - Electromagnet control
- `sensor_monitor.py` - ToF sensor integration

---

## 🤝 **Support & Community**

### **Getting Help**
- 🐛 **GitHub Issues**: [Report bugs and get support](https://github.com/Vanszs/Dirgagah-KAERTEI/issues)
- 📖 **Wiki**: [Comprehensive documentation](https://github.com/Vanszs/Dirgagah-KAERTEI/wiki)
- 💬 **Discussions**: [Community support](https://github.com/Vanszs/Dirgagah-KAERTEI/discussions)

### **Quick Help Commands**
```bash
just help           # Essential commands guide
just help-mission   # Mission-specific help
just help-hardware  # Hardware setup help
just help-debug     # Debugging guide
```

### **Contributing**
1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

---

## 🎖️ **Credits & License**

### **Development Team**
- **KAERTEI 2025 Development Team**
- **Indonesian Drone Enthusiasts Community**

### **Technology Stack**
- **ROS 2 Humble Hawksbill** - Robot Operating System
- **Ubuntu 22.04 LTS** - Operating System
- **OpenCV** - Computer Vision
- **YOLOv8** - Object Detection
- **PyTorch** - Deep Learning Framework
- **MAVROS** - MAVLink ROS Interface
- **ArduPilot** - Flight Controller Firmware

### **License**
This project is licensed under the MIT License - see the LICENSE file for details.

---

<div align="center">

## 🏆 **KAERTEI 2025 FAIO**
### **Ready for Competition!**

*Built with ❤️ by Indonesian Drone Enthusiasts*

[![GitHub](https://img.shields.io/github/stars/Vanszs/Dirgagah-KAERTEI?style=social)](https://github.com/Vanszs/Dirgagah-KAERTEI)

</div>
