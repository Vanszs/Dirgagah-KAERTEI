# 🚀 KAERTEI 2025 FAIO - Installation Guide

<div align="center">

![KAERTEI 2025](logo-krti25.png)

## **Ubuntu 22.04 & Docker Ready**
### *Zero to Competition in 3 Steps*

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04_LTS-orange?logo=ubuntu)](https://ubuntu.com/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker)](https://docker.com/)

</div>

---

## 📋 **System Requirements**

### **Minimum Requirements**
| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **CPU** | Intel/AMD x64, 2+ cores |
| **RAM** | 4GB (8GB recommended) |
| **Storage** | 20GB free space |
| **USB** | 3.0+ ports for hardware |

### **Supported Environments**
- ✅ **Ubuntu 22.04 LTS** (Primary, fully tested)
- ✅ **Docker** (Universal, any OS with Docker)
- ⚠️ Other Ubuntu versions (experimental)

---

## 🚀 **Quick Installation**

### **Option 1: Ubuntu 22.04 (Recommended)**

```bash
# Step 1: Clone repository
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp

# Step 2: One-command setup
./setup_ubuntu22.sh

# Step 3: Validate installation
just test-all

# Step 4: Ready for competition!
just mission
```

### **Option 2: Docker (Universal)**

```bash
# Step 1: Clone repository
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI/ros2_ws/src/drone_mvp

# Step 2: Build Docker environment
docker-compose up --build

# Step 3: Enter container and run
docker exec -it kaertei2025_hexacopter bash
just mission
```

---

## 🔧 **Detailed Installation - Ubuntu 22.04**

### **Prerequisites**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Verify Ubuntu version
lsb_release -a
# Should show: Ubuntu 22.04.x LTS
```

### **Automated Installation**
The `setup_ubuntu22.sh` script installs everything automatically:

- ✅ **ROS 2 Humble Hawksbill**
- ✅ **MAVROS** with GeographicLib datasets
- ✅ **Python packages**: OpenCV, PyMAVLink, YOLOv8
- ✅ **Hardware permissions** and udev rules
- ✅ **Just command runner**
- ✅ **Competition workspace** build

```bash
# Run the installer
chmod +x setup_ubuntu22.sh
./setup_ubuntu22.sh
```

### **Manual Installation (Advanced)**
If you prefer step-by-step control:

```bash
# 1. Install ROS 2 Humble
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-mavros ros-humble-mavros-extras

# 2. Install dependencies
pip3 install -r requirements.txt

# 3. Setup permissions
sudo usermod -a -G dialout,video $USER

# 4. Build workspace
cd ../../
colcon build --packages-select drone_mvp
```

---

## 🐳 **Docker Installation**

### **System Requirements**
```bash
# Install Docker (Ubuntu)
sudo apt install -y docker.io docker-compose
sudo usermod -a -G docker $USER
# Logout and login again
```

### **Build and Run**
```bash
# Build the environment
docker-compose build

# Run interactive container
docker-compose up -d
docker exec -it kaertei2025_hexacopter bash

# Inside container - run mission
source /opt/ros/humble/setup.bash
just mission
```

### **Docker Features**
- 🔒 **Isolated environment**
- 🌍 **Cross-platform compatibility**
- 📦 **All dependencies pre-installed**
- 🔌 **Hardware device passthrough**
- 🚀 **Quick deployment**

---

## 🧪 **Post-Installation Validation**

### **System Health Check**
```bash
# Quick validation
just status

# Comprehensive test
just test-all

# System diagnosis
just doctor
```

### **Hardware Connection Test**
```bash
# List available devices
ls /dev/tty{USB,ACM}*  # Flight controller
ls /dev/video*         # Cameras

# Test MAVROS connection
just test-mavros

# Test camera systems
just test-camera
```

### **Mission Readiness Check**
```bash
# Competition checklist
just checklist

# Run mission simulation
just test-mission

# Debug mission (step-by-step)
just mission
```

---

## ⚡ **Just Commands Reference**

### **Essential Commands**
| Command | Purpose | Use When |
|---------|---------|----------|
| `just mission` | Debug mission mode | Competition practice |
| `just mission-auto` | Autonomous mission | Competition run |
| `just status` | System health | Quick check |
| `just test-all` | Complete validation | After installation |
| `just doctor` | Diagnosis | Troubleshooting |

### **System Management**
| Command | Purpose |
|---------|---------|
| `just setup` | Run setup process |
| `just build` | Build workspace |
| `just clean` | Clean build files |
| `just validate` | Full system test |

### **Emergency Commands**
| Command | Purpose |
|---------|---------|
| `just emergency-stop` | Stop all processes |
| `just recover` | Full system recovery |
| `just reset-usb` | Reset USB devices |

---

## 🛠️ **Hardware Setup**

### **Pixhawk Connection**
```bash
# Connect via USB
# Device appears as: /dev/ttyACM0 or /dev/ttyUSB0

# Check connection
ls -la /dev/tty{ACM,USB}*

# Test communication
just test-mavros
```

### **Camera Configuration**
```bash
# List cameras
v4l2-ctl --list-devices

# Test cameras
just test-camera

# Configuration file
nano config/hardware_config.conf
```

### **GPIO Setup (Raspberry Pi)**
```bash
# For electromagnet control
# GPIO pins 18, 19 (configurable)

# Test GPIO access
python3 -c "import RPi.GPIO; print('GPIO Ready')"
```

---

## 🔍 **Troubleshooting**

### **Common Issues**

| Problem | Solution |
|---------|----------|
| **ROS 2 not found** | `source /opt/ros/humble/setup.bash` |
| **Permission denied** | `sudo usermod -a -G dialout $USER` (logout/login) |
| **MAVROS timeout** | Check USB connection, try different port |
| **Camera not detected** | Check USB 3.0 connection, power |
| **Import errors** | `pip3 install -r requirements.txt` |

### **System Diagnosis**
```bash
# Comprehensive diagnosis
just doctor

# Check specific components
just test-hardware    # Hardware connections
just test-mavros     # MAVROS communication
just test-camera     # Camera systems
just validate        # Full system test
```

### **Log Analysis**
```bash
# ROS 2 logs
ls ~/.ros/log/

# Mission logs
ls logs/

# System logs
journalctl -f
```

---

## 📞 **Support & Help**

### **Getting Help**
```bash
# Quick help for competition
just help-quick

# Complete help system
just help

# Show all available commands
just --list
```

### **Documentation**
- 📖 **README.md** - Project overview
- 🛠️ **HARDWARE_SETUP.md** - Hardware guide
- 🏁 **COMPETITION_GUIDE.md** - Competition specifics
- 🚨 **TROUBLESHOOTING.md** - Problem solving

### **Community Support**
- 🐛 **GitHub Issues**: Report bugs and get help
- 📧 **Repository Wiki**: Detailed documentation
- 💬 **Discussions**: Community support

---

## ✅ **Installation Checklist**

- [ ] Ubuntu 22.04 LTS verified
- [ ] Repository cloned
- [ ] `setup_ubuntu22.sh` completed successfully
- [ ] Hardware permissions configured
- [ ] ROS 2 Humble installed and sourced
- [ ] MAVROS connection tested
- [ ] Python dependencies installed
- [ ] Just commands working
- [ ] Hardware devices detected
- [ ] Mission simulation successful
- [ ] Competition configuration updated

---

<div align="center">

## 🏆 **Ready for KAERTEI 2025 FAIO Competition!**

*Built with ❤️ for Indonesian Drone Enthusiasts*

</div>
