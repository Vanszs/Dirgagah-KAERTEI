# 🚁 KAERTEI 2025 FAIO - Multi-Distribution Installation Guide

## 🐧 **Supported Operating Systems**

| Distribution | Status | Package Manager | Notes |
|--------------|--------|-----------------|-------|
| **Ubuntu 22.04** | ✅ Fully Supported | `apt` | Recommended for beginners |
| **Ubuntu 20.04** | ✅ Supported | `apt` | Older but stable |
| **Debian 11+** | ✅ Supported | `apt` | Stable, minimal |
| **Arch Linux** | ✅ Fully Supported | `pacman` | Rolling release, latest packages |
| **Manjaro** | ✅ Supported | `pacman` | User-friendly Arch derivative |
| **EndeavourOS** | ✅ Supported | `pacman` | Lightweight Arch derivative |

---

## 🚀 **Quick Installation (Any Distro)**

### **Option 1: Master Installer (Recommended)**
```bash
cd /home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp/
chmod +x install_kaertei.sh
./install_kaertei.sh
# Select option 5 (Install All)
```

### **Option 2: Manual Step-by-Step**
```bash
# 1. Install ROS 2 Humble
./install_ros2.sh

# 2. Install MAVROS  
./install_mavros.sh

# 3. Build project
colcon build --packages-select drone_mvp
```

---

## 📋 **Distribution-Specific Instructions**

### **🟠 Ubuntu/Debian Users**

#### **Prerequisites**
```bash
sudo apt update
sudo apt install git curl wget build-essential
```

#### **ROS 2 Installation**
```bash
# Automated installation
./install_ros2.sh

# Manual installation (if needed)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools
```

#### **MAVROS Installation**
```bash
# Try package installation first
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# If packages fail, script will build from source automatically
./install_mavros.sh
```

---

### **🔵 Arch Linux Users**

#### **Prerequisites**
```bash
sudo pacman -S git base-devel yay
```

#### **ROS 2 Installation**
```bash
# Automated installation (installs yay if needed)
./install_ros2.sh

# Manual installation
yay -S ros-humble-desktop
yay -S python-colcon-common-extensions python-rosdep python-vcstool
```

#### **MAVROS Installation**
```bash
# Always builds from source (no packages available)
./install_mavros.sh

# Manual dependencies
sudo pacman -S eigen geographiclib
```

---

## 🎮 **Usage Modes**

### **Execution Modes**
- **`debug`** - Manual checkpoint progression (default)
- **`auto`** - Autonomous execution

### **Communication Modes**  
- **`mavros`** - Use MAVROS bridge (recommended)
- **`mavlink`** - Direct MAVLink communication

### **Launch Examples**
```bash
# Debug mode with MAVROS (recommended)
./run_checkpoint_mission.sh debug mavros

# Autonomous mode with MAVROS
./run_checkpoint_mission.sh auto mavros

# Debug mode with direct MAVLink (fallback)
./run_checkpoint_mission.sh debug mavlink

# Autonomous mode with direct MAVLink  
./run_checkpoint_mission.sh auto mavlink
```

---

## 🔧 **Configuration Management**

### **Hardware Configuration**
```bash
# Edit hardware settings
nano config/hardware_config.conf

# Quick debug mode toggle
./set_debug_mode.sh on    # Enable debug mode
./set_debug_mode.sh off   # Enable autonomous mode
./set_debug_mode.sh status # Check current mode
```

### **Key Configuration Sections**
```ini
[MISSION]
debug_mode = true          # true = manual, false = autonomous
indoor_altitude = 0.6      # 60cm indoor altitude
outdoor_altitude = 3.0     # 3m outdoor altitude
turn_direction = right     # left/right turn preference

[flight_controller]
connection_port = "/dev/ttyUSB0"  # Adjust for your system
baud_rate = 57600

[GPS_WAYPOINTS]
waypoint_1_lat = -6.365000 # UPDATE for your venue!
waypoint_1_lon = 106.825000
# ... more waypoints
```

---

## 🛠️ **Troubleshooting by Distribution**

### **Ubuntu/Debian Issues**

#### **ROS 2 Key Error**
```bash
# Fix GPG key issues
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

#### **MAVROS GeographicLib Error**
```bash
# Manual GeographicLib installation
sudo apt install geographiclib-tools libgeographic-dev
sudo /usr/share/GeographicLib/get-geoids all
```

### **Arch Linux Issues**

#### **yay Not Found**
```bash
# Install yay AUR helper
git clone https://aur.archlinux.org/yay.git /tmp/yay
cd /tmp/yay && makepkg -si
```

#### **rosdep Errors (Normal)**  
```bash
# rosdep may fail on Arch - this is expected
# Dependencies are handled by pacman instead
rosdep install --from-paths src --ignore-src -r -y || true
```

#### **Python Path Issues**
```bash
# Fix Python path for pip packages
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

---

## 🔍 **Hardware Detection**

### **Flight Controller Detection**
```bash
# List serial devices
ls /dev/tty{USB,ACM}* 2>/dev/null

# Ubuntu/Debian permissions
sudo usermod -a -G dialout $USER

# Arch Linux permissions  
sudo usermod -a -G uucp $USER

# Logout and login after group changes
```

### **Camera Detection**
```bash
# List camera devices
ls /dev/video*

# Test camera
v4l2-ctl --list-devices

# Ubuntu camera permissions
sudo usermod -a -G video $USER

# Arch camera permissions (usually not needed)
# Camera permissions are typically handled automatically
```

---

## 🧪 **Testing Installation**

### **Quick Test**
```bash
# Test all components
./install_kaertei.sh
# Select option 6 (Test Installation)
```

### **Manual Testing**
```bash
# Test ROS 2
source /opt/ros/humble/setup.bash
ros2 topic list

# Test MAVROS (if installed)
ros2 pkg list | grep mavros

# Test Python packages
python3 -c "import cv2, ultralytics, pymavlink; print('All packages OK')"

# Test workspace build
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep drone_mvp
```

---

## 📊 **Performance Comparison**

| Feature | Ubuntu | Arch Linux |
|---------|--------|------------|
| **Installation Speed** | 🟡 Moderate | 🟢 Fast |
| **Package Availability** | 🟢 Excellent | 🟡 Good |
| **System Resources** | 🟡 Moderate | 🟢 Minimal |
| **Stability** | 🟢 Stable | 🟡 Rolling |
| **Beginner Friendly** | 🟢 Very Easy | 🟡 Moderate |
| **Latest Packages** | 🟡 LTS Versions | 🟢 Latest |

---

## 🎯 **Recommendations**

### **For Competition (Stability Priority)**
- **Ubuntu 22.04 LTS** - Most stable, extensive testing
- Use **MAVROS mode** - Better ROS 2 integration
- Use **debug mode** - Manual control for critical moments

### **For Development (Latest Features)**
- **Arch Linux** - Latest packages, faster compilation
- Use **mavlink mode** - Direct communication, less overhead
- Use **auto mode** - Faster testing cycles

### **For Learning (Ease of Use)**
- **Ubuntu 22.04** - Extensive documentation, community support
- Use **MAVROS mode** - More ROS 2 learning opportunities
- Use **debug mode** - Step-by-step understanding

---

## 🔄 **Migration Between Distributions**

### **Ubuntu → Arch Linux**
```bash
# Export configuration
cp -r config/ ~/drone_config_backup/

# Fresh install on Arch
./install_kaertei.sh

# Restore configuration
cp -r ~/drone_config_backup/* config/
```

### **Arch Linux → Ubuntu**
```bash
# Same process - configuration files are compatible
# Only difference is package installation method
```

---

## 🎉 **Ready for KAERTEI 2025!**

After successful installation on any distribution:

```bash
# Final test launch
./run_checkpoint_mission.sh debug mavros

# Competition launch (autonomous)
./run_checkpoint_mission.sh auto mavros
```

**🏆 All systems ready for KAERTEI 2025 FAIO competition!** 🚁
