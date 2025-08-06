# 🔧 HARDWARE SETUP - KAERTEI 2025 FAIO

## **Panduan Setup Hardware Lengkap**

*Dokumen ini menjelaskan cara setup semua komponen hardware*

---

## 📋 **Daftar Hardware yang Dibutuhkan**

### **🚁 Flight System:**
- **Hexacopter Frame** - 6 motor configuration
- **Pixhawk4 Flight Controller** - untuk kontrol motor + GPS
- **6x ESC + Motor** - Electronic Speed Controller + Motor brushless
- **Propeller** - 6 buah (3 CW + 3 CCW)
- **LiPo Battery** - 4S atau 6S sesuai motor

### **💻 Computing System:**
- **Ubuntu PC/Laptop** - Ubuntu 22.04 LTS
- **Raspberry Pi 5** - untuk AI processing + sensor control
- **MicroSD Card** - 64GB+ untuk Pi 5
- **USB Hub** - untuk multiple devices

### **📡 Sensor System:**
- **3x TF Mini Plus LiDAR** - obstacle detection (depan, kiri, kanan)
- **3x USB Camera** - vision system (depan, belakang, atas)
- **GPS Module** - terhubung ke Pixhawk4
- **IMU/Compass** - integrated di Pixhawk4

### **🧲 Payload System:**
- **2x Electromagnet** - untuk pickup/drop object
- **2x Relay Module** - GPIO control dari Pi 5
- **Power Supply** - untuk electromagnet (12V/5V)

### **🔌 Connection & Power:**
- **USB A-to-B Cable** - PC ke Pixhawk4
- **USB Cables** - untuk cameras
- **GPIO Jumper Wires** - Pi 5 ke relay
- **Power Distribution** - untuk semua komponen

---

## 🔗 **Diagram Koneksi**

### **System Architecture:**
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Ubuntu PC     │    │   Pixhawk4      │    │   Hexacopter    │
│  (Mission PC)   │◄──►│ (Flight Ctrl)   │◄──►│  (6 Motors)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │ USB                    │ Telemetry         │ PWM Signals
         │                        ▼                    ▼
         │              ┌─────────────────┐    ┌─────────────────┐
         │              │   GPS Module    │    │   ESC + Motors  │
         │              └─────────────────┘    └─────────────────┘
         │
         ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Raspberry Pi 5  │◄──►│   3x Cameras    │    │   3x LiDAR      │
│  (AI Computer)  │    │ (USB Connected) │    │ (Serial/I2C)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │ GPIO
         ▼
┌─────────────────┐    ┌─────────────────┐
│   2x Relay      │◄──►│ 2x Electromagnet│
│ (GPIO Control)  │    │  (Payload Sys)  │
└─────────────────┘    └─────────────────┘
```

---

## 🛠️ **Setup Step-by-Step**

### **1. Setup Ubuntu PC**

#### **Install OS:**
```bash
# Download Ubuntu 22.04 LTS Desktop
# Install di PC/Laptop yang akan digunakan sebagai ground station

# Update system
sudo apt update && sudo apt upgrade -y

# Install basic tools
sudo apt install git curl wget build-essential -y
```

#### **Install KAERTEI System:**
```bash
# Clone project
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI

# Auto setup semua
just setup
```

### **2. Setup Raspberry Pi 5**

#### **Prepare Pi 5:**
```bash
# Flash Raspberry Pi OS 64-bit ke MicroSD
# Enable SSH, I2C, SPI, Camera di raspi-config

# Install dependencies
sudo apt update
sudo apt install python3-pip git -y
pip install RPi.GPIO pyserial
```

#### **Install ROS 2 di Pi 5:**
```bash
# Install ROS 2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base -y
```

### **3. Setup Pixhawk4**

#### **Firmware:**
```bash
# Download QGroundControl
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.8/QGroundControl.AppImage
chmod +x QGroundControl.AppImage

# Flash PX4 firmware terbaru
# Konfigurasi frame sebagai Hexacopter X
# Kalibrasi sensor (accelerometer, gyro, compass, radio)
```

#### **Parameter Configuration:**
- **Frame Type:** Generic Hexacopter X
- **Flight Mode:** Auto, Position, Return
- **Failsafe:** Return to Launch (RTL)
- **GPS:** Required for outdoor navigation

### **4. Setup Kamera System**

#### **3x USB Camera:**
```bash
# Test camera detection
ls /dev/video*

# Should show /dev/video0, /dev/video1, /dev/video2

# Test each camera
just camera-test
```

#### **Camera Mounting:**
- **Front Camera:** untuk navigation + object detection
- **Back Camera:** untuk monitoring + reverse navigation
- **Top Camera:** untuk overhead view + landing

### **5. Setup LiDAR System**

#### **3x TF Mini Plus:**
```bash
# Connect via Serial/UART
# Front LiDAR: /dev/ttyUSB0 (atau Serial0)
# Left LiDAR:  /dev/ttyUSB1 (atau Serial1)  
# Right LiDAR: /dev/ttyUSB2 (atau Serial2)

# Test LiDAR
just lidar-test
```

#### **LiDAR Mounting:**
- **Front:** obstacle detection saat forward flight
- **Left:** obstacle detection saat sideways movement
- **Right:** obstacle detection saat sideways movement

### **6. Setup Electromagnet System**

#### **GPIO Wiring (Pi 5):**
```
Pi 5 GPIO 18 ──► Relay 1 ──► Electromagnet 1 (Front)
Pi 5 GPIO 19 ──► Relay 2 ──► Electromagnet 2 (Back)
Pi 5 Ground  ──► Relay Ground
Pi 5 5V      ──► Relay VCC
```

#### **Power Supply:**
- **12V DC** untuk electromagnet (high power)
- **5V DC** untuk relay logic
- **Separate power** dari main battery untuk safety

### **7. Setup Koneksi Network**

#### **PC ↔ Pixhawk4:**
```bash
# USB Serial connection
# Biasanya /dev/ttyACM0 atau /dev/ttyUSB0

# Test connection
just mavlink-test
```

#### **PC ↔ Raspberry Pi 5:**
```bash
# Via WiFi atau Ethernet
# Set same network untuk ROS 2 communication

# Test ROS network
export ROS_DOMAIN_ID=42
ros2 topic list
```

---

## 🧪 **Testing Hardware**

### **Individual Component Test:**
```bash
# Test flight controller
just pixhawk-test

# Test all cameras
just camera-test

# Test all LiDAR
just lidar-test

# Test electromagnets
just magnet-test

# Test GPS
just gps-test
```

### **Integrated System Test:**
```bash
# Test all hardware together
just hardware-test

# System health check
just status

# Full system validation
just test
```

### **Pre-Flight Test:**
```bash
# Competition readiness check
just competition-ready

# Mission system test
just test-mission

# Emergency procedures test
just test-emergency
```

---

## ⚖️ **Weight & Balance**

### **Weight Distribution:**
- **Center of Gravity** harus di tengah frame
- **Battery placement** untuk balance
- **Camera gimbal** jika diperlukan untuk stabilization
- **Total weight** sesuai kemampuan motor

### **Power Consumption:**
- **Flight time** minimum 10 menit untuk mission
- **Power budget** untuk semua elektronik
- **Backup power** untuk critical system

---

## 🔧 **Maintenance & Calibration**

### **Regular Calibration:**
```bash
# Compass calibration (outdoor)
just calibrate-compass

# Accelerometer calibration (level surface)
just calibrate-accel

# Camera calibration
just calibrate-camera

# LiDAR calibration
just calibrate-lidar
```

### **Pre-Competition Check:**
```bash
# Full hardware inspection
just inspect-hardware

# Calibration check
just check-calibration

# Performance test
just performance-test
```

---

## 🚨 **Safety Considerations**

### **Safety Features:**
- **Emergency stop** button/command
- **Failsafe mode** jika koneksi terputus
- **Low battery** return to launch
- **Obstacle avoidance** dengan LiDAR
- **Manual override** capability

### **Safety Procedures:**
```bash
# Enable all safety features
just enable-safety

# Test emergency procedures
just test-emergency

# Set failsafe parameters
just configure-failsafe
```

---

## 📊 **Hardware Status Dashboard**

### **Monitoring Commands:**
```bash
# Real-time hardware status
just monitor

# Hardware health dashboard
just hardware-dashboard

# Performance metrics
just performance-metrics

# System resources
just system-resources
```

---

## 🏆 **Competition Configuration**

### **Final Setup:**
```bash
# Competition mode configuration
just competition-config

# Lock settings
just lock-config

# Create backup
just backup-config

# Competition ready check
just final-check
```

**✅ Hardware setup complete! Siap untuk kompetisi KAERTEI 2025!**
