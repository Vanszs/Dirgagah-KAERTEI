# 🚨 TROUBLESHOOTING - KAERTEI 2025 FAIO

## **Panduan Mengatasi Masalah Umum**

*Dokumen ini membantu mengatasi masalah yang sering terjadi*

---

## 🔧 **Masalah Installation**

### **❌ Error: "command not found: just"**
**Penyebab:** Just command belum ter-install

**Solusi:**
```bash
# Install just
curl -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin

# Tambah ke PATH
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Test
just --version
```

### **❌ Error: "No ROS 2 found"**
**Penyebab:** ROS 2 Humble belum ter-install atau environment belum di-source

**Solusi:**
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop-full

# Source environment
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### **❌ Error: "Permission denied"**
**Penyebab:** File tidak executable atau tidak punya akses sudo

**Solusi:**
```bash
# Buat file executable
chmod +x install_kaertei.sh
chmod +x kaertei

# Atau jalankan dengan sudo jika diperlukan
sudo ./install_kaertei.sh
```

---

## 🔌 **Masalah Hardware**

### **❌ "No flight controllers detected"**
**Penyebab:** Pixhawk tidak terdeteksi

**Solusi:**
```bash
# Cek USB devices
lsusb | grep -i "px4\|pixhawk\|3dr"

# Cek serial ports
ls -la /dev/ttyACM* /dev/ttyUSB*

# Test koneksi
sudo chmod 666 /dev/ttyACM0  # atau ttyUSB0

# Test dengan Justfile
just hardware
```

### **❌ "No cameras detected"**  
**Penyebab:** USB camera tidak terhubung

**Solusi:**
```bash
# Cek video devices
ls -la /dev/video*

# Test camera
just camera-test

# Install v4l-utils jika diperlukan
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

### **❌ "GPIO not available"**
**Penyebab:** Tidak jalan di Raspberry Pi atau library belum install

**Solusi:**
```bash
# Di Raspberry Pi, install GPIO library
pip install RPi.GPIO

# Test GPIO
just magnet-test

# Untuk development di PC (simulation mode)
# System akan otomatis detect dan jalan dalam simulation mode
```

---

## 🚁 **Masalah Mission**

### **❌ "Mission failed at checkpoint X"**
**Penyebab:** Berbagai, tergantung checkpoint

**Diagnosa:**
```bash
# Jalankan debug mode
just debug

# Lihat logs
just logs

# Cek system status
just status
```

### **❌ "Drone tidak takeoff"**
**Penyebab:** ARM gagal atau GPS belum fix

**Solusi:**
```bash
# Cek GPS status
just gps-test

# Force ARM (hati-hati!)
just emergency-arm

# Reset system
just reset
```

### **❌ "Vision detection tidak jalan"**
**Penyebab:** Camera atau AI model bermasalah

**Solusi:**
```bash
# Test kamera
just camera-test

# Test AI detection
just vision-test

# Download ulang model jika perlu
just setup-vision
```

---

## 🐳 **Masalah Docker**

### **❌ "Docker permission denied"**
**Penyebab:** User belum dalam docker group

**Solusi:**
```bash
# Tambah user ke docker group
sudo usermod -aG docker $USER
newgrp docker

# Logout dan login ulang
logout
```

### **❌ "Container tidak start"**
**Penyebab:** Port conflict atau resource tidak cukup

**Solusi:**
```bash
# Stop semua container
docker stop $(docker ps -aq)

# Clean up
docker system prune -a

# Restart Docker service
sudo systemctl restart docker
```

---

## 📊 **Diagnostic Tools**

### **System Health Check:**
```bash
# Comprehensive check
just doctor

# Quick status
just status

# Hardware check
just hardware

# Software check  
just software-check
```

### **Log Collection:**
```bash
# System logs
journalctl -f

# ROS logs
ls ~/.ros/log/

# Mission logs
just logs

# Hardware logs
dmesg | tail
```

### **Network Issues:**
```bash
# Test ROS network
ros2 topic list
ros2 node list

# Test MAVLink
just mavlink-test

# Reset network
just network-reset
```

---

## 🆘 **Emergency Procedures**

### **Emergency Stop:**
```bash
# Stop everything immediately
just stop
# atau
just emergency
```

### **System Reset:**
```bash
# Reset semua komponen
just reset-all

# Reset mission only
just reset-mission

# Reset hardware only  
just reset-hardware
```

### **Recovery Mode:**
```bash
# Masuk recovery mode
just recovery

# Minimal safe mode
just safe-mode

# Factory reset (last resort)
just factory-reset
```

---

## 📞 **Getting Help**

### **Collect Information:**
```bash
# System info untuk bug report
just collect-logs

# Hardware info
just hardware-info

# Software versions
just version-info
```

### **Contact Support:**
- 🐛 **Bug Report:** [GitHub Issues](https://github.com/Vanszs/Dirgagah-KAERTEI/issues)
- 💬 **Discussion:** [GitHub Discussions](https://github.com/Vanszs/Dirgagah-KAERTEI/discussions)
- 📖 **Documentation:** [Project Wiki](https://github.com/Vanszs/Dirgagah-KAERTEI/wiki)

### **Include in Bug Report:**
1. Error message lengkap
2. Output dari `just status`  
3. Output dari `just doctor`
4. System info (`uname -a`)
5. Langkah untuk reproduce error

---

## 🏆 **Tips Kompetisi**

### **Pre-Competition:**
```bash
# Full system check
just competition-ready

# Backup configuration
just backup-config

# Test full mission
just test-mission
```

### **During Competition:**
```bash
# Quick health check
just quick-check

# Emergency procedures
just emergency-stop    # Stop drone
just emergency-arm     # Force arm if stuck
just emergency-land    # Force landing
```

---

**💡 Tips:** Selalu jalankan `just status` dan `just test` sebelum kompetisi!
