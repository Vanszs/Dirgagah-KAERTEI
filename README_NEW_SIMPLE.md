# 🚁 KAERTEI 2025 FAIO - Sistem Drone Otonom

<div align="center">

<img src="logo-krti25.png" alt="KAERTEI 2025" width="200"/>

### **Hexacopter Otonom untuk Kompetisi FAIO**
**Ubuntu 22.04 + ROS 2 Humble - Siap Kompetisi**

[![Competition Ready](https://img.shields.io/badge/Status-Competition%20Ready-brightgreen)](https://github.com/Vanszs/Dirgagah-KAERTEI)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange)](https://ubuntu.com/download/desktop)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)

</div>

---

## 📖 **Apa itu KAERTEI 2025?**

**KAERTEI 2025** adalah sistem drone hexacopter yang dibuat khusus untuk kompetisi **FAIO (Festival Aeromodelling Indonesia Open)**. Drone ini bisa terbang sendiri (otonom) tanpa dikontrol manual untuk menyelesaikan misi 26 checkpoint.

### 🎯 **Misi Drone:**
1. **Takeoff otomatis** - Terbang sendiri ke ketinggian 1 meter
2. **Navigasi 26 checkpoint** - Mengunjungi 26 titik secara berurutan  
3. **Deteksi objek** - Mengenali objek dengan kamera dan AI
4. **Pickup & Drop** - Mengambil dan meletakkan objek dengan magnet
5. **Landing otomatis** - Mendarat sendiri di titik finish

---

## ⚡ **Install & Jalankan (3 Langkah Simple)**

> **Catatan:** Untuk orang awam, ikuti langkah ini dengan copy-paste perintah

### **Langkah 1: Download Project**
```bash
# Download project dari GitHub
git clone https://github.com/Vanszs/Dirgagah-KAERTEI.git
cd Dirgagah-KAERTEI
```

### **Langkah 2: Install Semua (Otomatis)**
```bash
# Install semua dependency otomatis (tunggu 10-15 menit)
just setup
```

### **Langkah 3: Test System**
```bash
# Cek apakah semua berhasil ter-install
just test
```

**🎉 Selesai! System siap digunakan.**

---

## 🎮 **Cara Menjalankan Drone**

### **📋 Perintah Utama**

| Perintah | Fungsi | Kapan Digunakan |
|----------|--------|-----------------|
| `just setup` | Install semua | Pertama kali saja |
| `just test` | Cek system | Sebelum kompetisi |
| `just debug` | Mission step-by-step | Latihan/Testing |
| `just run` | Mission otomatis penuh | Saat kompetisi |
| `just stop` | Stop darurat | Emergency |
| `just status` | Cek kondisi system | Kapan saja |

### **🚀 Menjalankan Mission**

#### **Mode Debug (Untuk Latihan):**
```bash
# Jalankan mission step-by-step (bisa pause/continue)
just debug
```

#### **Mode Otomatis (Untuk Kompetisi):**
```bash
# Jalankan full otomatis 26 checkpoint
just run
```

#### **Mode Emergency:**
```bash
# Stop semua secara darurat
just stop
```

---

## 🔧 **Hardware yang Dibutuhkan**

### **Komponen Utama:**
- **💻 Komputer:** Ubuntu 22.04 (Laptop/PC)
- **🚁 Flight Controller:** Pixhawk4 (untuk motor + GPS)
- **🖥️ Single Board Computer:** Raspberry Pi 5 (untuk AI & sensor)
- **📷 Kamera:** 3x USB Camera (depan, belakang, atas)
- **📡 LiDAR:** 3x TF Mini Plus (depan, kiri, kanan)
- **🧲 Magnet:** 2x Electromagnet + GPIO Relay
- **🔌 Koneksi:** USB cable ke Pixhawk

### **Skema Sistem:**
```
Komputer (Ubuntu) ←→ Pixhawk4 ←→ 6 Motor Hexacopter
      ↓
Raspberry Pi 5 ←→ Kamera + LiDAR + Magnet
```

---

## 📚 **Dokumentasi Lengkap**

Untuk informasi lebih detail, baca dokumen-dokumen berikut:

| Dokumen | Isi | Untuk Siapa |
|---------|-----|-------------|
| **[SETUP.md](SETUP.md)** | Panduan install detail | Pemula yang butuh penjelasan step-by-step |
| **[INSTALLATION.md](INSTALLATION.md)** | Install manual & troubleshooting | Advanced user |
| **[HARDWARE_UPDATE_SUMMARY.md](HARDWARE_UPDATE_SUMMARY.md)** | Update hardware terbaru | Developer |
| **[OPTIMIZATION_SUMMARY.md](OPTIMIZATION_SUMMARY.md)** | Optimisasi system | Technical team |

---

## 🚨 **Troubleshooting (Jika Ada Masalah)**

### **Masalah Umum:**

#### **❌ "command not found: just"**
```bash
# Install just command
curl -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin
export PATH="$HOME/.local/bin:$PATH"
```

#### **❌ "No ROS 2 found"**
```bash
# Install ulang ROS 2
sudo apt update
sudo apt install ros-humble-desktop-full
```

#### **❌ "Hardware not detected"**
```bash
# Cek koneksi hardware
just hardware
```

### **Cek System Health:**
```bash
# Cek apakah semua OK
just status

# Cek detail system
just test

# Lihat semua perintah available
just --list
```

---

## 🎯 **Detail Mission 26 Checkpoint**

Mission drone terdiri dari 26 checkpoint yang harus diselesaikan secara berurutan:

### **Fase 1: Takeoff (Checkpoint 1-3)**
1. **Arm & Takeoff** - Drone siap dan terbang ke 1m
2. **Stabilize** - Hover stabil di ketinggian
3. **GPS Lock** - Tunggu GPS fix

### **Fase 2: Navigation (Checkpoint 4-20)**
- **Checkpoint 4-8:** Navigasi indoor menggunakan kamera
- **Checkpoint 9-15:** Transisi ke outdoor + GPS navigation
- **Checkpoint 16-20:** Object detection & pickup preparation

### **Fase 3: Mission Task (Checkpoint 21-24)**
- **Checkpoint 21:** Object detection dengan AI
- **Checkpoint 22:** Pickup object dengan magnet
- **Checkpoint 23:** Transport ke dropzone
- **Checkpoint 24:** Drop object

### **Fase 4: Return & Landing (Checkpoint 25-26)**
- **Checkpoint 25:** Return to home
- **Checkpoint 26:** Auto landing

---

## 🛠️ **Advanced Commands**

### **Developer Commands:**
```bash
# Build ROS 2 workspace
just build

# Clean & rebuild
just rebuild  

# Run specific node
just camera      # Test camera
just lidar       # Test LiDAR  
just magnet      # Test magnet

# Hardware testing
just hardware    # Test all hardware
```

### **Competition Commands:**
```bash
# Competition readiness check
just ready

# Emergency stop during mission
just emergency

# System diagnosis
just doctor

# View logs
just logs
```

---

## 🏆 **Competition Ready Status**

✅ **Hardware Architecture:** Pi 5 + Pixhawk4  
✅ **Altitude Setting:** 1.0m takeoff (sesuai regulasi)  
✅ **Sensor Integration:** 3x LiDAR + 3x Camera  
✅ **Payload System:** 2x GPIO Electromagnet  
✅ **Build System:** ROS 2 Humble + Python  
✅ **Universal Commands:** `kaertei` dan `just` tersedia  
✅ **Testing:** All nodes functional  

---

## 🤝 **Bantuan & Support**

### **Kalau Ada Masalah:**
- 🐛 **GitHub Issues:** [Laporkan bug di sini](https://github.com/Vanszs/Dirgagah-KAERTEI/issues)
- 💬 **GitHub Discussions:** [Forum diskusi](https://github.com/Vanszs/Dirgagah-KAERTEI/discussions)
- 📖 **Wiki:** [Dokumentasi lengkap](https://github.com/Vanszs/Dirgagah-KAERTEI/wiki)

### **Quick Help:**
```bash
just help       # Panduan perintah
just info       # Info system
```

---

## 📊 **System Architecture (Technical)**

<details>
<summary>Click untuk lihat detail teknis</summary>

### **Software Stack:**
```
Mission Control (Python) ← 26-checkpoint FSM
     ↓
ROS 2 Nodes Network ← Node coordination  
     ↓
MAVLink Bridge ← MAVROS communication
     ↓
PX4 Firmware ← Flight control logic
     ↓
Pixhawk PX4 ← Hardware control
```

### **Hardware Architecture:**
```
Ubuntu PC → Pixhawk4 → 6 Motor Hexacopter
    ↓           ↓              ↓
Raspberry → 3x Camera → 3x LiDAR → 2x Magnet
    Pi 5       System      Sensors    Payload
```

### **Node Network:**
- `mission_node` - Mission control FSM
- `checkpoint_mission_node` - Checkpoint logic  
- `camera_control_node` - Camera management
- `lidar_control_node` - LiDAR obstacle detection
- `gpio_control_node` - Magnet control
- `vision_detector_node` - AI object detection

</details>

---

## 🏆 **KAERTEI 2025 FAIO Competition Ready!**

<div align="center">

**🚁 Autonomous Hexacopter System**  
**✅ 26-Checkpoint Mission Capable**  
**🎯 Competition Tested & Validated**

**Good Luck untuk Kompetisi KAERTEI 2025! 🏆**

</div>
