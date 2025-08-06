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

# Monitor checkpoint progress real-time
just monitor-checkpoint

# Debug specific checkpoint
just debug-checkpoint 15
```

#### **Mode Otomatis (Untuk Kompetisi):**
```bash
# Jalankan full otomatis 26 checkpoint
just run

# Monitor mission dashboard
just dashboard

# Live telemetry
just telemetry
```

#### **Mode Emergency:**
```bash
# Stop semua secara darurat
just stop

# Emergency landing
just emergency-land

# Return to launch immediately
just rtl
```

#### **🖥️ Real-Time Monitoring:**
```bash
# Mission progress: [████████░░] 18/26 checkpoints
# Current Phase: Object Detection (CP-17)
# Status: AI scanning for target objects
# Sensors: Camera✅ LiDAR✅ GPS✅ Electromagnet⚠️
# Battery: 68% (Est. 4.2min remaining)
# Flight Time: 00:03:42
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
| **[HARDWARE_SETUP.md](HARDWARE_SETUP.md)** | Setup hardware lengkap | Yang mau setup fisik drone |
| **[COMPETITION_GUIDE.md](COMPETITION_GUIDE.md)** | Panduan kompetisi FAIO | Peserta kompetisi |
| **[INSTALLATION.md](INSTALLATION.md)** | Install manual & troubleshooting | Advanced user |
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | Solusi masalah umum | Semua user yang ada masalah |
| **[HARDWARE_UPDATE_SUMMARY.md](HARDWARE_UPDATE_SUMMARY.md)** | Update hardware terbaru | Developer |
| **[OPTIMIZATION_SUMMARY.md](OPTIMIZATION_SUMMARY.md)** | Optimisasi system | Technical team |

---

## 🚨 **Troubleshooting (Jika Ada Masalah)**

> **📖 Untuk panduan troubleshooting lengkap, baca: [TROUBLESHOOTING.md](TROUBLESHOOTING.md)**

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

# Emergency help
just doctor
```

**💡 Tip:** Lihat [TROUBLESHOOTING.md](TROUBLESHOOTING.md) untuk solusi lengkap masalah lainnya.

---

## 🎯 **Detail Mission 26 Checkpoint**

Mission drone terdiri dari 26 checkpoint yang harus diselesaikan secara berurutan. Setiap checkpoint punya tugas spesifik dan sistem akan validate completion sebelum lanjut ke checkpoint berikutnya.

### **📋 Complete Checkpoint Breakdown:**

#### **🚀 Fase 1: Launch Sequence (Checkpoint 1-3)**
| Checkpoint | Task | Kriteria Sukses | Sensor Used | Waktu |
|------------|------|----------------|-------------|-------|
| **CP-01** | **ARM System** | Flight controller armed, motors spinning | Pixhawk4 | 3s |
| **CP-02** | **Takeoff to 1m** | Altitude reach 1.0m ±0.1m, stable hover | GPS + Barometer | 5s |
| **CP-03** | **GPS Lock & Stabilize** | GPS accuracy <2m, position hold stable | GPS + IMU | 5s |

#### **🏠 Fase 2: Indoor Navigation (Checkpoint 4-8)**
| Checkpoint | Task | Kriteria Sukses | Sensor Used | Waktu |
|------------|------|----------------|-------------|-------|
| **CP-04** | **Indoor Start Position** | Reach start waypoint, camera active | Camera + LiDAR | 8s |
| **CP-05** | **Visual Odometry Init** | Camera calibrated, visual tracking active | Front Camera | 5s |
| **CP-06** | **Obstacle Avoidance Test** | Navigate around obstacle using LiDAR | 3x LiDAR | 10s |
| **CP-07** | **Indoor Waypoint 1** | Reach WP1 using vision navigation | Camera + LiDAR | 15s |
| **CP-08** | **Indoor Waypoint 2** | Reach WP2, prepare for outdoor transition | Camera + GPS | 15s |

#### **🌍 Fase 3: Outdoor Transition (Checkpoint 9-15)**
| Checkpoint | Task | Kriteria Sukses | Sensor Used | Waktu |
|------------|------|----------------|-------------|-------|
| **CP-09** | **Exit Indoor Area** | Exit through designated gate/opening | Camera + LiDAR | 10s |
| **CP-10** | **GPS Navigation Switch** | Switch to GPS primary navigation | GPS + Compass | 5s |
| **CP-11** | **Outdoor Waypoint 1** | Reach outdoor WP1 using GPS | GPS | 20s |
| **CP-12** | **Altitude Adjustment** | Adjust to outdoor cruise altitude | Barometer + GPS | 5s |
| **CP-13** | **Outdoor Waypoint 2** | Long-range GPS navigation | GPS | 25s |
| **CP-14** | **Outdoor Waypoint 3** | Complex waypoint with turn | GPS + Compass | 20s |
| **CP-15** | **Pre-Mission Position** | Position for object search area | GPS + Camera | 10s |

#### **🔍 Fase 4: Object Detection & Approach (Checkpoint 16-20)**
| Checkpoint | Task | Kriteria Sukses | Sensor Used | Waktu |
|------------|------|----------------|-------------|-------|
| **CP-16** | **Object Search Init** | Activate AI detection, scan area | AI Camera | 10s |
| **CP-17** | **Object Detection** | Identify target objects using YOLOv8 | AI Camera | 15s |
| **CP-18** | **Object Classification** | Classify objects, select pickup target | AI Camera | 10s |
| **CP-19** | **Approach Planning** | Calculate safe approach path | Camera + LiDAR | 8s |
| **CP-20** | **Approach Position** | Position above target object | Camera + GPS | 12s |

#### **🧲 Fase 5: Pickup & Transport (Checkpoint 21-24)**
| Checkpoint | Task | Kriteria Sukses | Sensor Used | Waktu |
|------------|------|----------------|-------------|-------|
| **CP-21** | **Fine Positioning** | Precise alignment with target object | Camera + LiDAR | 15s |
| **CP-22** | **Object Pickup** | Activate electromagnet, secure object | Electromagnet + Camera | 10s |
| **CP-23** | **Transport to Dropzone** | Carry object to designated drop area | GPS + All sensors | 30s |
| **CP-24** | **Object Drop** | Release object at correct dropzone | Electromagnet + Camera | 8s |

#### **🏠 Fase 6: Return & Landing (Checkpoint 25-26)**
| Checkpoint | Task | Kriteria Sukses | Sensor Used | Waktu |
|------------|------|----------------|-------------|-------|
| **CP-25** | **Return to Launch** | Navigate back to launch position | GPS + All sensors | 45s |
| **CP-26** | **Auto Landing** | Controlled descent and landing | GPS + Barometer + LiDAR | 20s |

---

### **📊 Mission Statistics:**
- **Total Checkpoints:** 26
- **Estimated Mission Time:** 6-8 minutes  
- **Success Criteria:** All 26 checkpoints completed
- **Scoring:** Progressive points + time bonus
- **Safety Features:** Emergency RTL at any checkpoint

### **🎯 Mission Flow Diagram:**
```
START → ARM → TAKEOFF → GPS_LOCK
  ↓
INDOOR_NAV (CP 4-8) → VISUAL_ODOMETRY → OBSTACLE_AVOID → WAYPOINTS
  ↓
TRANSITION (CP 9-15) → EXIT_INDOOR → GPS_PRIMARY → OUTDOOR_NAV  
  ↓
DETECTION (CP 16-20) → AI_SEARCH → OBJECT_ID → APPROACH_PLAN
  ↓
MISSION (CP 21-24) → PICKUP → TRANSPORT → DROPZONE → DROP
  ↓
RETURN (CP 25-26) → RTL → LANDING → MISSION_COMPLETE
```

### **🔧 Debug Features:**
- **Step Mode:** Execute one checkpoint at a time
- **Skip Mode:** Skip problematic checkpoints (with penalty)  
- **Replay Mode:** Retry failed checkpoints
- **Monitor Mode:** Real-time checkpoint status

### **⚠️ Failure Handling:**
- **Checkpoint Timeout:** Auto-advance with penalty
- **Sensor Failure:** Fallback to alternative sensors
- **Mission Abort:** Emergency RTL to launch point
- **Manual Override:** Competition official can take control

---

### **📝 Quick Reference Card:**

#### **Critical Checkpoints (Must Complete):**
- **CP-02:** Takeoff to 1m (Mission Start)
- **CP-09:** Exit Indoor (Phase Transition)
- **CP-17:** Object Detection (AI Critical)  
- **CP-22:** Object Pickup (Mission Task)
- **CP-24:** Object Drop (Mission Complete)
- **CP-26:** Safe Landing (Mission End)

#### **Most Common Failures:**
1. **CP-06:** Obstacle avoidance (LiDAR calibration)
2. **CP-17:** Object detection (lighting conditions)
3. **CP-22:** Object pickup (electromagnet alignment)
4. **CP-23:** Transport (object security)

#### **Emergency Procedures:**
```bash
# If stuck at checkpoint X:
just skip-checkpoint X        # Skip with penalty

# If sensor fails:
just switch-sensor backup     # Use backup sensor

# If mission critical failure:
just abort-rtl               # Return to launch safely
```

#### **Success Tips:**
- 🔋 **Battery:** Start with 100%, minimum 15 minutes flight time
- 🌤️ **Weather:** Calm winds (<5mph), good lighting for cameras
- 📡 **GPS:** Clear sky view, minimum 8 satellites
- 🧲 **Electromagnet:** Test pickup strength with actual objects
- 📹 **Camera:** Clean lenses, proper exposure settings

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

### **🏆 Competition Ready Status**

✅ **Hardware Architecture:** Pi 5 + Pixhawk4  
✅ **Altitude Setting:** 1.0m takeoff (sesuai regulasi)  
✅ **Sensor Integration:** 3x LiDAR + 3x Camera  
✅ **Payload System:** 2x GPIO Electromagnet  
✅ **Build System:** ROS 2 Humble + Python  
✅ **Universal Commands:** `kaertei` dan `just` tersedia  
✅ **Testing:** All nodes functional  

### **🎯 Competition Strategy & Scoring:**

#### **Point System (Estimasi):**
- **Checkpoint Completion:** 10 points per checkpoint (260 max)
- **Time Bonus:** 50 points if completed under 6 minutes
- **Precision Bonus:** 30 points for accurate pickup/drop
- **Full Mission Bonus:** 100 points for all 26 checkpoints
- **No Manual Intervention:** 50 points autonomy bonus

#### **Risk vs Reward Strategy:**
```
🟢 Conservative (Safe): 
   - Complete 20+ checkpoints reliably
   - Target Score: 200-250 points

🟡 Balanced (Recommended):  
   - Complete all 26 checkpoints
   - Target Score: 350-400 points

🔴 Aggressive (High Risk):
   - Speed run under 5 minutes
   - Target Score: 450+ points
```

#### **Competition Day Commands:**
```bash
# Pre-competition validation
just competition-ready

# Competition mode (full autonomous)  
just competition-run

# Real-time competition dashboard
just competition-dashboard
```  

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
