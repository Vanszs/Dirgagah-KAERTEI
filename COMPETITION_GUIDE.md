# 🏆 COMPETITION GUIDE - KAERTEI 2025 FAIO

## **Panduan Kompetisi FAIO**

*Dokumen ini menjelaskan cara menggunakan drone untuk kompetisi FAIO*

---

## 🎯 **Overview Kompetisi FAIO**

**FAIO (Festival Aeromodelling Indonesia Open)** adalah kompetisi drone terbesar di Indonesia. KAERTEI 2025 adalah sistem drone hexacopter otonom yang dibuat khusus untuk misi 12-checkpoint.

### **Misi Utama:**
1. **Takeoff otomatis** dari start point
2. **Navigasi 12 checkpoint** secara berurutan
3. **Object detection & pickup** dengan AI + magnet
4. **Transport & drop** object ke dropzone
5. **Return to launch** dan landing otomatis

### **Scoring System:**
- **Checkpoint completion:** Point per checkpoint
- **Time bonus:** Faster completion = more points  
- **Accuracy bonus:** Precise navigation = bonus points
- **Mission completion:** Full mission = major bonus

---

## ⏰ **Timeline Kompetisi**

### **Sebelum Kompetisi (1 Minggu):**
```bash
# Complete system check
just competition-ready

# Practice run full mission
just test-mission

# Hardware inspection
just hardware-test

# Backup semua konfigurasi
just backup-config
```

### **Hari H-1 (Setup Day):**
```bash
# Transport setup
just transport-mode

# Setup di venue
just venue-setup

# Final testing
just final-test

# Register system
just register-competition
```

### **Hari H (Competition Day):**
```bash
# Pre-flight check
just pre-flight

# Competition mode
just competition-mode

# Mission execution
just run-mission

# Post-mission data
just post-mission
```

---

## 🚁 **Pre-Competition Preparation**

### **1. Hardware Preparation**

#### **Physical Inspection:**
```bash
# Full hardware check
just inspect-all

# Propeller balance check
just check-props

# Battery voltage check
just check-battery

# All connections secure
just check-connections
```

#### **Calibration Check:**
```bash
# Compass calibration (outdoor)
just calibrate-compass

# Accelerometer level check
just calibrate-accel

# Camera focus & exposure
just calibrate-camera

# GPS accuracy check
just check-gps-accuracy
```

### **2. Software Validation**

#### **System Test:**
```bash
# Complete system validation
just test-all

# Mission logic test
just test-mission-logic

# Failsafe procedures test
just test-failsafe

# Emergency stop test
just test-emergency
```

#### **Performance Benchmark:**
```bash
# Flight performance test
just benchmark-flight

# AI detection accuracy
just benchmark-vision

# Navigation precision
just benchmark-navigation

# Mission timing analysis
just benchmark-timing
```

---

## 🎮 **Competition Day Operations**

### **Setup di Venue:**

#### **1. Physical Setup:**
```bash
# Setup base station
just setup-base-station

# Connect all hardware
just connect-hardware

# Power on sequence
just power-on-sequence

# System startup
just startup-competition
```

#### **2. Pre-Flight Check:**
```bash
# Complete pre-flight checklist
just pre-flight-checklist

# GPS signal quality
just check-gps-signal

# Communication link test
just test-comm-link

# Mission waypoint verification
just verify-waypoints
```

### **Mission Execution:**

#### **Competition Commands:**
```bash
# Start competition mode
just competition-start

# Monitor mission progress
just mission-monitor

# Emergency interventions (jika diperlukan)
just emergency-stop      # Stop immediately
just emergency-land      # Force landing
just emergency-rtl       # Return to launch
```

#### **Real-time Monitoring:**
```bash
# Mission dashboard
just mission-dashboard

# Telemetry monitoring
just telemetry-monitor

# System health monitoring
just health-monitor

# Performance metrics
just performance-monitor
```

---

## 🎯 **12-Checkpoint Mission Details**

### **Checkpoint Categories:**

#### **Navigation Checkpoints (1-15):**
- **Indoor navigation:** Visual odometry + LiDAR
- **Outdoor navigation:** GPS + compass
- **Transition points:** Indoor ↔ Outdoor
- **Altitude management:** 1.0m standard height

#### **Detection Checkpoints (16-22):**
- **Object recognition:** YOLOv8 AI detection
- **Target acquisition:** Camera-based targeting
- **Approach planning:** Safe approach to objects
- **Pickup preparation:** Magnet system activation

#### **Mission Task Checkpoints (9-12):**
- **Object pickup:** Electromagnet control
- **Transport phase:** Secure object transport
- **Dropzone navigation:** Precise positioning
- **Object release:** Controlled drop mechanism

### **Mission Flow:**
```
Start Position
    ↓
1-3: ARM → TAKEOFF → GPS_LOCK
    ↓
4-8: INDOOR_NAV → CAMERA_NAV → LIDAR_AVOID
    ↓  
9-15: OUTDOOR_TRANS → GPS_NAV → WAYPOINT_SEQ
    ↓
16-20: OBJ_SEARCH → OBJ_DETECT → APPROACH_PREP
    ↓
21-22: TARGET_ACQ → PICKUP_SEQ
    ↓
23-24: TRANSPORT → DROPZONE_NAV → DROP_SEQ  
    ↓
11-12: RTL → LANDING
    ↓
Mission Complete
```

---

## 📊 **Real-Time Mission Control**

### **Mission Dashboard:**
```bash
# Launch mission control interface
just mission-control

# Monitor all systems
just system-monitor

# Track checkpoint progress
just checkpoint-tracker

# View live telemetry
just live-telemetry
```

### **Performance Metrics:**
- **Mission Progress:** X/12 checkpoints completed
- **Flight Time:** Elapsed time vs estimated
- **System Health:** All subsystems status
- **Position Accuracy:** GPS/Vision positioning error
- **Battery Status:** Remaining flight time

### **Key Indicators:**
- 🟢 **Green:** Normal operation
- 🟡 **Yellow:** Warning condition
- 🔴 **Red:** Critical condition requiring intervention

---

## ⚠️ **Emergency Procedures**

### **Emergency Situations:**

#### **Communication Loss:**
- **Auto RTL:** Return to Launch otomatis
- **Manual Override:** Switch ke manual control
- **Emergency Land:** Land di posisi aman terdekat

#### **Hardware Failure:**
```bash
# GPS failure
just emergency-vision-nav

# Camera failure  
just emergency-lidar-nav

# Motor failure
just emergency-land-now

# Battery critical
just emergency-rtl-now
```

#### **Mission Abort:**
```bash
# Abort mission safely
just abort-mission

# Emergency landing
just emergency-land

# System shutdown
just emergency-shutdown
```

---

## 🏅 **Scoring Optimization**

### **Maximum Points Strategy:**

#### **Speed Optimization:**
- **Efficient waypoints:** Minimal flight path
- **Smooth transitions:** Reduce hover time
- **Parallel processing:** AI detection while flying

#### **Accuracy Bonus:**
- **Precise positioning:** GPS + vision fusion
- **Stable hovering:** Reduce position drift
- **Clean object pickup:** First-try success

#### **Completion Bonus:**
- **All 12 checkpoints:** Major point bonus
- **No manual intervention:** Autonomy bonus
- **Clean mission:** No emergency stops

### **Risk Management:**
- **Conservative altitude:** Safe clearance
- **Redundant sensors:** Multiple navigation sources
- **Graceful degradation:** Fallback modes
- **Early abort strategy:** If mission success unlikely

---

## 📋 **Post-Mission Analysis**

### **Data Collection:**
```bash
# Download flight logs
just download-logs

# Mission analysis report
just mission-analysis

# Performance metrics
just performance-report

# Optimization recommendations
just optimization-report
```

### **Competition Debrief:**
- **Mission completion rate:** X/12 checkpoints
- **Time analysis:** vs competition average
- **System performance:** Hardware utilization
- **Areas for improvement:** Next competition prep

---

## 🏆 **Competition Day Checklist**

### **☑️ Pre-Competition:**
- [ ] Hardware inspection complete
- [ ] All calibrations verified
- [ ] Software test passed
- [ ] Mission practice completed
- [ ] Emergency procedures tested
- [ ] Backup configurations ready
- [ ] Team roles assigned
- [ ] Competition registration complete

### **☑️ Competition Setup:**
- [ ] Base station setup
- [ ] Hardware connections verified
- [ ] Communication links tested
- [ ] GPS signal quality good
- [ ] Battery fully charged
- [ ] Propellers balanced
- [ ] Safety equipment ready
- [ ] Mission waypoints loaded

### **☑️ Mission Execution:**
- [ ] Pre-flight checklist complete
- [ ] Mission control dashboard active
- [ ] Emergency procedures briefed
- [ ] Team communication ready
- [ ] Competition mode activated
- [ ] Mission launch ready
- [ ] Data logging active
- [ ] Post-mission procedures ready

---

## 🎖️ **Success Tips**

### **Technical Tips:**
- **Test early, test often:** Regular practice runs
- **Multiple backup plans:** Redundancy in all systems
- **Conservative settings:** Reliability over speed
- **Monitor everything:** Real-time system health

### **Team Tips:**
- **Clear roles:** Everyone knows their job
- **Communication protocol:** Standard procedures
- **Stay calm:** Stress affects decision making
- **Trust the system:** Let automation work

### **Competition Tips:**
- **Arrive early:** Extra setup time
- **Watch other teams:** Learn from others
- **Document everything:** For next competition
- **Enjoy the experience:** Learning opportunity

---

## 🏆 **KAERTEI 2025 - Competition Ready!**

<div align="center">

**🚁 12-Checkpoint Autonomous Mission System**  
**✅ Competition Tested & Validated**  
**🎯 Optimized for FAIO Competition**

**Good Luck untuk KAERTEI 2025! 🏆**

</div>
