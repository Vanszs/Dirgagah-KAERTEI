# 🎯 MISSION CLARIFICATION - KAERTEI 2025 FAIO

## **PEMAHAMAN MISI YANG BENAR**

Berdasarkan analisis ulang konteks misi 12 checkpoint, berikut adalah clarification yang diperlukan:

---

## 🚁 **PICKUP & DROP LOGIC**

### **❌ SALAH PEMAHAMAN SEBELUMNYA:**
- Pickup dan drop memiliki waypoint tetap
- Menggunakan 3 waypoint sederhana
- Top camera untuk exit gate detection

### **✅ PEMAHAMAN YANG BENAR:**

#### **PICKUP Logic:**
- **TIDAK ADA waypoint tetap untuk pickup**
- Mengikuti logic checkpoint 12-mission:
  - **CP-03**: Search Item 1 (forward movement + front camera)
  - **CP-04**: Search Item 2 & Turn (turn logic + back camera)
  - **CP-05**: Drop Item 1 (dropzone detection)
  - **CP-06**: Drop Item 2 (dropzone detection)

#### **DROP Logic:**
- **TIDAK ADA waypoint tetap untuk drop**
- Drop locations ditentukan oleh:
  - Dropzone detection via vision system
  - Indoor mission area scanning
  - Real-time dropzone identification

#### **WAYPOINT 1-5 Logic:**
- **HANYA** diambil dari PX4/ArduPilot **SETELAH** berhasil drop bucket
- Bukan bagian dari pickup/drop phase
- Bagian dari outdoor GPS navigation phase

---

## 📷 **CAMERA CONFIGURATION UPDATE**

### **Hardware Reality:**
```
✅ Front Camera (index 0) - AVAILABLE
✅ Back Camera (index 2) - AVAILABLE  
❌ Top Camera (index 4) - TEMPORARILY NOT AVAILABLE
```

### **YOLO Usage:**
- **Front Camera**: Item detection, exit gate, dropzone
- **Back Camera**: Item detection, dropzone
- **Top Camera**: DISABLED (temporarily not available)

### **Impact pada Vision System:**
- Exit gate detection menggunakan **front camera** (bukan top)
- Item detection switch antara front/back cameras
- No top-down view untuk precision landing (use LiDAR instead)

---

## 🎯 **12 CHECKPOINT MISSION FLOW (CORRECTED)**

### **PHASE 1: Initialization (CP 1-2)**
```
CP-01: INIT        → ARM drone & initialize systems
CP-02: TAKEOFF     → Takeoff ke 1.0m altitude
```

### **PHASE 2: Indoor Mission (CP 3-6)**
```
CP-03: SEARCH_ITEM_1_FRONT    → Forward movement + front camera detection
CP-04: SEARCH_ITEM_2_BACK     → Turn logic + back camera detection  
CP-05: DROP_ITEM_1            → Dropzone detection + front magnet release
CP-06: DROP_ITEM_2            → Dropzone detection + back magnet release
```

### **PHASE 3: Exit & Outdoor Transition (CP 7)**
```
CP-07: FIND_EXIT              → Exit gate detection (front camera)
       ASCEND_TO_OUTDOOR      → Ascend ke 3m untuk outdoor
```

### **PHASE 4: GPS Navigation (CP 8-12)**
```
CP-08: GET_PX4_WAYPOINTS      → Retrieve waypoints 1-5 from PX4/ArduPilot
CP-09: AUTO_WAYPOINT_1        → Navigate to waypoint 1
CP-10: AUTO_WAYPOINT_2        → Navigate to waypoint 2  
CP-11: AUTO_WAYPOINT_3        → Navigate to waypoint 3
CP-12: COMPLETED              → Mission completion & landing
```

---

## ⚙️ **TECHNICAL IMPLICATIONS**

### **Hardware Config Changes:**
```properties
# cameras section
front_camera_index = 0          # ✅ Available
back_camera_index = 2           # ✅ Available  
# top_camera_index = 4          # ❌ Disabled

# vision section  
# YOLO only for front and back cameras
# Top camera detection methods disabled
```

### **Mission Logic Changes:**
```python
# Pickup/drop tidak menggunakan fixed waypoints
pickup_waypoints = None         # ❌ Removed
drop_waypoints = None          # ❌ Removed

# Waypoint 1-5 source
waypoint_source = "PX4_ARDUPILOT"  # Retrieved after bucket drop
waypoint_timing = "POST_DROP"      # Not pre-configured
```

### **Vision System Updates:**
```python
# Exit gate detection
exit_gate_camera = "front"      # Changed from "top"

# Multi-camera item detection
item_detection_cameras = ["front", "back"]  # Removed "top"

# Dropzone detection  
dropzone_cameras = ["front", "back"]        # Removed "top"
```

---

## 🔧 **REQUIRED CODE CHANGES**

### **1. Hardware Configuration**
- ✅ **DONE**: Updated `hardware_config.conf`
- ✅ **DONE**: Disabled top camera references
- ✅ **DONE**: Updated vision section comments

### **2. Mission Flow Updates**
- 🔄 **NEEDED**: Update mission checkpoint logic
- 🔄 **NEEDED**: Remove fixed pickup/drop waypoints
- 🔄 **NEEDED**: Add PX4 waypoint retrieval after drop

### **3. Vision System Updates**
- 🔄 **NEEDED**: Update camera switching logic
- 🔄 **NEEDED**: Remove top camera dependencies
- 🔄 **NEEDED**: Update exit gate detection to use front camera

### **4. Documentation Updates**
- 🔄 **NEEDED**: Update vision system integration guide
- 🔄 **NEEDED**: Update competition guide
- 🔄 **NEEDED**: Update hardware setup documentation

---

## 📋 **NEXT ACTIONS**

### **Priority 1: Mission Logic Correction**
1. Update mission checkpoint implementation
2. Remove fixed waypoint dependencies for pickup/drop
3. Implement PX4 waypoint retrieval logic

### **Priority 2: Vision System Updates**  
1. Update camera configuration logic
2. Remove top camera dependencies
3. Update exit gate detection implementation

### **Priority 3: Documentation Sync**
1. Update all markdown documentation
2. Sync configuration examples
3. Update troubleshooting guides

---

## ✅ **VALIDATION CHECKLIST**

### **Mission Logic:**
- [ ] Pickup follows checkpoint logic (no fixed waypoints)
- [ ] Drop follows checkpoint logic (no fixed waypoints)  
- [ ] Waypoint 1-5 retrieved from PX4 after drop
- [ ] 12 checkpoint flow correctly implemented

### **Hardware:**
- [x] Front camera configuration verified
- [x] Back camera configuration verified
- [x] Top camera properly disabled
- [ ] Camera switching logic updated

### **Vision System:**
- [ ] YOLO limited to front/back cameras
- [ ] Exit gate uses front camera
- [ ] No top camera dependencies

### **Documentation:**
- [x] Hardware config updated
- [ ] Mission flow documentation updated
- [ ] Vision system guide updated
- [ ] Competition guide updated

---

## 🏆 **MISSION SUCCESS CRITERIA**

Dengan pemahaman yang benar ini, sistem KAERTEI 2025 akan:

1. **Pickup Items**: Menggunakan real-time detection dan movement logic
2. **Drop Items**: Menggunakan dropzone detection, bukan fixed coordinates  
3. **GPS Navigation**: Menggunakan waypoints dari PX4 setelah drop completion
4. **Camera Usage**: Optimal dengan 2 cameras (front/back) saja
5. **Mission Flow**: Mengikuti 12 checkpoint logic yang sebenarnya

**STATUS**: ✅ **Clarification Complete - Ready for Implementation**
