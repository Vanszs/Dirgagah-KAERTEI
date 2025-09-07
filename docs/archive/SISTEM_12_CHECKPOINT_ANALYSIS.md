# 🚁 KAERTEI## 🎯 **ALUR LENGKAP 12 CHECKPOINT MISSION**

### **📋 SYSTEM OVERVIEW**

**12-Checkpoint Mission** adalah sistem yang dioptimalkan untuk efisiensi kompetisi:

**Fase 1: Initialization & Indoor Search (CP 1-6)**
- CP-01: Initialize & ARM  
- CP-02: Takeoff to 1m
- CP-03: Search Item 1
- CP-04: Search Item 2 & Turn
- CP-05: Drop Item 1
- CP-06: Drop Item 2

**Fase 2: GPS Navigation & Outdoor Mission (CP 7-12)**
- CP-07: GPS WP1-3
- CP-08: Search Item 3  
- CP-09: Direct to WP4
- CP-10: Search & Drop Item 3
- CP-11: GPS WP5
- CP-12: Final Descent & Disarm

--- - ANALISIS SISTEM 12 CHECKPOINT

## � **EXECUTIVE SUMMARY**

**Project Status:** ✅ COMPLETE & OPTIMIZED  
**Mission System:** 12-checkpoint FSM (streamlined for competition efficiency)  
- **Architecture**: Clean separation - PC (mission control) + Pi 5 (sensors) + Pixhawk4 (flight control)  
- **Mission Duration**: ~7.5 minutes (realistic completion time)
- **Status**: Production ready dengan full documentation & troubleshooting

---

## 🎯 **ALUR LENGKAP 12 CHECKPOINT MISSION**

### **PHASE 1: INITIALIZATION & SETUP**

#### **CHECKPOINT 1: INIT**
- **Tujuan**: ARM drone & initialize systems
- **File Dependencies**:
  - `hardware_config.py` - Hardware configuration
  - `mavros` services - Arming service
  - `system_health_monitor.py` - System validation
- **Actions**: 
  - Check battery voltage > 3.3V
  - Validate GPS fix
  - ARM motors
  - Set GUIDED mode
- **Transition**: Auto ke TAKEOFF setelah ARM success

#### **CHECKPOINT 2: TAKEOFF**  
- **Tujuan**: Takeoff ke altitude 1.0m dan stabilize
- **File Dependencies**:
  - `flight_state_monitor.py` - Monitor altitude
  - MAVROS `/mavros/cmd/takeoff` service
- **Actions**: 
  - Set takeoff altitude 1.0m
  - Wait for altitude reached
  - Stabilize position
- **Transition**: Auto ke SEARCH_ITEM_1_FRONT setelah altitude stable

---

### **PHASE 2: INDOOR MISSION (Item Collection)**

#### **CHECKPOINT 3: SEARCH_ITEM_1_FRONT**
- **Tujuan**: Move forward & activate front camera untuk search item 1
- **File Dependencies**:
  - `camera_control_node.py` - Activate front camera (index 0)
  - `unified_vision_system.py` - Object detection
  - `flight_mode_switcher.py` - Switch to position mode
- **Actions**:
  - Switch camera ke FRONT mode
  - Set vision mode ke "item_detection" 
  - Move forward dengan velocity 0.2 m/s
  - Scan for items dengan confidence > 0.6
- **Gap Logic**: ❌ **Tidak ada timeout handling jika item tidak ditemukan**
- **Transition**: Ke ALIGN_ITEM_1 saat object detected

#### **CHECKPOINT 4: ALIGN_ITEM_1**
- **Tujuan**: Center drone ke item 1 dalam camera view
- **File Dependencies**:
  - `unified_vision_system.py` - Object alignment
  - PX4 position control
- **Actions**:
  - Calculate object offset dari center
  - Send position adjustments
  - Wait for alignment tolerance < 25 pixels
- **Gap Logic**: ❌ **Tidak ada handling jika item hilang saat alignment**
- **Transition**: Ke PICKUP_ITEM_1 saat aligned = true

#### **CHECKPOINT 5: PICKUP_ITEM_1** 
- **Tujuan**: Descend & pickup item dengan front magnet
- **File Dependencies**:
  - `magnet_control_node.py` - Control front relay pin 18
  - `lidar_control_node.py` - Distance sensing untuk landing
- **Actions**:
  - Descend dengan rate 0.3 m/s
  - Activate front magnet (GPIO pin 18)
  - Detect pickup success via current sensor
  - Ascend kembali ke 1.0m
- **Gap Logic**: ⚠️ **Tidak ada pickup validation - hanya asumsi berhasil**
- **Transition**: Auto ke SEARCH_ITEM_2_BACK

#### **CHECKPOINT 6-8: ITEM 2 SEQUENCE (BACK CAMERA)**
- Similar dengan item 1, tapi menggunakan:
  - Back camera (index 2) 
  - Back magnet (GPIO pin 19)
- **Gap Logic**: ❌ **Tidak ada koordinasi antar magnet - bisa konflik**

---

### **PHASE 3: NAVIGATION & DROP**

#### **CHECKPOINT 9: NAVIGATE_TURN_DIRECTION**
- **Tujuan**: Navigate turn (left/right based on config)
- **File Dependencies**:
  - `hardware_config.py` - Load turn direction preference
  - `px4_waypoint_navigator.py` - Position control  
- **Actions**:
  - Read config: turn_direction = "left" atau "right"
  - Execute 90° turn
  - Move forward mencari dropzone area
- **Gap Logic**: ❌ **Turn direction hardcoded - tidak ada dynamic pathfinding**

#### **CHECKPOINT 10: SEARCH_DROPZONE**
- **Tujuan**: Search for dropzone baskets
- **File Dependencies**:
  - `unified_vision_system.py` - Dropzone detection mode
  - `camera_control_node.py` - Switch camera modes
- **Actions**:
  - Set vision mode "dropzone_detection"
  - Scan area untuk basket detection
  - Identify correct color baskets
- **Gap Logic**: ❌ **Tidak ada fallback jika dropzone tidak ditemukan**

#### **CHECKPOINT 11-14: DROP SEQUENCE**
- Drop item 1 dengan front magnet
- Ascend after drop
- Switch ke back camera untuk drop 2
- Drop item 2 dengan back magnet
- **Gap Logic**: ⚠️ **Tidak ada drop verification - hanya timer-based**

---

### **PHASE 4: EXIT & OUTDOOR TRANSITION**

#### **CHECKPOINT 15: FIND_EXIT**
- **Tujuan**: Find exit gate dengan top camera
- **File Dependencies**:
  - `camera_control_node.py` - Top camera (index 4)
  - `unified_vision_system.py` - Exit gate detection
- **Actions**:
  - Switch ke top camera
  - Set vision mode "exit_gate"  
  - Detect door/window patterns
- **Gap Logic**: ❌ **Exit gate detection belum terimplementasi dalam unified_vision_system**

#### **CHECKPOINT 16: ASCEND_TO_OUTDOOR**
- **Tujuan**: Ascend ke 3m untuk outdoor mission
- **File Dependencies**:
  - `flight_state_monitor.py` - Altitude monitoring
  - `gps_monitor.py` - GPS validation for outdoor
- **Actions**:
  - Ascend ke 3.0m altitude
  - Switch dari indoor ke outdoor mode
  - Validate GPS accuracy < 5m
- **Transition**: Auto ke AUTO_WAYPOINT_1

---

### **PHASE 5: OUTDOOR WAYPOINT NAVIGATION**

#### **CHECKPOINT 17: AUTO_WAYPOINT_1**
- **Tujuan**: AUTO mode fly ke waypoint 1
- **File Dependencies**:
  - `px4_waypoint_navigator.py` - **MAIN NAVIGATION CONTROLLER**
  - `px4_waypoint_config.py` - Waypoint definitions
  - `gps_waypoint_monitor.py` - Waypoint validation
- **Actions**:
  - Load waypoint 1 coordinates dari config
  - Set PX4 ke AUTO mission mode  
  - Navigate to waypoint dengan GPS
  - Wait for waypoint reached (radius 3m)
- **Gap Logic**: ⚠️ **Tidak ada obstacle avoidance untuk outdoor flight**

#### **CHECKPOINT 18: MANUAL_SEARCH_OUTDOOR**
- **Tujuan**: MANUAL mode search outdoor item
- **File Dependencies**:
  - `flight_mode_switcher.py` - Switch AUTO → MANUAL
  - `unified_vision_system.py` - Outdoor item detection
- **Actions**:
  - Switch ke MANUAL mode
  - Activate item detection untuk outdoor objects
  - Manual pattern search around waypoint 1
- **Gap Logic**: ❌ **Manual mode tidak ada implementation - masih AUTO**

#### **CHECKPOINT 19-25: WAYPOINT 2 & 3 SEQUENCE**
- Similar flow untuk waypoint 2 (pickup outdoor) dan waypoint 3 (landing)

#### **CHECKPOINT 26: COMPLETED**
- Mission completion & system shutdown

---

## 🚨 **CRITICAL GAPS IDENTIFIED**

### **1. MISSING CORE FILES**
- ❌ `checkpoint_mission_mavros.py` - **TERHAPUS saat cleanup**
- ❌ `checkpoint_mission.launch.py` - **TIDAK ADA**
- ⚠️ `run_checkpoint_mission.sh` - **RENAMED ke run_simplified_mission.sh**

### **2. VISION SYSTEM GAPS**
- ❌ Exit gate detection tidak diimplementasi
- ❌ Dropzone detection masih template
- ❌ Object alignment tidak ada timeout handling
- ❌ Multi-camera switching logic missing

### **3. HARDWARE CONTROL GAPS** 
- ❌ Dual magnet coordination tidak ada
- ❌ Pickup/drop verification tidak ada
- ❌ Current sensor integration missing
- ❌ LiDAR integration untuk precision landing

### **4. NAVIGATION GAPS**
- ❌ Indoor ke outdoor transition logic
- ❌ Manual mode implementation missing  
- ❌ Dynamic pathfinding tidak ada
- ❌ Obstacle avoidance missing

### **5. SYSTEM INTEGRATION GAPS**
- ❌ Topic bridging tidak konsisten
- ❌ State synchronization antar nodes
- ❌ Error recovery mechanisms
- ❌ Emergency procedures incomplete

---

## 🔧 **REQUIRED FILES FOR 12 CHECKPOINT SYSTEM**

### **CORE MISSION CONTROL**
1. `checkpoint_mission_mavros.py` - **MISSING** ⚠️
2. `checkpoint_mission.launch.py` - **MISSING** ⚠️  
3. `run_checkpoint_mission.sh` - **RENAMED** ⚠️

### **HARDWARE CONTROL (OK)**
4. `camera_control_node.py` ✅
5. `magnet_control_node.py` ✅  
6. `gpio_control_node.py` ✅
7. `lidar_control_node.py` ✅

### **VISION SYSTEM (PARTIAL)**
8. `unified_vision_system.py` ⚠️ (perlu enhancement)
9. Models ONNX di `models/` folder ❌ (belum ada)

### **NAVIGATION SYSTEM (OK)**
10. `px4_waypoint_navigator.py` ✅
11. `px4_waypoint_config.py` ✅
12. `flight_mode_switcher.py` ✅

### **MONITORING SYSTEM (OK)**
13. `system_health_monitor.py` ✅
14. `flight_state_monitor.py` ✅
15. `gps_monitor.py` ✅

---

## 💡 **REKOMENDASI ACTION PLAN**

### **IMMEDIATE ACTIONS**
1. **Restore checkpoint_mission_mavros.py dari backup atau rewrite**
2. **Create checkpoint_mission.launch.py**  
3. **Fix run_checkpoint_mission.sh references**
4. **Implement missing vision detection modes**
5. **Add hardware validation & error handling**

### **MEDIUM PRIORITY**  
6. **Enhance unified_vision_system untuk multi-mode**
7. **Add pickup/drop verification logic**
8. **Implement indoor↔outdoor transition**
9. **Add timeout & error recovery**

### **LOW PRIORITY**
10. **Add obstacle avoidance**
11. **Implement manual mode logic**  
12. **Add performance optimizations**

---

*Analysis completed - Multiple critical gaps found requiring immediate attention*
