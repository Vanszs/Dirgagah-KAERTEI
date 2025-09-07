# 🏟️ ARENA CONFIGURATION GUIDE - KAERTEI 2025

## 📍 **Competition Day Setup**

### **Arena Layout Detection:**

#### **🔄 ARENA KANAN (Right Turn):**
```bash
# Edit konfigurasi untuk arena kanan
nano kaertei_drone/config/hardware_config.conf

# Set parameter ini:
[MISSION]
turn_direction = right   # ARENA KANAN
```

#### **🔃 ARENA KIRI (Left Turn):**
```bash
# Edit konfigurasi untuk arena kiri  
nano kaertei_drone/config/hardware_config.conf

# Set parameter ini:
[MISSION]
turn_direction = left    # ARENA KIRI
```

---

## 🎯 **Implementasi dalam Code**

### **Checkpoint 9 - NAVIGATE_TURN_DIRECTION:**
```python
# checkpoint_mission_mavros.py line ~500
def execute_turn(self):
    turn_direction = self.hw_config.get_turn_direction()
    
    if turn_direction.lower() == "left":
        # ARENA KIRI: Turn left sequence
        self.send_velocity_command(0, 0.5, 0)   # Move left 
        time.sleep(3)
        self.send_velocity_command(0.5, 0, 0)   # Move forward
        time.sleep(2)
    else:  
        # ARENA KANAN: Turn right sequence (default)
        self.send_velocity_command(0, -0.5, 0)  # Move right
        time.sleep(3) 
        self.send_velocity_command(0.5, 0, 0)   # Move forward
        time.sleep(2)
```

---

## 🚨 **CRITICAL GAPS DARI TASK 2 ANALYSIS**

### **1. SISTEM NAVIGATION GAP:**
```yaml
Problem: Turn direction hardcoded, tidak ada dynamic pathfinding
Impact: Tidak bisa adapt jika ada obstacle di arena
Solution: Implement LiDAR-based dynamic navigation
```

### **2. VISION SYSTEM GAP:**
```yaml  
Problem: Exit gate detection tidak diimplementasi
Impact: CP-15 (FIND_EXIT) tidak bisa berjalan
Solution: Implement exit gate detection dalam unified_vision_system.py
```

### **3. HARDWARE CONTROL GAP:**
```yaml
Problem: Dual magnet coordination missing
Impact: Front/back magnet bisa konflik saat pickup
Solution: Add magnet state coordination logic
```

### **4. MISSING CORE FILES:**
```yaml
Problem: checkpoint_mission_mavros.py main controller hilang
Impact: 12 checkpoint system tidak bisa jalan
Solution: Restore dari backup atau rewrite complete
```

---

## 💡 **QUICK FIX RECOMMENDATIONS**

### **For Arena Left/Right:**
1. **Pre-competition reconnaissance** - Survei arena layout
2. **Manual configuration** - Set turn_direction parameter  
3. **Test both scenarios** - Pastikan left/right turn work
4. **Backup plan** - Manual override jika auto navigation fail

### **For Critical Gaps:**
1. **Restore checkpoint_mission_mavros.py** - Priority 1
2. **Fix vision system** - Implement missing detection modes
3. **Add hardware validation** - Pickup/drop verification  
4. **Implement error recovery** - Fallback mechanisms

---

## 🔧 **Arena Test Commands**

### **Test Arena Kanan:**
```bash
# Set config ke right turn
sed -i 's/turn_direction = left/turn_direction = right/' config/hardware_config.conf

# Test mission
./run_checkpoint_mission.sh debug mavros
```

### **Test Arena Kiri:**  
```bash
# Set config ke left turn
sed -i 's/turn_direction = right/turn_direction = left/' config/hardware_config.conf

# Test mission
./run_checkpoint_mission.sh debug mavros
```

### **Competition Day Quick Switch:**
```bash
# Script untuk quick arena switch
./switch_arena_config.sh left   # Untuk arena kiri
./switch_arena_config.sh right  # Untuk arena kanan
```

---

**🏆 Competition Ready Status:**
- ✅ Arena configuration system implemented
- ❌ Dynamic pathfinding missing (critical gap)
- ❌ Vision system incomplete (critical gap)  
- ❌ Core mission controller missing (critical gap)

**Priority:** Fix critical gaps sebelum implement advanced arena detection!
