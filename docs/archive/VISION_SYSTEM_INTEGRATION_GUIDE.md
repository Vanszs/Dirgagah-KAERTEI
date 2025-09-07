# KAERTEI 2025 FAIO - Vision System Integration Guide

## Overview
Sistem vision KAERTEI 2025 telah direorganisasi untuk misi 12-checkpoint dengan konfigurasi 2 kamera (front + back). Top kamera sementara tidak tersedia.

## Mission Flow - 12 Checkpoint System

### Corrected Flow:
```
CP 1-2: INIT → TAKEOFF
CP 3-6: SEARCH ITEMS → DROP ITEMS (no fixed waypoints, follow checkpoint logic)  
CP 7: EXIT GATE → OUTDOOR TRANSITION
CP 8-12: PX4 WAYPOINTS (retrieved after drop completion)
```

### Key Changes:
- **Pickup/Drop**: NO fixed waypoints, follows checkpoint detection logic
- **Camera Config**: Only front (index 0) + back (index 2) cameras
- **Waypoint 1-5**: Retrieved from PX4/ArduPilot AFTER successful bucket drop
- **Exit Gate**: Uses front camera (not top camera)

## Vision System Architecture

### Division-Based Organization
```
vision/
├── vision_manager.py          # Central coordinator
├── exit_gate_detector.py      # Portal alignment division
├── item_detector.py          # Pickup mission division  
├── dropzone_detector.py      # Drop mission division
```

### 1. Vision Manager (Central Coordinator)
**File**: `kaertei_drone/vision/vision_manager.py`

**Fungsi**:
- Central coordinator untuk semua vision modules
- YOLO model management dan loading
- Camera switching (front/bottom)
- Detection mode switching
- Alignment checking dan validation
- Debug visualization coordination

**Key Features**:
- YOLOv8 integration dengan fallback detection methods
- Multi-camera support dengan automatic switching
- Detection mode management (exit_gate, item, dropzone, disabled)
- Alignment threshold checking
- Real-time debug visualization

**ROS Topics**:
```bash
# Publishers
/vision/detection_mode        # Current detection mode
/vision/camera_switch        # Camera switching control (front/back only)
/vision/aligned              # General alignment status
/vision/alignment_error      # Alignment error vector

# Subscribers  
/vision/front/image          # Front camera feed
/vision/back/image           # Back camera feed (replaces bottom)
/vision/detection_mode       # Detection mode commands
```

### 2. Exit Gate Detector (Portal Alignment Division)
**File**: `kaertei_drone/vision/exit_gate_detector.py`

**Fungsi**:
- Specialized detection untuk exit gate/portal
- Multiple detection methods (YOLO, shape-based, edge-based)
- Portal center alignment untuk safe passage
- Gate dimension validation
- **UPDATE**: Uses FRONT camera instead of top camera

**Detection Methods**:
1. **YOLO Detection**: Deteksi menggunakan YOLOv8 untuk object "door", "gate", "entrance"
2. **Shape-based Detection**: Deteksi rectangular shapes dengan aspect ratio validation
3. **Edge-based Detection**: Deteksi menggunakan Canny edge detection dan contour analysis

**Alignment Logic**:
- Mengatur logic YOLO untuk exit gate agar keluar ditengah-tengah kotak portal exit
- Center alignment threshold dapat dikonfigurasi
- Real-time alignment error calculation
- **NOTE**: Now uses front camera for exit gate detection

**ROS Topics**:
```bash
# Publishers
/vision/exit_gate/detected    # Gate detection status
/vision/exit_gate/center      # Gate center point
/vision/exit_gate/aligned     # Gate alignment status
/vision/exit_gate/debug       # Debug visualization

# Subscribers
/vision/front/image           # Front camera feed (changed from top)
/vision/exit_gate/enable      # Enable/disable detection
```

### 3. Item Detector (Pickup Mission Division)
**File**: `kaertei_drone/vision/item_detector.py`

**Fungsi**:
- Item detection untuk pickup mission
- Forward search movement logic
- Pickup alignment checking
- Search progress monitoring

**Detection Methods**:
1. **YOLO Detection**: Deteksi common objects untuk pickup
2. **Color-based Detection**: Fallback detection berdasarkan warna
3. **Shape-based Detection**: Deteksi berdasarkan geometric shapes

**Search Logic**:
- Jika tidak ketemu item, trigger forward movement
- Search timeout dengan progress monitoring
- Pickup alignment threshold checking
- Integration dengan mission command system

**ROS Topics**:
```bash
# Publishers
/vision/item/detected         # Item detection status
/vision/item/center          # Item center point  
/vision/item/aligned         # Item alignment status
/vision/item/debug           # Debug visualization
/mission/command             # Mission commands (forward search)

# Subscribers
/vision/front/image          # Front camera feed (primary)
/vision/back/image           # Back camera feed (secondary)
/vision/item/enable          # Enable/disable detection
```

### 4. Dropzone Detector (Drop Mission Division)
**File**: `kaertei_drone/vision/dropzone_detector.py`

**Fungsi**:
- Dropzone detection untuk delivery mission
- Multiple container/basket detection
- Drop zone center calculation
- Basket count validation

**Detection Methods**:
1. **YOLO Detection**: Deteksi containers (bowl, basket, bucket, table, etc.)
2. **Shape Detection**: Circular dan rectangular container detection
3. **Color Detection**: Container detection berdasarkan common colors

**Drop Logic**:
- Multi-basket detection untuk dropzone identification
- Weighted center calculation dari multiple containers
- Basket count threshold untuk validation
- Drop alignment checking

**ROS Topics**:
```bash
# Publishers
/vision/dropzone/detected     # Dropzone detection status
/vision/dropzone/center       # Dropzone center point
/vision/dropzone/aligned      # Dropzone alignment status
/vision/dropzone/basket_count # Number of detected baskets
/vision/dropzone/debug        # Debug visualization

# Subscribers
/vision/front/image           # Front camera feed (primary)
/vision/back/image            # Back camera feed (secondary)  
/vision/dropzone/enable       # Enable/disable detection
```

## YOLO Integration

### Model Loading
```python
from ultralytics import YOLO

# Load YOLOv8 model
self.yolo_model = YOLO('yolov8n.pt')
```

### Object Classes per Division
- **Exit Gate**: door, gate, entrance, opening
- **Item**: bottle, cup, book, cell phone, mouse, keyboard, etc.
- **Dropzone**: bowl, basket, bucket, container, table, sink, etc.

### Confidence Thresholding
- Default confidence: 0.5
- Adjustable per module via parameters
- Fallback methods when confidence low

## OpenCV Manipulation

### Alignment Logic
Setiap detector memiliki logic alignment yang dapat dikonfigurasi:

```python
def check_alignment(self, object_center, image_shape):
    """Check if object aligned for action"""
    height, width = image_shape[:2]
    
    # Screen center
    screen_center_x = width // 2
    screen_center_y = height // 2
    
    # Calculate error
    error_x = object_center.x - screen_center_x
    error_y = object_center.y - screen_center_y
    
    # Check threshold
    threshold = self.alignment_threshold
    aligned = abs(error_x) < threshold and abs(error_y) < threshold
    
    return aligned, error_x, error_y
```

### Positioning Control
- Center alignment thresholds per object type
- Real-time error calculation dan feedback
- Integration dengan movement commands

## 12-Checkpoint Mission System

### Mission Node
**File**: `kaertei_drone/mission/checkpoint_mission_mavros.py`

**Features**:
- Complete 12-checkpoint mission flow
- Vision system integration
- PX4 waypoint system integration
- Automatic checkpoint progression
- **KEY**: Pickup/drop follow checkpoint logic (no fixed waypoints)

### Mission Checkpoints
```python
class CheckpointMission(Enum):
    # Phase 1: Initialization
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    
    # Phase 2: Indoor Mission (no fixed waypoints - follow detection logic)
    SEARCH_ITEM_1_FRONT = "SEARCH_ITEM_1_FRONT"
    SEARCH_ITEM_2_BACK = "SEARCH_ITEM_2_BACK"  
    DROP_ITEM_1 = "DROP_ITEM_1"
    DROP_ITEM_2 = "DROP_ITEM_2"
    
    # Phase 3: Exit & Outdoor Transition
    FIND_EXIT = "FIND_EXIT"
    ASCEND_TO_OUTDOOR = "ASCEND_TO_OUTDOOR"
    
    # Phase 4: GPS Navigation (waypoints retrieved from PX4 after drop)
    GET_PX4_WAYPOINTS = "GET_PX4_WAYPOINTS"
    AUTO_WAYPOINT_1 = "AUTO_WAYPOINT_1"
    AUTO_WAYPOINT_2 = "AUTO_WAYPOINT_2"
    AUTO_WAYPOINT_3 = "AUTO_WAYPOINT_3"
    
    COMPLETED = "COMPLETED"
```

## Installation dan Setup

### 1. Build System
```bash
cd /home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone
colcon build --packages-select kaertei_drone
source install/setup.bash
```

### 2. Install Dependencies
```bash
# YOLO dependencies
pip install ultralytics

# OpenCV (jika belum ada)
pip install opencv-python

# ROS2 dependencies sudah ter-handle via package.xml
```

### 3. Launch System

#### Complete 12-Checkpoint System
```bash
ros2 launch kaertei_drone kaertei_12checkpoint_system.launch.py
```

#### Vision System Only
```bash  
ros2 launch kaertei_drone vision_system.launch.py
```

#### Individual Nodes
```bash
# Vision manager
ros2 run kaertei_drone vision_manager

# Exit gate detector
ros2 run kaertei_drone exit_gate_detector

# Item detector  
ros2 run kaertei_drone item_detector

# Dropzone detector
ros2 run kaertei_drone dropzone_detector_new

# Simple mission system (for development)
ros2 run kaertei_drone simple_3waypoint_mission

# Main 12-checkpoint mission system
ros2 run kaertei_drone checkpoint_mission_mavros
```

## Configuration

### Parameters
Setiap vision module dapat dikonfigurasi via parameters:

```bash
# YOLO model path
yolo_model_path: 'yolov8n.pt'

# Confidence threshold
confidence_threshold: 0.5

# Alignment thresholds
center_alignment_threshold: 30  # pixels

# Detection areas
min_item_area: 500
max_item_area: 20000

# Timeouts
search_timeout: 30.0  # seconds
alignment_timeout: 10.0  # seconds
```

### Waypoint Configuration
```bash
# Use PX4 waypoint config
ros2 run kaertei_drone px4_waypoint_config --preset simple_3wp
```

## Usage

### 1. Start Mission
```bash
# Publish mission start command
ros2 topic pub /mission/start std_msgs/Bool "data: true"
```

### 2. Monitor Progress
```bash
# Monitor mission checkpoints
ros2 topic echo /mission/checkpoint

# Monitor vision detection
ros2 topic echo /vision/exit_gate/detected
ros2 topic echo /vision/item/detected  
ros2 topic echo /vision/dropzone/detected

# Monitor PX4 waypoint status (after drop completion)
ros2 topic echo /mission/px4_waypoints
```

### 3. Debug Visualization
```bash
# View debug images (2 cameras only)
ros2 run rqt_image_view rqt_image_view /vision/exit_gate/debug
ros2 run rqt_image_view rqt_image_view /vision/item/debug
ros2 run rqt_image_view rqt_image_view /vision/dropzone/debug

# Monitor camera feeds
ros2 run rqt_image_view rqt_image_view /vision/front/image
ros2 run rqt_image_view rqt_image_view /vision/back/image
```

### 4. Manual Control
```bash
# Manual checkpoint advancement
ros2 topic pub /mission/checkpoint_complete std_msgs/String "data: 'advance'"

# Enable/disable specific vision modules
ros2 topic pub /vision/exit_gate/enable std_msgs/Bool "data: true"
ros2 topic pub /vision/item/enable std_msgs/Bool "data: true"
ros2 topic pub /vision/dropzone/enable std_msgs/Bool "data: true"
```

## Debugging Tips

### Per Division Debugging
1. **Exit Gate Issues**: Check `/vision/exit_gate/debug` topic
2. **Item Detection Issues**: Check `/vision/item/debug` topic  
3. **Dropzone Issues**: Check `/vision/dropzone/debug` topic

### Common Issues
1. **YOLO Model Loading**: Ensure model path correct dan file exists
2. **Camera Feed**: Check camera topics `/vision/front/image` dan `/vision/bottom/image`
3. **Alignment Issues**: Adjust `center_alignment_threshold` parameters
4. **Detection Issues**: Lower `confidence_threshold` atau check lighting conditions

### Log Monitoring
```bash
# Monitor specific node logs
ros2 run kaertei_drone vision_manager --ros-args --log-level debug
ros2 run kaertei_drone exit_gate_detector --ros-args --log-level debug
```

## Integration Points

### Vision ↔ Mission Integration
- Vision detection results trigger mission checkpoint progression
- Mission system enables/disables appropriate vision modules
- Alignment results control drone movement dan actions

### Vision ↔ PX4 Integration  
- PX4 waypoint system handles navigation between waypoints
- Vision system handles precise alignment at each waypoint
- Integrated command system untuk movement dan positioning

### Vision ↔ Hardware Integration
- Magnet control integration untuk pickup/drop actions
- Camera switching control untuk appropriate detection angles
- Movement commands untuk alignment dan search patterns

## Future Enhancements

1. **Machine Learning Improvements**:
   - Custom trained models untuk competition-specific objects
   - Online learning untuk environment adaptation

2. **Vision Enhancements**:
   - Multi-object tracking
   - 3D position estimation
   - Stereo vision integration

3. **Mission Improvements**:
   - Dynamic waypoint adjustment
   - Failure recovery mechanisms
   - Performance optimization

Sistem vision sekarang sudah modular per divisi dan terintegrasi dengan mission flow 3-waypoint sederhana sesuai permintaan. Setiap divisi dapat di-debug secara terpisah dan YOLO integration tersedia di semua modules dengan OpenCV manipulation untuk precise positioning.
