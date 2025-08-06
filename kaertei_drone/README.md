# KAERTEI 2025 FAIO - Clean File Structure 🚁

## 📁 NEW ORGANIZED STRUCTURE

```
kaertei_drone/                           # 🎯 MAIN PROJECT DIRECTORY
├── run_kaertei.sh                      # 🚀 MAIN LAUNCHER SCRIPT
│
├── src/                                # 📝 SOURCE CODE (ORGANIZED)
│   ├── mission/                        # 🎯 Mission Control
│   │   ├── checkpoint_mission_mavros.py      # 26-checkpoint system
│   │   ├── simplified_mission_control.py     # Simple 3-waypoint system
│   │   ├── simple_3waypoint_mission.py       # Alternative simple system
│   │   ├── simple_mission_node.py            # Basic mission node
│   │   └── simple_3wp_config.py              # 3-waypoint configuration
│   │
│   ├── hardware/                       # 🔧 Hardware Control
│   │   ├── camera_control_node.py            # 3x Camera management
│   │   ├── gpio_control_node.py              # GPIO/relay control
│   │   ├── lidar_control_node.py             # LiDAR sensors
│   │   ├── magnet_control_node.py            # Electromagnets
│   │   ├── magnet_control.py                 # Magnet utilities
│   │   └── emergency_controller.py           # Emergency procedures
│   │
│   ├── navigation/                     # 🧭 Navigation & Flight
│   │   ├── px4_waypoint_navigator.py         # PX4 GPS waypoints
│   │   ├── px4_waypoint_config.py            # Waypoint configuration
│   │   ├── waypoint_navigator.py             # Custom navigation
│   │   ├── waypoint_config.py                # Waypoint settings
│   │   ├── gps_monitor.py                    # GPS monitoring
│   │   ├── gps_waypoint_monitor.py           # GPS waypoint tracking
│   │   ├── flight_mode_switcher.py           # Flight mode control
│   │   ├── flight_state_monitor.py           # Flight state tracking
│   │   └── kalibrasi_navigator.py            # Navigation calibration
│   │
│   ├── vision/                         # 👁️ Computer Vision
│   │   └── unified_vision_system.py          # Unified YOLO detection
│   │
│   ├── monitoring/                     # 📊 System Monitoring
│   │   ├── sensor_monitor.py                 # Sensor monitoring
│   │   ├── system_health_monitor.py          # System health
│   │   └── topic_adapters.py                 # ROS topic bridges
│   │
│   ├── hardware_config.py              # Hardware configuration loader
│   ├── validate_ubuntu22.py            # System validation
│   ├── calibrate_hardware.py           # Hardware calibration
│   ├── test_node.py                    # Testing utilities
│   └── __init__.py                     # Python package init
│
├── config/                             # ⚙️ CONFIGURATION FILES
│   ├── hardware_config.conf                  # 🔧 Main hardware config
│   ├── competition_config.yaml               # Competition settings
│   ├── hexacopter_config.yaml                # Hexacopter parameters
│   ├── mavros_config.yaml                    # MAVROS settings
│   ├── params.yaml                           # ROS parameters
│   ├── setup.py                              # Python setup
│   ├── requirements.txt                      # Python dependencies
│   ├── package.xml                           # ROS package info
│   └── CMakeLists.txt                        # Build configuration
│
├── launch/                             # 🚀 LAUNCH FILES
│   ├── kaertei_pi5_system.launch.py          # Main system launcher
│   └── simple_3waypoint_system.launch.py     # Simple system launcher
│
├── scripts/                            # 📜 UTILITY SCRIPTS
│   ├── run_simplified_mission.sh             # Launch simplified mission
│   ├── run_simple_mission.sh                 # Launch simple mission
│   ├── competition_startup.sh                # Competition startup
│   ├── install_kaertei.sh                    # System installation
│   ├── setup_ubuntu.sh                       # Ubuntu setup
│   ├── test_hardware_pi5.sh                  # Hardware testing
│   ├── validate_system.sh                    # System validation
│   └── docker_runner.sh                      # Docker utilities
│
├── docs/                               # 📚 DOCUMENTATION
│   ├── ONNX_MODEL_PLACEMENT.md              # Model placement guide
│   └── TROUBLESHOOTING.md                   # Troubleshooting guide
│
├── models/                             # 🤖 AI MODELS
│   └── README.md                            # Model placement instructions
│
└── logs/                              # 📊 LOG FILES
    └── (runtime logs)                       # System logs during execution
```

## 🚀 HOW TO USE

### Quick Start Commands:
```bash
# Navigate to main directory
cd /home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone

# Launch debug mode (step-by-step)
./run_kaertei.sh debug checkpoint

# Launch autonomous mode (full auto)
./run_kaertei.sh auto checkpoint

# Launch simple 3-waypoint system
./run_kaertei.sh debug simple
```

### Available Systems:
1. **Checkpoint System**: 26-checkpoint full autonomous mission
2. **Simple System**: 3-waypoint basic navigation

### Available Modes:
- **debug**: Step-by-step execution with user confirmation
- **auto**: Fully autonomous execution

## 🎯 BENEFITS OF NEW STRUCTURE

### ✅ BEFORE vs AFTER:

**❌ OLD MESSY STRUCTURE:**
```
ros2_ws/src/drone_mvp/
├── drone_mvp/          # Nested confusion
│   ├── vision/         # More nesting
│   ├── config/         # Duplicate configs
│   └── 50+ mixed files # Everything mixed together
├── vision/             # Duplicate vision folder
├── config/             # Another config folder
└── 30+ root files      # Files everywhere
```

**✅ NEW CLEAN STRUCTURE:**
```
kaertei_drone/
├── src/                # Clean source organization
│   ├── mission/        # Mission control (5 files)
│   ├── hardware/       # Hardware control (6 files)  
│   ├── navigation/     # Navigation (9 files)
│   ├── vision/         # Vision (1 unified file)
│   └── monitoring/     # Monitoring (3 files)
├── config/             # All configs together
├── launch/             # All launch files
├── scripts/            # All scripts
├── docs/               # All documentation
└── models/             # AI model placement
```

### 🎯 Key Improvements:
- **50+ messy files** → **Organized into 7 functional categories**
- **3+ duplicate folders** → **Single clean structure**
- **Mixed file types** → **Separated by function and type**
- **Hard to find files** → **Logical functional grouping**
- **Complex navigation** → **Simple, clear directory structure**

## 📋 FILE MAPPING GUIDE

### 🎯 Mission Control (`src/mission/`):
- `checkpoint_mission_mavros.py` - **26 checkpoint system**
- `simplified_mission_control.py` - **3 waypoint system**

### 🔧 Hardware Control (`src/hardware/`):
- `camera_control_node.py` - **3x USB cameras**
- `gpio_control_node.py` - **2x electromagnet relays**
- `lidar_control_node.py` - **3x ToF sensors**

### 🧭 Navigation (`src/navigation/`):
- `px4_waypoint_navigator.py` - **GPS waypoint navigation**
- `flight_state_monitor.py` - **MAVROS bridge**

### 👁️ Vision (`src/vision/`):
- `unified_vision_system.py` - **YOLO object detection**

### ⚙️ Configuration (`config/`):
- `hardware_config.conf` - **Main hardware settings**

## 🚀 NEXT STEPS

1. **Place ONNX models** in `models/` directory
2. **Configure hardware** in `config/hardware_config.conf`
3. **Run system** with `./run_kaertei.sh debug checkpoint`
4. **Monitor logs** in `logs/` directory

---
*Structure reorganized for maximum clarity and efficiency* 🎯
