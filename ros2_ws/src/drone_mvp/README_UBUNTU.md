# KAERTEI 2025 FAIO - Ubuntu 22.04 Competition Ready

<div align="center">

![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange?style=for-the-badge&logo=ubuntu&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-green?style=for-the-badge&logo=python&logoColor=white)
![Competition Ready](https://img.shields.io/badge/Competition-Ready-gold?style=for-the-badge&logo=trophy&logoColor=white)

**🏆 26-Checkpoint Autonomous Drone Mission System**  
*Zero to competition ready in 3 commands*

</div>

## 🚀 Quick Start (Ubuntu 22.04)

```bash
# 1. Complete setup (one-time)
./setup_ubuntu.sh

# 2. Validate system
just test

# 3. Run competition mission
just mission-auto
```

**That's it! 🎉 You're competition ready!**

---

## 📋 Competition Workflow

### 🎯 Competition Day Commands
```bash
just mission-debug     # Practice run (recommended first)
just mission-auto      # Official competition run
just emergency         # Emergency procedures
```

### 🔧 Setup & Maintenance
```bash
just setup             # Complete Ubuntu 22.04 setup
just test              # System health check
just doctor            # Diagnose issues
just status            # System overview
```

---

## 🏗️ System Architecture

### 🎯 26 Checkpoint Mission System

Our competition system implements a comprehensive 26-checkpoint autonomous mission:

1. **INIT** - System initialization and safety checks
2. **PREFLIGHT** - Pre-flight hardware validation
3. **ARM** - Vehicle arming and motor check
4. **TAKEOFF** - Autonomous takeoff to mission altitude
5. **WAYPOINT_1** through **WAYPOINT_15** - GPS navigation points
6. **PATTERN_SEARCH** - Computer vision pattern recognition
7. **OBJECT_DETECTION** - Target object identification
8. **PRECISION_LANDING** - Accurate landing sequence
9. **CARGO_PICKUP** - Electromagnet cargo acquisition
10. **CARGO_TRANSPORT** - Transport to destination
11. **CARGO_DROP** - Precise cargo delivery
12. **RETURN_HOME** - Navigate back to launch point
13. **FINAL_LANDING** - Competition completion landing
14. **DISARM** - Safe shutdown sequence

### 🤖 Dual Mission Architecture

- **MAVROS Node** (`checkpoint_mission_mavros.py`) - ROS 2 integrated mission
- **Standalone Node** (`checkpoint_mission_node.py`) - Direct Python execution
- **Smart Launcher** - Automatically selects best execution method

### 🔧 Hardware Abstraction

- **Dummy Mode** - Compete without physical hardware
- **Real Hardware** - Full PX4/ArduPilot integration
- **Hybrid Mode** - Mix dummy and real components

---

## 📁 Project Structure

```
ros2_ws/src/drone_mvp/
├── 🎯 Mission Core
│   ├── checkpoint_mission_mavros.py    # ROS 2 MAVROS mission
│   ├── checkpoint_mission_node.py      # Standalone mission
│   └── mission_state_machine.py        # Core state logic
├── 🔧 Configuration
│   ├── config/hardware_config.conf     # Hardware settings
│   ├── config/mission_params.yaml      # Mission parameters
│   └── config/gps_waypoints.conf       # GPS coordinates
├── 🛠️ Hardware
│   ├── hardware/
│   │   ├── flight_controller.py        # PX4/ArduPilot interface
│   │   ├── camera_module.py            # Computer vision
│   │   ├── electromagnet.py            # Cargo system
│   │   └── dummy_hardware.py           # Simulation mode
├── 🚀 Automation
│   ├── Justfile                        # Command automation
│   ├── setup_ubuntu.sh                 # Ubuntu 22.04 setup
│   ├── launch_mission_ubuntu.sh        # Mission launcher
│   └── validate_ubuntu.py              # System validation
└── 📊 Utilities
    ├── debug_v2.py                     # System diagnosis
    ├── simulate_mission.py             # Mission simulation
    └── competition_quickstart.sh       # Emergency procedures
```

---

## 🎯 Competition Features

### ✅ Competition Requirements Met

- **✅ 26 Checkpoints** - Full mission state machine
- **✅ Autonomous Navigation** - GPS waypoint following
- **✅ Computer Vision** - Pattern recognition and object detection
- **✅ Cargo System** - Electromagnet pickup and delivery
- **✅ Safety Systems** - Multiple fallback modes
- **✅ Real-time Monitoring** - Mission status and telemetry
- **✅ Hardware Abstraction** - Works with or without hardware

### 🏆 Competition Advantages

- **Zero Setup Time** - One command installation
- **Dummy Mode** - Compete without hardware
- **Debug Mode** - Step-by-step mission control
- **Emergency Procedures** - Quick recovery systems
- **Cross-compatible** - Works with PX4 and ArduPilot

---

## 🔧 Technical Specifications

### 🐧 Ubuntu 22.04 LTS Optimized

- **ROS 2 Humble Hawksbill** - Latest LTS robotics framework
- **Python 3.10+** - Modern Python with async support
- **OpenCV 4.x** - Advanced computer vision
- **MAVROS** - MAVLink communication protocol
- **Just Command Runner** - Streamlined task automation

### 🤖 Supported Hardware

- **Flight Controllers**: PX4, ArduPilot
- **Cameras**: USB, CSI, IP cameras
- **Sensors**: GPS, IMU, magnetometer
- **Actuators**: Electromagnets, servos
- **Communication**: USB, radio telemetry

### 📡 Communication Protocols

- **MAVLink v2** - Primary flight controller communication
- **ROS 2 DDS** - Inter-node communication
- **Serial/USB** - Hardware interfaces
- **UDP/TCP** - Network communication

---

## 🎮 Usage Guide

### 🚀 First Time Setup

1. **Install Ubuntu 22.04 LTS** (recommended)
2. **Clone repository** to `/home/vanszs/Documents/ros2/`
3. **Run setup script**: `./setup_ubuntu.sh`
4. **Validate installation**: `just test`

### 🎯 Competition Preparation

```bash
# 1. System check
just doctor

# 2. Update GPS coordinates
nano config/hardware_config.conf

# 3. Hardware test
just test-hardware

# 4. Practice mission
just mission-debug

# 5. Competition ready!
just mission-auto
```

### 🐛 Debug Mode Usage

```bash
# Start debug mission
just mission-debug

# Mission will pause at each checkpoint
# Press ENTER to continue to next checkpoint
# Type 'skip' to skip current checkpoint
# Type 'abort' to safely abort mission
```

### ⚡ Direct Execution

```bash
# Bypass ROS 2 for fastest startup
python3 checkpoint_mission_node.py --debug
python3 checkpoint_mission_node.py --autonomous
```

---

## 🔧 Configuration

### 📍 GPS Waypoints

Edit `config/hardware_config.conf`:

```ini
[GPS_WAYPOINTS]
HOME_LAT = -6.200000
HOME_LON = 106.816666
WAYPOINT_1_LAT = -6.201000
WAYPOINT_1_LON = 106.817000
# ... up to WAYPOINT_15
```

### 🔌 Hardware Settings

```ini
[HARDWARE]
FLIGHT_CONTROLLER_PORT = /dev/ttyUSB0
CAMERA_INDEX = 0
ELECTROMAGNET_PIN = 18
USE_DUMMY_HARDWARE = False
```

### 🎯 Mission Parameters

```ini
[MISSION]
TAKEOFF_ALTITUDE = 10.0
CRUISE_SPEED = 5.0
LANDING_SPEED = 1.0
PATTERN_SEARCH_TIME = 30
DEBUG_MODE = True
```

---

## 🆘 Troubleshooting

### ❌ Common Issues

**"ROS 2 not found"**
```bash
source /opt/ros/humble/setup.bash
# Or run: just setup
```

**"Permission denied on /dev/ttyUSB0"**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**"Python modules missing"**
```bash
source ros2_env/bin/activate
pip install -r requirements.txt
```

**"Workspace not built"**
```bash
just build
# Or: colcon build --packages-select drone_mvp
```

### 🩺 System Diagnosis

```bash
just doctor           # Comprehensive system check
just status           # Quick system overview
python3 validate_ubuntu.py  # Detailed validation
```

### 🚨 Emergency Procedures

```bash
just emergency        # Emergency command reference

# Kill all processes
pkill -f ros2; pkill -f mavros; pkill -f python3

# Reset USB devices
sudo rmmod usbserial; sudo modprobe usbserial

# Direct mission launch
python3 checkpoint_mission_node.py --debug
```

---

## 🏆 Competition Checklist

### ✅ Pre-Competition

- [ ] `just setup` - System setup completed
- [ ] `just test` - All tests passing
- [ ] `just doctor` - No critical issues
- [ ] Update GPS coordinates in `config/hardware_config.conf`
- [ ] Test hardware connections: `ls /dev/tty{USB,ACM}*`
- [ ] Verify camera access: `ls /dev/video*`
- [ ] Test electromagnet system
- [ ] Calibrate compass and GPS

### 🎯 Competition Day

- [ ] `just mission-debug` - Practice run completed
- [ ] RC transmitter ready for manual override
- [ ] QGroundControl available for monitoring
- [ ] Emergency procedures reviewed
- [ ] `just mission-auto` - Final competition run

### 🚨 Emergency Equipment

- [ ] RC transmitter (manual override)
- [ ] QGroundControl (monitoring)
- [ ] Backup flight controller
- [ ] Extra batteries
- [ ] USB cables and adapters

---

## 📚 Documentation

### 🔗 Key Files

- **[Justfile](Justfile)** - All available commands
- **[Hardware Config](config/hardware_config.conf)** - Configuration settings
- **[Mission Nodes](checkpoint_mission_mavros.py)** - Core mission logic
- **[Debug Tools](debug_v2.py)** - System diagnosis
- **[Setup Script](setup_ubuntu.sh)** - Ubuntu installation

### 📖 External Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [PX4 Autopilot](https://px4.io/)
- [Just Command Runner](https://just.systems/)

---

## 🤝 Support

### 💬 Getting Help

1. **System Issues**: Run `just doctor`
2. **Mission Problems**: Check logs in `/tmp/drone_mission.log`
3. **Hardware Issues**: Verify connections and permissions
4. **Emergency**: Use `just emergency` for quick reference

### 🐛 Bug Reports

If you encounter issues:

1. Run `just doctor` and save output
2. Check mission logs
3. Include system information (`just status`)
4. Describe steps to reproduce

---

<div align="center">

**🏆 KAERTEI 2025 FAIO Competition Ready**

*Zero to competition in 3 commands | Ubuntu 22.04 Optimized | 26 Checkpoints*

**Good luck in the competition! 🚁✨**

</div>
