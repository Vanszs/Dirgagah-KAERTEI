# KAERTEI 2025 FAIO - Autonomous Drone System

## Overview
Sistem drone otonom untuk kompetisi "KAERTEI 2025 Divisi Fully Autonomous Indoor-Outdoor (FAIO)" dengan sistem debugging 26 checkpoint.

## Installation Guide

### For Ubuntu/Debian Users (Recommended)
```bash
# Clone repository
git clone <your-repo-url>
cd ros2_ws/src/drone_mvp

# Run simple installer (binary packages)
chmod +x install_mavros_simple.sh
./install_mavros_simple.sh

# Build workspace
cd ../../../
colcon build --packages-select drone_mvp
source install/setup.bash
```

### For Arch Linux Users (Advanced)
MAVROS binary packages are not available for ROS 2 Humble on Arch Linux. You have several options:

#### Option 1: Docker (Recommended)
```bash
# Use Ubuntu-based Docker container
docker run -it --net=host osrf/ros:humble-desktop bash
# Then follow Ubuntu installation steps inside container
```

#### Option 2: AUR Packages (May be outdated)
```bash
yay -S ros-humble-mavros
# Note: May not be compatible with latest ROS 2 Humble
```

#### Option 3: Build from Source (Complex)
```bash
# Requires extensive dependency management
# See original install_mavros.sh for detailed steps
# Not recommended due to Python environment conflicts
```

#### Option 4: Switch to Ubuntu 22.04 (Recommended for competition)
For the KAERTEI 2025 competition, Ubuntu 22.04 LTS provides the most stable and well-supported environment.

## System Architecture

### Communication Modes
1. **MAVROS Mode** (Ubuntu/Debian): Uses ROS 2 ↔ MAVLink bridge
2. **Direct MAVLink Mode**: Direct communication with flight controller

### Checkpoint System
- 26 predefined checkpoints for debugging
- State machine with comprehensive error handling
- Real-time position tracking and waypoint navigation

### Hardware Configuration
Supports multiple hardware configurations:
- Pixhawk flight controllers
- Various camera systems
- GPS modules
- Companion computers (Raspberry Pi, Jetson)

## Quick Start

### Testing Installation
```bash
# Test ROS 2 installation
ros2 node list

# Test MAVROS (Ubuntu/Debian only)
ros2 launch drone_mvp test_mavros.launch.py

# Check topics
ros2 topic list | grep mavros
```

### Running the Drone System
```bash
# Launch full system
ros2 launch drone_mvp drone_system.launch.py

# Enable debug mode
ros2 launch drone_mvp drone_system.launch.py debug:=true
```

## Development Notes

### Distribution Compatibility
- **Ubuntu 22.04 LTS**: Full support, recommended
- **Debian 11+**: Full support
- **Arch Linux**: Limited support, MAVROS requires manual setup

### Python Environment
- Uses virtual environment for Python dependencies
- Automatic activation in fish/bash shells
- Isolated from system Python to prevent conflicts

### Competition Preparation
For KAERTEI 2025 FAIO competition:
1. Use Ubuntu 22.04 LTS for maximum compatibility
2. Test all 26 checkpoints in simulation first
3. Validate hardware integration
4. Practice emergency procedures

## Troubleshooting

### Common Issues on Arch Linux
1. **MAVROS not available**: Use Docker or Ubuntu VM
2. **Python conflicts**: Use provided virtual environment
3. **Build failures**: Check dependencies in install script

### Getting Help
1. Check ROS 2 documentation: https://docs.ros.org/en/humble/
2. MAVROS documentation: https://github.com/mavlink/mavros
3. Competition guidelines: KAERTEI 2025 official documentation

## Contributing
This system is prepared for the KAERTEI 2025 competition. Improvements and bug fixes are welcome through pull requests.

## License
Developed for KAERTEI 2025 Divisi Fully Autonomous Indoor-Outdoor (FAIO) competition.
