# 🏆 KAERTEI 2025 FAIO Competition Guide

## 🎯 **Competition Overview**
- **Event**: KAERTEI 2025 Divisi Fully Autonomous Indoor-Outdoor (FAIO)
- **Mission Type**: Autonomous navigation with checkpoint validation
- **System**: 26 debugging checkpoints with manual progression control
- **Mode**: Debug mode recommended for competition safety

---

## 🚀 **Pre-Competition Checklist**

### **📋 Hardware Preparation**
- [ ] **Flight Controller**: Pixhawk 2.1 Cube Orange configured and tested
- [ ] **Propulsion**: All motors and ESCs calibrated
- [ ] **Power System**: Battery fully charged, voltage monitoring active
- [ ] **Communication**: Telemetry radio, USB connections verified
- [ ] **Sensors**: GPS lock achieved, IMU calibrated, compass calibrated
- [ ] **Vision System**: All cameras focused, exposure settings optimized
- [ ] **Actuators**: Electromagnets and relay systems tested
- [ ] **Emergency**: RC transmitter paired and failsafe configured

### **💻 Software Preparation**
- [ ] **Operating System**: Tested on competition hardware (Ubuntu/Arch)
- [ ] **ROS 2 Installation**: All dependencies installed and verified
- [ ] **MAVROS Bridge**: Connection to flight controller confirmed
- [ ] **Workspace Build**: `colcon build` successful without errors
- [ ] **Configuration**: Hardware config updated for competition venue
- [ ] **Testing**: All 26 checkpoints tested in debug mode
- [ ] **Backup**: System image created, configuration files backed up

### **🗺️ Venue-Specific Configuration**
```bash
# Update GPS waypoints for competition venue
nano config/hardware_config.conf

[GPS_WAYPOINTS]
# UPDATE THESE COORDINATES FOR ACTUAL VENUE!
waypoint_1_lat = -6.365000  # Replace with actual coordinates
waypoint_1_lon = 106.825000 # Replace with actual coordinates
waypoint_2_lat = -6.365050
waypoint_2_lon = 106.825050
# ... continue for all waypoints
```

---

## 🎮 **Competition Day Procedures**

### **🌅 Setup Phase (30 minutes before)**
```bash
# 1. Power on all systems
./install_kaertei.sh
# Select option 7 (View System Status)

# 2. Verify hardware connections
./install_kaertei.sh  
# Select option 8 (Hardware Detection)

# 3. GPS lock verification (outdoor venue)
# Wait for GPS to acquire satellites (2-5 minutes)
# Verify position accuracy in QGroundControl

# 4. Camera system check
# Test all camera feeds
# Verify object detection is working
# Check lighting conditions and adjust exposure

# 5. Final system test
./run_checkpoint_mission.sh debug mavros
# Run through first 5 checkpoints to verify operation
```

### **🏁 Competition Launch Sequence**
```bash
# Competition launch command (DEBUG MODE)
cd ~/ros2_ws/src/drone_mvp/
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
./run_checkpoint_mission.sh debug mavros

# System will start and wait at each checkpoint
# Press 'n' + Enter to proceed to next checkpoint
# This gives you full control during competition
```

### **⚡ Emergency Procedures**
```bash
# Emergency stop (keep terminal ready)
Ctrl+C  # Kills the mission immediately

# Emergency land command
ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL

# Switch to manual control
# Use RC transmitter failsafe switch
# Or use QGroundControl manual mode
```

---

## 🔍 **Checkpoint Strategy**

### **26 Debugging Checkpoints**
1. **System Initialization** - Verify all systems online
2. **Hardware Check** - Confirm sensors and actuators
3. **GPS Lock** - Wait for positioning accuracy
4. **Camera Calibration** - Verify vision systems
5. **Takeoff Preparation** - Pre-flight safety checks
6. **Initial Takeoff** - Controlled ascent to mission altitude
7. **Hover Test** - Stability verification
8. **Vision Activation** - Computer vision system online
9. **First Waypoint** - Navigate to starting position
10. **Object Detection** - Identify first target
11. **Approach Phase** - Navigate towards object
12. **Precision Positioning** - Fine position adjustment
13. **Pickup Preparation** - Activate electromagnet system
14. **Object Pickup** - Execute pickup sequence
15. **Pickup Verification** - Confirm object attached
16. **Transit Phase** - Navigate to drop zone
17. **Drop Zone Approach** - Precision navigation
18. **Drop Preparation** - Position for release
19. **Object Release** - Deactivate electromagnet
20. **Release Verification** - Confirm object dropped
21. **Next Object** - Navigate to second target
22. **Repeat Sequence** - Checkpoints 10-20 for additional objects
23. **Mission Complete** - All objects processed
24. **Return Navigation** - Navigate to landing zone
25. **Landing Approach** - Prepare for landing
26. **Safe Landing** - Complete mission sequence

### **Manual Progression Strategy**
- **Never rush** - Take time to verify each checkpoint
- **Visual confirmation** - Always visually verify before proceeding
- **Safety first** - If anything looks wrong, stop and assess
- **Communication** - Keep judges informed of your progress
- **Backup plans** - Know your emergency procedures

---

## 🛡️ **Safety Protocols**

### **Pre-Flight Safety**
- [ ] Propeller guards installed and secure
- [ ] Battery voltage above minimum threshold
- [ ] All personnel clear of flight area
- [ ] Emergency stop procedures reviewed
- [ ] RC transmitter ready for manual override
- [ ] Landing area clear and identified

### **During Flight Safety**
- [ ] Maintain visual line of sight
- [ ] Monitor battery levels continuously
- [ ] Watch for obstacle conflicts
- [ ] Be ready for manual takeover
- [ ] Monitor system status messages
- [ ] Keep emergency stop ready (Ctrl+C)

### **Emergency Responses**
- **Low Battery**: Immediate return to land
- **GPS Loss**: Switch to manual control
- **Communication Loss**: Activate RC failsafe
- **System Error**: Emergency land sequence
- **Obstacle Detected**: Manual override and assess

---

## 📊 **Performance Optimization**

### **System Performance**
```bash
# Before competition, optimize system performance
sudo systemctl stop unnecessary-services
sudo swapoff -a  # If enough RAM available
sudo sysctl vm.swappiness=10  # Reduce swap usage

# Close unnecessary applications
pkill -f firefox
pkill -f chrome
pkill -f thunderbird
# Keep only essential: terminal, QGroundControl, your mission
```

### **Network Optimization**
```bash
# Optimize ROS 2 network performance
export ROS_DOMAIN_ID=42  # Use unique domain ID
export RCUTILS_LOGGING_SEVERITY=WARN  # Reduce logging

# Disable unnecessary ROS 2 nodes
# Keep only essential nodes running
```

### **Camera Optimization**
```bash
# Set optimal camera parameters for venue lighting
v4l2-ctl --set-ctrl=brightness=128
v4l2-ctl --set-ctrl=contrast=128
v4l2-ctl --set-ctrl=exposure_auto=1
# Adjust based on actual lighting conditions
```

---

## 🎯 **Competition Scoring Strategy**

### **Time Management**
- **Budget 20 minutes** for complete mission
- **Allocate 2 minutes** for each major phase
- **Reserve 5 minutes** for unexpected issues
- **Don't rush** - accuracy over speed

### **Accuracy Priorities**
1. **Takeoff/Landing Safety** - Maximum points preservation
2. **Object Identification** - Computer vision accuracy
3. **Pickup Precision** - Electromagnet positioning
4. **Drop Accuracy** - Target zone precision
5. **Navigation Efficiency** - Waypoint accuracy

### **Risk Mitigation**
- **Test indoor mode first** if venue has indoor/outdoor phases
- **Verify altitude limits** for venue restrictions
- **Practice object pickup** with similar objects beforehand
- **Plan for lighting changes** between indoor/outdoor
- **Have manual override ready** at all times

---

## 📞 **Competition Day Support**

### **Quick Status Commands**
```bash
# System health check
ros2 topic hz /mavros/state  # Should show ~20Hz
ros2 topic echo /mavros/battery  # Battery monitoring
ros2 topic list | grep mavros  # Active MAVROS topics

# Hardware verification
ls /dev/tty*  # Serial connections
lsusb | grep -i px4  # Flight controller USB
v4l2-ctl --list-devices  # Camera devices
```

### **Troubleshooting During Competition**
```bash
# If MAVROS connection fails
sudo chmod 666 /dev/ttyUSB0
ros2 launch drone_mvp checkpoint_mission_mavros_launch.py

# If that fails, switch to direct MAVLink
./run_checkpoint_mission.sh debug mavlink

# Nuclear option: restart everything
pkill -f ros2; sleep 2; ./run_checkpoint_mission.sh debug mavros
```

### **Judge Communication**
- **Explain each checkpoint** as you progress
- **Show debug terminal** to demonstrate control
- **Announce intentions** before each action
- **Request clarification** if rules are unclear
- **Stay calm** and professional throughout

---

## 🏆 **Winning Strategy**

### **Technical Excellence**
- **Robust system** - Multi-mode fallbacks (MAVROS/MAVLink)
- **Debug mode** - Full manual control and visibility
- **Multi-distro** - Works on any Linux distribution
- **Comprehensive testing** - 26 checkpoints ensure thorough validation

### **Operational Excellence**
- **Safety first** - Never compromise on safety protocols
- **Clear communication** - Keep judges informed
- **Methodical approach** - Don't skip checkpoints
- **Stay calm** - Handle unexpected situations professionally

### **Competitive Advantages**
- **26-checkpoint validation** - Most thorough testing system
- **Multi-mode fallbacks** - MAVROS + MAVLink redundancy
- **Universal compatibility** - Works on Ubuntu, Arch, Debian, Manjaro
- **Professional tooling** - Industry-standard ROS 2 + MAVROS
- **Emergency preparedness** - Comprehensive troubleshooting guides

---

## 🎊 **Post-Competition**

### **Data Collection**
```bash
# Save competition logs
mkdir -p ~/competition_logs/kaertei_2025_$(date +%Y%m%d)
cd ~/competition_logs/kaertei_2025_$(date +%Y%m%d)

# Copy relevant files
cp ~/ros2_ws/src/drone_mvp/config/hardware_config.conf .
ros2 bag record -a -o competition_data &  # If available
# Stop with Ctrl+C after mission

# System status
ros2 topic list > topics_used.log
ros2 node list > nodes_active.log
```

### **Analysis and Improvement**
- Review checkpoint progression logs
- Analyze any issues encountered
- Document lessons learned
- Plan improvements for future competitions
- Share success story with community!

---

## 🎯 **Final Reminders**

1. **Practice, practice, practice** - Run through all 26 checkpoints multiple times
2. **Know your emergency procedures** - Practice manual override
3. **Verify GPS coordinates** - Update for actual competition venue
4. **Bring backup equipment** - Extra USB cables, SD cards, etc.
5. **Stay calm and professional** - You've got this! 

**🏆 Good luck at KAERTEI 2025 FAIO! Your system is ready to compete! 🚁**
