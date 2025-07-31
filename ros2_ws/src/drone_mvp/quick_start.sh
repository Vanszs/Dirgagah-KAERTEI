#!/bin/bash

# KAERTEI 2025 FAIO Drone System - Quick Start Guide
# This script provides a complete overview of the system setup and usage

cat << 'EOF'

🚁 KAERTEI 2025 FAIO Drone System Quick Start
============================================

📋 SYSTEM OVERVIEW
------------------
✅ Complete ROS 2 autonomous drone system
✅ Indoor-outdoor mission capability  
✅ Computer vision with YOLOv8
✅ ArduPilot + MAVROS integration
✅ Real-time monitoring & safety systems

🛠️ HARDWARE REQUIREMENTS
------------------------
• Pixhawk 2.1 Cube Orange with ArduPilot
• 3x Cameras (front, back, top)
• 3x ToF sensors (distance measurement)
• 2x Electromagnets + relay control
• GPS + IMU for navigation
• 4S LiPo battery with monitoring

💻 SOFTWARE REQUIREMENTS
-----------------------
• Ubuntu 22.04 LTS
• ROS 2 Humble
• Python 3.8+
• MAVROS packages
• QGroundControl

🚀 QUICK INSTALLATION
--------------------
1. Run setup script:
   ./setup.sh

2. Build workspace:
   colcon build --packages-select drone_mvp
   source install/setup.bash

3. Test hardware:
   ./drone_manager.sh calibrate

4. Start mission:
   ./drone_manager.sh mission --camera front

📊 MONITORING & DEBUGGING
-------------------------
• Real-time monitor: ./drone_manager.sh monitor
• System status: ./drone_manager.sh status
• Test MAVROS: ./drone_manager.sh test-mavros
• Mission simulation: ./drone_manager.sh simulate

🛡️ SAFETY FEATURES
------------------
• Emergency stop: ./drone_manager.sh emergency
• Automatic failsafes for GPS/sensor loss
• Battery monitoring with auto-land
• Geofence protection
• Manual override capability

📖 MISSION FLOW
---------------
Indoor Phase:
1. Takeoff → 1.5m altitude
2. Search & pickup items (front/back cameras)
3. Navigate to dropzones
4. Drop items at correct baskets
5. Find exit gate (top camera)

Outdoor Phase:
1. GPS waypoint navigation
2. Outdoor item pickup
3. Return navigation
4. Precision drop
5. Auto landing

⚙️ CONFIGURATION FILES
----------------------
• config/competition_config.yaml - Mission parameters
• config/mavros_config.yaml - Flight controller settings
• config/params.yaml - Node-specific settings

🔧 TROUBLESHOOTING
-----------------
• No MAVROS connection: Check USB port & permissions
• GPS issues: Ensure clear sky view & antenna placement
• Camera problems: Test with v4l2-ctl --list-devices
• Vision detection: Verify lighting & object visibility

📞 SUPPORT
----------
For technical support or questions:
• Hardware: Check calibration results
• Software: Review ROS 2 logs with rqt_console
• Mission: Use simulation mode for testing

🏆 COMPETITION READY CHECKLIST
------------------------------
☐ Hardware calibration passed (./drone_manager.sh calibrate)
☐ MAVROS connection stable (./drone_manager.sh test-mavros)
☐ GPS fix quality good (outdoor testing)
☐ All cameras functioning (vision detection working)
☐ Mission simulation successful (./drone_manager.sh simulate)
☐ Safety systems tested (emergency procedures)
☐ Competition waypoints configured (GPS coordinates)
☐ Team communication established (monitor system)

🎯 LAUNCH COMMANDS
-----------------
# Complete system check
./drone_manager.sh calibrate

# Start competition mission
./drone_manager.sh mission --camera front

# Monitor system during flight
./drone_manager.sh monitor

# Emergency stop (if needed)
./drone_manager.sh emergency

═══════════════════════════════════════════════════════
🏁 READY FOR KAERTEI 2025 FAIO COMPETITION! 🏁
═══════════════════════════════════════════════════════

EOF

# Check if this is being run from the correct directory
if [ ! -f "package.xml" ]; then
    echo ""
    echo "⚠️  Warning: Run this script from the drone_mvp package directory"
    echo "   cd /path/to/ros2_ws/src/drone_mvp"
    echo "   ./quick_start.sh"
    echo ""
fi

echo ""
echo "📁 Available scripts in this directory:"
ls -la *.sh *.py | grep -E '^-rwx' | awk '{print "   " $9}' | sort

echo ""
echo "🎯 Next steps:"
echo "   1. ./setup.sh              - Install dependencies"
echo "   2. ./drone_manager.sh help  - See all commands"
echo "   3. ./drone_manager.sh calibrate - Test hardware"
echo ""
