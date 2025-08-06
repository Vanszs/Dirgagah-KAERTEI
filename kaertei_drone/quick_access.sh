#!/bin/bash

# KAERTEI 2025 - Quick Development Access Script
# Navigate quickly to different parts of the codebase

echo "🚁 KAERTEI 2025 - Quick Access Menu"
echo "================================="
echo "Choose your destination:"
echo
echo "1) 🎯 Mission Control (checkpoint_mission_mavros.py)"
echo "2) 🔧 Hardware Config (hardware_config.conf)" 
echo "3) 🚀 Launch Files"
echo "4) 📊 Monitor System Status"
echo "5) 👁️ Vision System"
echo "6) 🧭 Navigation"
echo "7) 📜 Scripts"
echo "8) 📚 Documentation"
echo "9) 🔍 Find File by Name"
echo "0) Exit"
echo

read -p "Enter choice [0-9]: " choice

KAERTEI_DIR="/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone"

case $choice in
    1)
        echo "🎯 Opening Mission Control..."
        cd "$KAERTEI_DIR/src/mission"
        ls -la *.py
        echo "Files available:"
        echo "  - checkpoint_mission_mavros.py (12 checkpoint system)"
        echo "  - simplified_mission_control.py (3 waypoint system)"
        ;;
    2)
        echo "🔧 Opening Hardware Configuration..."
        cd "$KAERTEI_DIR/config"
        ls -la hardware_config.conf
        echo "Key settings in hardware_config.conf:"
        echo "  - Camera indices, GPIO pins"
        echo "  - Flight parameters, GPS waypoints"
        echo "  - Vision detection thresholds"
        ;;
    3)
        echo "🚀 Launch Files..."
        cd "$KAERTEI_DIR/launch"
        ls -la *.py
        echo "Available launch files:"
        echo "  - kaertei_pi5_system.launch.py (Main system)"
        echo "  - simple_3waypoint_system.launch.py (Simple system)"
        ;;
    4)
        echo "📊 System Status..."
        cd "$KAERTEI_DIR/src/monitoring"
        ls -la *.py
        echo "Monitoring components:"
        echo "  - system_health_monitor.py"
        echo "  - sensor_monitor.py"
        ;;
    5)
        echo "👁️ Vision System..."
        cd "$KAERTEI_DIR/src/vision"
        ls -la *.py
        echo "Vision components:"
        echo "  - unified_vision_system.py (YOLO detection)"
        echo "  - Place ONNX models in: $KAERTEI_DIR/models/"
        ;;
    6)
        echo "🧭 Navigation System..."
        cd "$KAERTEI_DIR/src/navigation"
        ls -la *.py
        echo "Navigation components:"
        echo "  - px4_waypoint_navigator.py (GPS waypoints)"
        echo "  - flight_state_monitor.py (MAVROS bridge)"
        ;;
    7)
        echo "📜 Utility Scripts..."
        cd "$KAERTEI_DIR/scripts"
        ls -la *.sh
        echo "Available scripts:"
        echo "  - run_simplified_mission.sh"
        echo "  - test_hardware_pi5.sh"
        echo "  - competition_startup.sh"
        ;;
    8)
        echo "📚 Documentation..."
        cd "$KAERTEI_DIR/docs"
        ls -la *.md
        echo "Documentation available:"
        echo "  - ONNX_MODEL_PLACEMENT.md (Model setup guide)"
        echo "  - TROUBLESHOOTING.md (Problem solving)"
        ;;
    9)
        read -p "🔍 Enter filename to search: " filename
        echo "Searching for: $filename"
        find "$KAERTEI_DIR" -name "*$filename*" -type f
        ;;
    0)
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice"
        ;;
esac

echo
echo "📁 Current directory: $(pwd)"
echo "🔙 Run this script again: cd $KAERTEI_DIR && ./quick_access.sh"
