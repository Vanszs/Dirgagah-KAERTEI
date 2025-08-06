#!/bin/bash

# KAERTEI 2025 FAIO - Main Launcher Script
# Clean and organized structure
# Usage: ./run_kaertei.sh [debug|auto] [checkpoint|simple]

echo "🚁 KAERTEI 2025 FAIO - Main Launcher"
echo "===================================="

# Set script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
MODE="${1:-debug}"        # debug or auto
SYSTEM="${2:-checkpoint}" # checkpoint or simple

echo "🔧 Configuration:"
echo "   Mode: $MODE"
echo "   System: $SYSTEM"
echo "   Directory: $SCRIPT_DIR"

# Source ROS2 environment
echo "🔄 Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

# Check if system is built
if [ ! -d "install" ]; then
    echo "❌ System not built. Building now..."
    ./build_kaertei.sh
    if [ $? -ne 0 ]; then
        echo "❌ Build failed. Exiting."
        exit 1
    fi
    echo "✅ System built successfully"
fi

# Source workspace
source install/setup.bash

# Launch appropriate system
if [ "$SYSTEM" == "checkpoint" ]; then
    echo "🎯 Launching 12 Checkpoint Mission System..."
    if [ "$MODE" == "debug" ]; then
        ros2 launch kaertei_drone kaertei_pi5_system.launch.py debug_mode:=true
    else
        ros2 launch kaertei_drone kaertei_pi5_system.launch.py debug_mode:=false
    fi
else
    echo "🎯 Launching Simple 3-Waypoint System..."
    if [ "$MODE" == "debug" ]; then
        ros2 launch kaertei_drone simple_3waypoint_system.launch.py debug_mode:=true
    else
        ros2 launch kaertei_drone simple_3waypoint_system.launch.py debug_mode:=false
    fi
fi

echo "🏁 Mission completed"
