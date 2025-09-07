#!/bin/bash

# KAERTEI 2025 FAIO - Main Launcher Script  
# 12-Checkpoint Mission System Only
# Usage: ./run_kaertei.sh [debug|auto]

echo "🚁 KAERTEI 2025 FAIO - 12 Checkpoint Mission Launcher"
echo "===================================================="

# Set script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
MODE="${1:-debug}"        # debug or auto

echo "🔧 Configuration:"
echo "   Mode: $MODE"
echo "   Mission: 12-Checkpoint System"
echo "   Directory: $SCRIPT_DIR"

# Source ROS2 environment
echo "🔄 Setting up ROS2 environment (Foxy)..."
source /opt/ros/foxy/setup.bash

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

# Launch 12-Checkpoint Mission System
echo "🎯 Launching 12-Checkpoint Mission System..."
if [ "$MODE" == "debug" ]; then
    echo "🐛 DEBUG MODE: Step-by-step checkpoint execution"
    ros2 launch kaertei_drone kaertei_12checkpoint_system.launch.py debug_mode:=true auto_continue:=false
elif [ "$MODE" == "auto" ]; then
    echo "🤖 AUTONOMOUS MODE: Full mission execution"
    ros2 launch kaertei_drone kaertei_12checkpoint_system.launch.py debug_mode:=false auto_continue:=true
else
    echo "❌ Invalid mode: $MODE"
    echo "Usage: $0 [debug|auto]"
    echo ""
    echo "Examples:"
    echo "  $0 debug  # Manual step-by-step debugging"
    echo "  $0 auto   # Full autonomous mission"
    exit 1
fi

echo "🏁 Mission completed"
