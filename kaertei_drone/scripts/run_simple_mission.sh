#!/bin/bash

# Simple Mission Runner - KAERTEI 2025 FAIO
# ARM -> Takeoff 1m -> Move forward until obstacle -> Land

echo "🎯 KAERTEI 2025 Simple Mission (No Dead Reckoning)"
echo "================================================"
echo "Mission Plan:"
echo "  1. ARM drone"
echo "  2. Takeoff to 1.0m altitude"
echo "  3. Move forward until 10cm from wall"
echo "  4. Land slowly"
echo ""

# Default mode
EXECUTION_MODE="debug"

# Parse command line arguments
if [ "$1" == "auto" ]; then
    EXECUTION_MODE="auto"
    echo "🤖 AUTONOMOUS mode - Mission will execute automatically"
elif [ "$1" == "debug" ] || [ -z "$1" ]; then
    EXECUTION_MODE="debug"
    echo "🐛 DEBUG mode - Manual 'next' input required for each step"
else
    echo "❌ Invalid execution mode: $1"
    echo "Usage: $0 [debug|auto]"
    exit 1
fi

echo "⚙️ Configuration:"
echo "   Execution Mode: $EXECUTION_MODE"
echo "   Dead Reckoning: DISABLED"
echo "   Navigation: Simple obstacle-based"
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace (check multiple possible locations)
if [ -f "../../install/setup.bash" ]; then
    source ../../install/setup.bash
    echo "✅ Workspace sourced from ../../install/setup.bash"
elif [ -f "../../../install/setup.bash" ]; then
    source ../../../install/setup.bash  
    echo "✅ Workspace sourced from ../../../install/setup.bash"
elif [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced from install/setup.bash"
else
    echo "❌ Workspace not built. Run 'colcon build' first"
    echo "🔍 Checked paths:"
    echo "   - ../../install/setup.bash"
    echo "   - ../../../install/setup.bash" 
    echo "   - install/setup.bash"
    exit 1
fi

echo "🚀 Launching simple mission system..."
echo "   Execution Mode: $EXECUTION_MODE"
echo "   Navigation: Obstacle-based (no dead reckoning)"

# Set debug parameter based on execution mode
if [ "$EXECUTION_MODE" == "debug" ]; then
    DEBUG_PARAM="debug_mode:=true"
else
    DEBUG_PARAM="debug_mode:=false"
fi

echo "   Parameters: $DEBUG_PARAM"
echo ""

# Launch simple mission node
echo "🎬 Starting simple mission node..."
ros2 run drone_mvp simple_mission_node --ros-args -p $DEBUG_PARAM
