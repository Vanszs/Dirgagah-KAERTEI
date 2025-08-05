#!/bin/bash

# KAERTEI 2025 FAIO - Multi-Mode Checkpoint Mission Launcher  
# Usage: ./run_checkpoint_mission.sh [debug|auto] [mavros|mavlink]

echo "🚁 KAERTEI 2025 FAIO - Checkpoint Mission System (Multi-Mode)"
echo "=============================================================="

# Default modes
EXECUTION_MODE="debug"
COMM_MODE="mavros"

# Parse command line arguments
if [ "$1" == "auto" ]; then
    EXECUTION_MODE="auto"
elif [ "$1" == "debug" ] || [ -z "$1" ]; then
    EXECUTION_MODE="debug"
elif [ "$1" != "" ]; then
    echo "❌ Invalid execution mode: $1"
    echo "Usage: $0 [debug|auto] [mavros|mavlink]"
    exit 1
fi

if [ "$2" == "mavlink" ]; then
    COMM_MODE="mavlink"
elif [ "$2" == "mavros" ] || [ -z "$2" ]; then
    COMM_MODE="mavros"
elif [ "$2" != "" ]; then
    echo "❌ Invalid communication mode: $2"
    echo "Usage: $0 [debug|auto] [mavros|mavlink]"
    exit 1
fi

# Display selected modes
echo "⚙️ Configuration:"
echo "   Execution Mode: $EXECUTION_MODE"
echo "   Communication: $COMM_MODE"

if [ "$EXECUTION_MODE" == "auto" ]; then
    echo "🤖 AUTONOMOUS mode - Mission will execute automatically"
else
    echo "🐛 DEBUG mode - Manual 'next' input required for each checkpoint"
fi

if [ "$COMM_MODE" == "mavros" ]; then
    echo "📡 Using MAVROS bridge for PX4 communication"
else
    echo "🔗 Using direct MAVLink for PX4 communication"
fi

echo ""

# Setup environment
setup_environment

echo "=============================================================="

# Auto-detect shell and setup environment
setup_environment() {
    local current_shell=$(basename "$SHELL")
    local venv_path="./ros2_venv"
    
    echo "🐚 Detected shell: $current_shell"
    
    # Check if virtual environment exists and activate if needed
    if [ -d "$venv_path" ]; then
        echo "🐍 Virtual environment found at $venv_path"
        case "$current_shell" in
            fish)
                if [ -f "$venv_path/bin/activate.fish" ]; then
                    echo "💡 To activate virtual environment in fish shell, run:"
                    echo "   source ros2_venv/bin/activate.fish"
                fi
                ;;
            bash|sh|zsh)
                if [ -f "$venv_path/bin/activate" ]; then
                    echo "💡 To activate virtual environment in $current_shell shell, run:"
                    echo "   source ros2_venv/bin/activate"
                fi
                ;;
        esac
        echo ""
    fi
}

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced"
else
    echo "❌ Workspace not built. Run 'colcon build' first"
    exit 1
fi

# Set parameters based on modes
if [ "$EXECUTION_MODE" == "debug" ]; then
    DEBUG_PARAM="debug_mode:=true"
else
    DEBUG_PARAM="debug_mode:=false"
fi

if [ "$COMM_MODE" == "mavros" ]; then
    USE_MAVROS_PARAM="use_mavros:=true"
else
    USE_MAVROS_PARAM="use_mavros:=false"
fi

echo "🚀 Launching checkpoint mission system..."
echo "   Execution Mode: $EXECUTION_MODE"
echo "   Communication: $COMM_MODE"
echo "   Parameters: $DEBUG_PARAM $USE_MAVROS_PARAM"
echo ""

# Check if MAVROS is required and available
if [ "$COMM_MODE" == "mavros" ]; then
    echo "🔍 Checking MAVROS availability..."
    if ! ros2 pkg list | grep -q mavros; then
        echo "❌ MAVROS not found! Please install MAVROS first:"
        echo "   ./install_mavros.sh"
        echo ""
        echo "🔄 Falling back to direct MAVLink mode..."
        USE_MAVROS_PARAM="use_mavros:=false"
        COMM_MODE="mavlink"
    else
        echo "✅ MAVROS found"
        
        # Check if MAVROS is running
        if ! ros2 topic list | grep -q "/mavros/state"; then
            echo "⚠️  MAVROS not running. Starting MAVROS..."
            echo "💡 In another terminal, run:"
            echo "   ros2 launch mavros px4.launch fcu_url:='/dev/ttyUSB0:57600'"
            echo ""
            echo "Press Enter when MAVROS is running..."
            read -r
        else
            echo "✅ MAVROS is running"
        fi
    fi
fi

# Launch the mission
ros2 launch drone_mvp checkpoint_mission.launch.py $DEBUG_PARAM $USE_MAVROS_PARAM

echo ""
echo "🏁 Mission system terminated"
