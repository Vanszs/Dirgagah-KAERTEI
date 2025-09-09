#!/bin/bash

# KAERTEI 2025 FAIO - Main Launcher Script  
# 12-Checkpoint Mission System Only
# Usage: ./run_kaertei.sh [debug|auto]

echo "🚁 KAERTEI 2025 FAIO - 12 Checkpoint Mission Launcher"
echo "===================================================="

# Set script and root directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
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
if [ ! -d "$ROOT_DIR/install" ]; then
    echo "❌ System not built. Building now..."
    "$SCRIPT_DIR/build_kaertei.sh"
    if [ $? -ne 0 ]; then
        echo "❌ Build failed. Exiting."
        exit 1
    fi
    echo "✅ System built successfully"
fi

# Source workspace
source "$ROOT_DIR/install/setup.bash"

# Ensure ROS2 can resolve executables under lib/<pkg>
LIBEXEC_DIR="$ROOT_DIR/install/kaertei_drone/lib/kaertei_drone"
BIN_DIR="$ROOT_DIR/install/kaertei_drone/bin"
if [ -d "$BIN_DIR" ] && [ ! -d "$LIBEXEC_DIR" ]; then
    mkdir -p "$LIBEXEC_DIR"
    for exe in "$BIN_DIR"/*; do
        name="$(basename "$exe")"
        ln -sf "../../bin/$name" "$LIBEXEC_DIR/$name"
    done
    echo "🔗 Repaired libexec wrappers in $LIBEXEC_DIR"
fi

# Launch 12-Checkpoint Mission System
echo "🎯 Launching 12-Checkpoint Mission System..."

# Capture all logs to log.txt (overwrite each run) and filter important lines to console
LOG_FILE="$SCRIPT_DIR/log.txt"
# Show only critical/important messages
# Only minimal mission progress and prompt (everything else goes to log.txt)
# Show CP progress regardless of ROS log prefix
FILTER_REGEX='\[CP\] |Ready for next checkpoint|Type '\''next'\''' 

if [ "$MODE" == "debug" ]; then
    echo "🐛 DEBUG MODE: Step-by-step checkpoint execution"
    : > "$LOG_FILE"
    # Start launch in background, write full output to log.txt
    stdbuf -oL -eL ros2 launch kaertei_drone mission_bringup.launch.py \
        debug_mode:=true auto_continue:=false \
        start_vision:=false start_health:=false start_gpsmon:=false start_emerg:=false start_adapters:=false \
        cfg:=config/hardware_config.yaml \
        > "$LOG_FILE" 2>&1 &
    LAUNCH_PID=$!

    # Stream filtered log to console
    ( tail -F "$LOG_FILE" 2>/dev/null | grep --line-buffered -E "$FILTER_REGEX" ) &
    TAIL_PID=$!

    # Clean up on Ctrl-C
    cleanup() {
        echo "\n🧹 Stopping..."
        kill $TAIL_PID 2>/dev/null || true
        kill $LAUNCH_PID 2>/dev/null || true
        wait $LAUNCH_PID 2>/dev/null || true
        echo "🏁 Mission completed (full log: $LOG_FILE)"
        exit 0
    }
    trap cleanup INT TERM

    echo "\nCommands: next | pause | emergency | quit"
    echo "(Full log: $LOG_FILE)"
    LAST_STATUS=""
    LAST_READY_HINT=""
    while true; do
        if ! kill -0 $LAUNCH_PID 2>/dev/null; then
            cleanup
        fi
        # Build dynamic status for prompt from latest CP lines
        LAST_CP_LINE=$(grep -E "\\[CP\\] (WAIT|START|DONE|FAIL) " "$LOG_FILE" | tail -n1)
        if [ -n "$LAST_CP_LINE" ]; then
            # Extract state and checkpoint label
            STATE=$(echo "$LAST_CP_LINE" | sed -E 's/.*\[CP\] ([A-Z]+) (.*)/\1/')
            CP=$(echo "$LAST_CP_LINE" | sed -E 's/.*\[CP\] ([A-Z]+) (.*)/\2/')
            NEW_STATUS="CP ${CP} - ${STATE}"
            if [ "$NEW_STATUS" != "$LAST_STATUS" ]; then
                echo "[Status] $NEW_STATUS"
                LAST_STATUS="$NEW_STATUS"
            fi
        fi
        # Only suggest 'next' when node signals readiness
        READY_LINE=$(grep -E "Ready for next checkpoint" "$LOG_FILE" | tail -n1)
        if [ -n "$READY_LINE" ] && [ "$READY_LINE" != "$LAST_READY_HINT" ]; then
            echo "Hint: ketik 'next' untuk lanjut ($READY_LINE)"
            LAST_READY_HINT="$READY_LINE"
        fi
        read -r -p ">> " CMD
        case "${CMD,,}" in
            next|n|continue)
                ros2 topic pub -1 /mission/user_input std_msgs/String "data: 'next'" >/dev/null 2>&1 ;;
            pause)
                ros2 topic pub -1 /mission/user_input std_msgs/String "data: 'pause'" >/dev/null 2>&1 ;;
            emergency|e)
                ros2 topic pub -1 /mission/user_input std_msgs/String "data: 'emergency'" >/dev/null 2>&1 ;;
            quit|q|exit)
                cleanup ;;
            *)
                echo "(use: next | pause | emergency | quit)" ;;
        esac
    done
elif [ "$MODE" == "auto" ]; then
    echo "🤖 AUTONOMOUS MODE: Full mission execution"
    stdbuf -oL -eL ros2 launch kaertei_drone mission_bringup.launch.py \
        debug_mode:=false auto_continue:=true \
        start_vision:=true start_health:=true start_gpsmon:=true start_emerg:=true start_adapters:=true \
        cfg:=config/hardware_config.yaml \
        2>&1 | tee "$LOG_FILE" | grep -E "$FILTER_REGEX"
else
    echo "❌ Invalid mode: $MODE"
    echo "Usage: $0 [debug|auto]"
    echo ""
    echo "Examples:"
    echo "  $0 debug  # Manual step-by-step debugging"
    echo "  $0 auto   # Full autonomous mission"
    exit 1
fi

echo "🏁 Mission completed (full log: $LOG_FILE)"
