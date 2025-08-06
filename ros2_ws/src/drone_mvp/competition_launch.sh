#!/bin/bash
# KAERTEI 2025 FAIO - Competition Launch Script (Ubuntu 22.04)
# ==========================================================
# Quick competition launcher optimized for Ubuntu 22.04

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Functions
log() { echo -e "${BLUE}[$(date +'%H:%M:%S')]${NC} $1"; }
success() { echo -e "${GREEN}✅ $1${NC}"; }
warning() { echo -e "${YELLOW}⚠️  $1${NC}"; }
error() { echo -e "${RED}❌ $1${NC}"; }

# Competition modes
MODE=${1:-debug}  # debug, auto, direct
INTERFACE=${2:-mavros}  # mavros, direct

echo "🏆 KAERTEI 2025 FAIO - Competition Launch"
echo "========================================"
echo "📍 Mode: $MODE"
echo "📡 Interface: $INTERFACE"
echo ""

# Pre-flight checks
log "Running pre-flight checks..."

# Check ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    log "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
    export ROS_DISTRO=humble
fi

# Check workspace
WORKSPACE_DIR="/home/vanszs/Documents/ros2/ros2_ws"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    log "Sourcing workspace..."
    source "$WORKSPACE_DIR/install/setup.bash"
else
    error "Workspace not built. Run: just build-workspace"
    exit 1
fi

# Check hardware
log "Checking hardware connectivity..."
if ls /dev/tty{USB,ACM}* 2>/dev/null; then
    success "Hardware devices found"
    ls /dev/tty{USB,ACM}* | head -3
else
    warning "No hardware devices found - using simulation mode"
fi

# Launch mission based on mode
case $MODE in
    "debug")
        log "Launching DEBUG mission (step-by-step)"
        echo "💡 Type 'next' in terminal to advance through checkpoints"
        echo ""
        
        if [ "$INTERFACE" = "mavros" ]; then
            ros2 launch drone_mvp checkpoint_mission_launch.py mode:=debug interface:=mavros
        else
            python3 checkpoint_mission_direct.py --mode debug
        fi
        ;;
        
    "auto")
        log "Launching AUTONOMOUS mission"
        warning "⚠️  COMPETITION MODE - Full autonomous execution"
        echo ""
        read -p "Ready for competition? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Mission cancelled."
            exit 0
        fi
        
        if [ "$INTERFACE" = "mavros" ]; then
            ros2 launch drone_mvp checkpoint_mission_launch.py mode:=auto interface:=mavros
        else
            python3 checkpoint_mission_direct.py --mode auto
        fi
        ;;
        
    "direct")
        log "Launching DIRECT mission (fastest startup)"
        python3 checkpoint_mission_direct.py --mode debug --no-ros
        ;;
        
    "test")
        log "Running system tests..."
        python3 validate_mission.py
        echo ""
        python3 debug_v2.py
        ;;
        
    *)
        error "Unknown mode: $MODE"
        echo "Available modes: debug, auto, direct, test"
        echo "Usage: ./competition_launch.sh [mode] [interface]"
        echo "Example: ./competition_launch.sh debug mavros"
        exit 1
        ;;
esac

success "Mission execution completed"
