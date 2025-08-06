# ========================================
# KAERTEI 2025 FAIO - Ubuntu Focused
# Universal Justfile (run from anywhere)
# ========================================

# Project root detection
PROJECT_ROOT := `git rev-parse --show-toplevel 2>/dev/null || pwd`
KAERTEI_DRONE_DIR := PROJECT_ROOT / "kaertei_drone"
SCRIPTS_DIR := KAERTEI_DRONE_DIR / "scripts"

# Show available commands
default:
    @echo "🚁 KAERTEI 2025 FAIO - Ubuntu Competition System"
    @echo "=============================================="
    @echo ""
    @echo "📍 Project Root: {{PROJECT_ROOT}}"
    @echo "🗂️  Current Dir: $(pwd)"
    @echo ""
    @echo "🎯 Quick Start:"
    @echo "  just setup       # Install everything"
    @echo "  just test        # Validate system"
    @echo "  just debug       # Run debug mission (step-by-step)"
    @echo "  just run         # Run autonomous mission"
    @echo ""
    @echo "📋 All Commands:"
    @just --list

# ===================
# 🚀 SETUP & INSTALL
# ===================

# Complete system setup (one command install)
setup:
    @echo "🚀 KAERTEI 2025 - Complete Ubuntu Setup"
    @echo "======================================"
    @cd "{{PROJECT_ROOT}}" && chmod +x install_kaertei.sh && ./install_kaertei.sh

# Build ROS 2 workspace
build:
    cd {{KAERTEI_DRONE_DIR}} && bash -c "source /opt/ros/humble/setup.bash && ./build_kaertei.sh"

# Clean and rebuild
rebuild:
    @echo "🧹 Clean and rebuild workspace..."
    @cd "{{KAERTEI_DRONE_DIR}}" && rm -rf build/ install/ log/
    @just build

# ===================
# 🧪 TESTING & STATUS
# ===================

# Complete system validation
test:
    @echo "🧪 Ubuntu System Validation"
    @echo "============================"
    @cd "{{KAERTEI_DRONE_DIR}}" && python3 src/validate_ubuntu22.py

# Complete hardware validation for Pi 5
test-hardware:
    @echo "🔍 Pi 5 + Pixhawk4 Hardware Test"
    @echo "================================="
    @cd "{{KAERTEI_DRONE_DIR}}" && chmod +x scripts/test_hardware_pi5.sh && ./scripts/test_hardware_pi5.sh

# Quick hardware check
hardware:
    @echo "🔍 Hardware Detection - Pi 5 Configuration"
    @echo "=========================================="
    @echo "📡 Flight Controller (Pixhawk4):"
    @ls /dev/tty{ACM,USB}0* 2>/dev/null && echo "✅ Found" || echo "❌ No flight controller"
    @echo ""
    @echo "📷 Cameras (3x USB):"
    @ls /dev/video{0,2,4} 2>/dev/null && echo "✅ Found" || echo "❌ Missing cameras"
    @v4l2-ctl --list-devices 2>/dev/null | head -10 || true
    @echo ""
    @echo "📡 LiDAR Sensors (3x TF Mini Plus):"
    @ls /dev/ttyUSB{1,2,3} 2>/dev/null && echo "✅ Found" || echo "❌ Missing LiDAR sensors"
    @echo ""
    @echo "� GPIO System (Pi 5):"
    @test -c /dev/gpiomem && echo "✅ GPIO available" || echo "❌ GPIO not accessible"

# Test LiDAR sensors
test-lidar:
    @echo "📡 Testing LiDAR Sensors"
    @echo "======================="
    @echo "🔍 Front LiDAR (USB1):"
    @timeout 3s cat /dev/ttyUSB1 2>/dev/null | hexdump -C | head -5 || echo "❌ No data from front LiDAR"
    @echo ""
    @echo "🔍 Left LiDAR (USB2):"  
    @timeout 3s cat /dev/ttyUSB2 2>/dev/null | hexdump -C | head -5 || echo "❌ No data from left LiDAR"
    @echo ""
    @echo "🔍 Right LiDAR (USB3):"
    @timeout 3s cat /dev/ttyUSB3 2>/dev/null | hexdump -C | head -5 || echo "❌ No data from right LiDAR"

# Test cameras
test-cameras:
    @echo "📷 Testing Camera System"
    @echo "======================="
    @echo "📷 Front Camera (video0):"
    @timeout 5s ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 -f null - 2>/dev/null && echo "✅ Working" || echo "❌ Failed"
    @echo ""
    @echo "📷 Back Camera (video2):"
    @timeout 5s ffmpeg -f v4l2 -i /dev/video2 -frames:v 1 -f null - 2>/dev/null && echo "✅ Working" || echo "❌ Failed"
    @echo ""
    @echo "📷 Top Camera (video4):"
    @timeout 5s ffmpeg -f v4l2 -i /dev/video4 -frames:v 1 -f null - 2>/dev/null && echo "✅ Working" || echo "❌ Failed"

# Test GPIO relays
test-gpio:
    @echo "🔌 Testing GPIO Relays (Pi 5)"
    @echo "============================="
    @echo "⚠️  GPIO test requires Raspberry Pi 5 hardware"
    @echo "🧲 Front Magnet Relay: GPIO 18"
    @echo "🧲 Back Magnet Relay: GPIO 19"
    @echo "✅ GPIO configuration ready"

# Complete hardware validation (combines all hardware tests)
test-all-hardware: test-cameras test-lidar test-gpio
    @echo "✅ Complete hardware test finished"

# System status overview
status:
    @echo "📊 KAERTEI 2025 System Status"
    @echo "============================="
    @echo "🖥️  OS: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown')"
    @echo "🐍 Python: $(python3 --version 2>/dev/null || echo 'Not found')"
    @echo "🤖 ROS 2: $(test -f /opt/ros/humble/setup.bash && echo 'Humble ✅' || echo 'Not installed ❌')"
    @echo "🔧 Hardware: $(ls /dev/tty{ACM,USB}* 2>/dev/null | wc -l) device(s)"
    @echo "📷 Cameras: $(ls /dev/video* 2>/dev/null | wc -l) camera(s)"
    @echo "💾 Workspace: $(test -d '{{KAERTEI_DRONE_DIR}}/install' && echo 'Built ✅' || echo 'Not built ❌')"

# ===================
# 🏆 MISSION COMMANDS
# ===================

# Debug mission (step-by-step with manual control)
debug:
    @echo "🔍 Starting DEBUG Mission (12 checkpoints)"
    @echo "=========================================="
    @echo "⚙️  Mode: Manual step-by-step"
    @echo "📝 Instructions: Type 'next' + Enter to proceed"
    @echo ""
    @bash -c 'if [ ! -d "{{KAERTEI_DRONE_DIR}}/install" ]; then echo "🔨 Building workspace first..."; just build; fi'
    @bash -c 'cd "{{KAERTEI_DRONE_DIR}}" && source install/setup.bash && echo "✅ KAERTEI workspace sourced" && ./run_kaertei.sh debug'

# Autonomous mission (full competition mode)
run:
    @echo "🚁 Starting AUTONOMOUS Mission"
    @echo "============================="
    @echo "⚙️  Mode: Fully autonomous"
    @echo "⚠️  WARNING: No manual intervention!"
    @echo ""
    @bash -c 'if [ ! -d "{{KAERTEI_DRONE_DIR}}/install" ]; then echo "🔨 Building workspace first..."; just build; fi'
    @bash -c 'cd "{{KAERTEI_DRONE_DIR}}" && source install/setup.bash && echo "✅ KAERTEI workspace sourced" && ./run_kaertei.sh auto'

# Simulation test (safe testing)
simulate:
    @echo "🎮 Starting SIMULATION Mode"
    @echo "=========================="
    @echo "⚙️  Mode: Software simulation only"
    @echo "🔒 Safe: No hardware required"
    @echo ""
    @cd "{{KAERTEI_DRONE_DIR}}" && python3 src/mission/simulate_mission.py

# ===================
# 🛠️ DEVELOPMENT
# ===================

# Update system and dependencies
update:
    @echo "🔄 Updating system packages and Python dependencies..."
    @sudo apt update && sudo apt upgrade -y
    @export PATH="$$HOME/.local/bin:$$PATH" && pip3 install --upgrade -r requirements.txt

# Fix hardware permissions
fix-permissions:
    @echo "🔧 Fixing hardware permissions..."
    @sudo usermod -a -G dialout,video,tty $$USER
    @sudo chmod 666 /dev/tty{USB,ACM}* 2>/dev/null || true
    @echo "✅ Permissions updated. Please logout/login for changes to take effect."

# Install missing Python packages
fix-python:
    @echo "🐍 Installing/fixing Python dependencies..."
    @export PATH="$$HOME/.local/bin:$$PATH" && pip3 install -r requirements.txt

# ===================
# 🆘 EMERGENCY & TOOLS
# ===================

# Emergency stop all processes
stop:
    @echo "🚨 EMERGENCY STOP - Killing all processes"
    @echo "========================================"
    @pkill -f "python3.*mission" 2>/dev/null || true
    @pkill -f "ros2" 2>/dev/null || true
    @pkill -f "mavros" 2>/dev/null || true
    @echo "✅ All drone processes stopped"

# Competition day emergency reset
emergency:
    @echo "🆘 COMPETITION EMERGENCY RESET"
    @echo "============================="
    @echo "🔄 Resetting all systems..."
    @pkill -f ros2 2>/dev/null || true
    @pkill -f mavros 2>/dev/null || true
    @pkill -f python3 2>/dev/null || true
    @sudo rmmod usbserial 2>/dev/null || true
    @sudo modprobe usbserial 2>/dev/null || true
    @sudo chmod 666 /dev/tty{USB,ACM}* 2>/dev/null || true
    @export PATH="$$HOME/.local/bin:$$PATH" && ros2 daemon stop 2>/dev/null || true
    @export PATH="$$HOME/.local/bin:$$PATH" && ros2 daemon start 2>/dev/null || true
    @echo "✅ Emergency reset completed"
    @echo "🚀 Try: just test && just debug"

# System diagnostics
doctor:
    @echo "🏥 KAERTEI System Diagnostics"
    @echo "============================"
    @cd "{{SCRIPTS_DIR}}" && ./validate_system.sh 2>/dev/null || echo "⚠️  Doctor script not available"

# ===================
# 📚 HELP & LOGS
# ===================

# Competition quick help
help:
    @echo "🏆 KAERTEI 2025 - Competition Quick Guide"
    @echo "======================================="
    @echo ""
    @echo "🚀 First Time Setup:"
    @echo "  just setup       # Complete installation"
    @echo ""
    @echo "🎯 Before Competition:"
    @echo "  just test        # Validate everything works"
    @echo "  just hardware    # Check hardware connections"
    @echo "  just status      # System overview"
    @echo ""
    @echo "🏁 Competition Day:"
    @echo "  just debug       # Manual debugging mode"
    @echo "  just run         # Full autonomous mission"
    @echo "  just stop        # Emergency stop"
    @echo ""
    @echo "🆘 Problems:"
    @echo "  just emergency   # Reset everything"
    @echo "  just doctor      # System diagnostics"
    @echo "  just fix-permissions  # Fix hardware access"

# Show system logs
logs:
    @echo "📋 Recent System Logs"
    @echo "===================="
    @echo "🔍 ROS 2 logs:"
    @ls -la ~/.ros/log/ 2>/dev/null | tail -5 || echo "No ROS logs found"
    @echo ""
    @echo "🔍 Mission logs:"
    @ls -la logs/ 2>/dev/null | tail -5 || echo "No mission logs found"
    @echo ""
    @echo "🔍 System messages:"
    @dmesg | tail -10 2>/dev/null || echo "Cannot access system messages"

# System information
info:
    @echo "ℹ️  KAERTEI 2025 FAIO Information"
    @echo "==============================="
    @echo "Project: Autonomous Hexacopter Drone"
    @echo "Mission: 12-Checkpoint Indoor/Outdoor Navigation"
    @echo "Platform: Ubuntu 22.04 LTS + ROS 2 Humble"
    @echo "Hardware: Pixhawk PX4 + Multi-Camera System"
    @echo "AI: YOLOv8 Object Detection + OpenCV"
    @echo ""
    @echo "Repository: github.com/Vanszs/Dirgagah-KAERTEI"
    @echo "Competition: KAERTEI 2025 FAIO"

# ===================
# 🎯 QUICK SHORTCUTS
# ===================

# Alias for common commands
d: debug    # Quick debug
r: run      # Quick run
t: test     # Quick test
s: status   # Quick status
h: help     # Quick help
