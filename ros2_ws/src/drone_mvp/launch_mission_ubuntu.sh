#!/bin/bash
# KAERTEI 2025 FAIO - Ubuntu 22.04 Mission Launcher
# Optimized checkpoint mission execution

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Print with colors
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

echo -e "${BLUE}🎯 KAERTEI 2025 FAIO - Mission Launcher${NC}"
echo -e "${BLUE}=======================================${NC}"
echo ""

# Check if ROS 2 workspace exists and is built
check_workspace() {
    local workspace_setup="/home/vanszs/Documents/ros2/ros2_ws/install/setup.bash"
    if [ -f "$workspace_setup" ]; then
        print_success "✅ ROS 2 workspace found and built"
        return 0
    else
        print_warning "⚠️  ROS 2 workspace not built"
        return 1
    fi
}

# Check Python environment
check_python_env() {
    local venv_path="/home/vanszs/Documents/ros2/ros2_env"
    if [ -d "$venv_path" ]; then
        source "$venv_path/bin/activate"
        if python3 -c "import pymavlink, cv2, numpy" 2>/dev/null; then
            print_success "✅ Python environment ready"
            return 0
        else
            print_warning "⚠️  Python dependencies missing"
            deactivate
            return 1
        fi
        deactivate
    else
        print_warning "⚠️  Python virtual environment not found"
        return 1
    fi
}

# Launch ROS 2 based mission
launch_ros2_mission() {
    local mode="$1"
    print_step "🤖 Launching ROS 2 mission in $mode mode..."
    
    # Source ROS 2 and workspace
    source /opt/ros/humble/setup.bash
    source /home/vanszs/Documents/ros2/ros2_ws/install/setup.bash
    
    # Activate Python environment
    source /home/vanszs/Documents/ros2/ros2_env/bin/activate
    
    cd /home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp
    
    case "$mode" in
        "debug")
            print_info "🐛 Starting debug mission with step-by-step control..."
            python3 checkpoint_mission_mavros.py --debug
            ;;
        "auto")
            print_info "🚀 Starting autonomous mission..."
            python3 checkpoint_mission_mavros.py --autonomous
            ;;
        "test")
            print_info "🧪 Starting test mission..."
            python3 checkpoint_mission_mavros.py --test
            ;;
        *)
            print_error "❌ Invalid mode: $mode"
            exit 1
            ;;
    esac
}

# Launch direct Python mission (fallback)
launch_direct_mission() {
    local mode="$1"
    print_step "⚡ Launching direct Python mission in $mode mode..."
    
    # Activate Python environment
    source /home/vanszs/Documents/ros2/ros2_env/bin/activate
    
    cd /home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp
    
    case "$mode" in
        "debug")
            print_info "🐛 Starting debug mission (direct Python)..."
            python3 checkpoint_mission_node.py --debug
            ;;
        "auto")
            print_info "🚀 Starting autonomous mission (direct Python)..."
            python3 checkpoint_mission_node.py --autonomous
            ;;
        "test")
            print_info "🧪 Starting test mission (direct Python)..."
            python3 checkpoint_mission_node.py --test
            ;;
        *)
            print_error "❌ Invalid mode: $mode"
            exit 1
            ;;
    esac
}

# Smart mission launcher
smart_launch() {
    local mode="${1:-debug}"
    
    print_info "🧠 Smart mission detection..."
    
    # Check if we have a built ROS 2 workspace
    if check_workspace && check_python_env; then
        print_info "✅ Using ROS 2 workspace (recommended)"
        launch_ros2_mission "$mode"
    elif check_python_env; then
        print_info "⚡ Using direct Python execution"
        launch_direct_mission "$mode"
    else
        print_error "❌ Python environment not ready"
        print_info "💡 Run: just setup"
        exit 1
    fi
}

# Pre-flight checks
pre_flight_check() {
    print_step "🔍 Pre-flight system check..."
    
    # Check hardware
    if ls /dev/tty{USB,ACM}* 2>/dev/null >/dev/null; then
        print_success "✅ Hardware devices detected"
    else
        print_warning "⚠️  No hardware - using dummy mode"
    fi
    
    # Check configuration
    if [ -f "config/hardware_config.conf" ]; then
        print_success "✅ Hardware configuration found"
    else
        print_error "❌ Hardware configuration missing"
        return 1
    fi
    
    # Check mission files
    if [ -f "checkpoint_mission_mavros.py" ] || [ -f "checkpoint_mission_node.py" ]; then
        print_success "✅ Mission files available"
    else
        print_error "❌ Mission files missing"
        return 1
    fi
    
    return 0
}

# Main execution
main() {
    local mode="${1:-debug}"
    
    print_info "🎯 Mission mode: $mode"
    
    # Validate mode
    case "$mode" in
        "debug"|"auto"|"test")
            ;;
        *)
            print_error "❌ Invalid mode: $mode"
            echo "   Valid modes: debug, auto, test"
            exit 1
            ;;
    esac
    
    # Pre-flight checks
    if ! pre_flight_check; then
        print_error "❌ Pre-flight check failed"
        exit 1
    fi
    
    # Launch mission
    smart_launch "$mode"
}

# Help function
show_help() {
    echo -e "${CYAN}KAERTEI 2025 FAIO - Mission Launcher${NC}"
    echo ""
    echo "Usage: $0 [mode]"
    echo ""
    echo "Mission modes:"
    echo "  debug    - Debug mode with step-by-step control (default)"
    echo "  auto     - Autonomous competition mode"
    echo "  test     - Test mode for validation"
    echo ""
    echo "Examples:"
    echo "  $0 debug    # Practice with debug controls"
    echo "  $0 auto     # Competition autonomous run"
    echo "  $0 test     # System validation"
    echo ""
    echo "Competition workflow:"
    echo "  1. just test         # Verify system"
    echo "  2. just mission      # Practice run"
    echo "  3. just mission-auto # Competition run"
}

# Handle command line arguments
case "${1:-debug}" in
    "help"|"-h"|"--help")
        show_help
        ;;
    *)
        main "$1"
        ;;
esac
