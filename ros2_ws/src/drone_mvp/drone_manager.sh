#!/bin/bash

# KAERTEI 2025 Drone System Management Script
# Main interface for all drone operations

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

print_header() {
    echo -e "${BLUE}============================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}============================================${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo -e "${CYAN}ℹ️  $1${NC}"
}

show_help() {
    cat << EOF
🚁 KAERTEI 2025 Drone System Management Script

USAGE:
    ./drone_manager.sh [COMMAND] [OPTIONS]

COMMANDS:
    setup           - Initial system setup and dependencies
    build           - Build the ROS2 workspace
    test-mavros     - Test MAVROS connection to flight controller
    calibrate       - Calibrate and test all hardware components
    simulate        - Run mission simulation (no hardware required)
    monitor         - Real-time system monitoring dashboard
    mission         - Start the actual mission
    emergency       - Emergency stop and landing
    status          - Show system status
    logs            - View system logs
    clean           - Clean build files and logs
    help            - Show this help message

OPTIONS:
    --fcu-url URL   - Flight controller URL (default: serial:///dev/ttyUSB0:57600)
    --gcs-url URL   - Ground control station URL (default: udp://@127.0.0.1:14550)
    --camera DEVICE - Camera device (front, back, top)
    --sim           - Use simulation mode
    --verbose       - Verbose output
    --dry-run       - Show commands without executing

EXAMPLES:
    ./drone_manager.sh setup                    # Initial setup
    ./drone_manager.sh test-mavros              # Test connection
    ./drone_manager.sh calibrate                # Hardware check
    ./drone_manager.sh mission --camera front   # Start mission
    ./drone_manager.sh monitor                  # Monitor system
    ./drone_manager.sh simulate                 # Test without hardware

For KAERTEI 2025 Fully Autonomous Indoor-Outdoor (FAIO) Competition
EOF
}

check_dependencies() {
    print_info "Checking dependencies..."
    
    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please install ROS2 Humble"
        exit 1
    fi
    
    # Check workspace
    if [ ! -f "$WORKSPACE_DIR/src/drone_mvp/package.xml" ]; then
        print_error "Drone MVP package not found in workspace"
        exit 1
    fi
    
    # Check if workspace is built
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        print_warning "Workspace not built. Run: ./drone_manager.sh build"
    fi
    
    print_success "Dependencies check completed"
}

setup_system() {
    print_header "SYSTEM SETUP"
    
    # Run setup script
    if [ -f "$SCRIPT_DIR/setup.sh" ]; then
        print_info "Running setup script..."
        bash "$SCRIPT_DIR/setup.sh"
    else
        print_error "Setup script not found"
        exit 1
    fi
    
    print_success "System setup completed"
}

build_workspace() {
    print_header "BUILDING WORKSPACE"
    
    cd "$WORKSPACE_DIR"
    
    print_info "Building ROS2 workspace..."
    colcon build --packages-select drone_mvp
    
    print_success "Workspace built successfully"
}

test_mavros() {
    print_header "TESTING MAVROS CONNECTION"
    
    local fcu_url="${FCU_URL:-serial:///dev/ttyUSB0:57600}"
    local gcs_url="${GCS_URL:-udp://@127.0.0.1:14550}"
    
    if [ -f "$SCRIPT_DIR/test_mavros.sh" ]; then
        print_info "Testing MAVROS connection..."
        bash "$SCRIPT_DIR/test_mavros.sh" "$fcu_url" "$gcs_url"
    else
        print_error "MAVROS test script not found"
        exit 1
    fi
}

calibrate_hardware() {
    print_header "HARDWARE CALIBRATION"
    
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    print_info "Starting hardware calibration..."
    python3 src/drone_mvp/calibrate_hardware.py
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        print_success "All hardware tests passed"
    elif [ $exit_code -eq 1 ]; then
        print_warning "Some hardware tests failed - check output"
    else
        print_error "Hardware calibration failed"
        exit 1
    fi
}

run_simulation() {
    print_header "MISSION SIMULATION"
    
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    print_info "Starting mission simulation..."
    if [ "$1" = "test" ]; then
        python3 src/drone_mvp/simulate_mission.py test
    else
        python3 src/drone_mvp/simulate_mission.py
    fi
}

start_monitor() {
    print_header "SYSTEM MONITOR"
    
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    print_info "Starting real-time monitor..."
    print_info "Press 'q' or ESC to quit monitor"
    python3 src/drone_mvp/monitor.py
}

start_mission() {
    print_header "STARTING MISSION"
    
    local camera="${CAMERA:-front}"
    local fcu_url="${FCU_URL:-serial:///dev/ttyUSB0:57600}"
    local gcs_url="${GCS_URL:-udp://@127.0.0.1:14550}"
    
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    print_info "Pre-flight checks..."
    
    # Quick hardware check
    print_info "Running quick hardware check..."
    if ! python3 src/drone_mvp/calibrate_hardware.py --quick; then
        print_error "Hardware check failed. Run full calibration first."
        exit 1
    fi
    
    print_success "Pre-flight checks passed"
    print_info "Starting mission with camera: $camera"
    
    if [ "$VERBOSE" = "true" ]; then
        ros2 launch drone_mvp drone.launch.py camera:="$camera" fcu_url:="$fcu_url" gcs_url:="$gcs_url"
    else
        ros2 launch drone_mvp drone.launch.py camera:="$camera" fcu_url:="$fcu_url" gcs_url:="$gcs_url" > mission.log 2>&1 &
        
        local mission_pid=$!
        print_success "Mission started (PID: $mission_pid)"
        print_info "Logs: tail -f mission.log"
        print_info "Monitor: ./drone_manager.sh monitor"
        print_info "Stop: kill $mission_pid"
    fi
}

emergency_stop() {
    print_header "EMERGENCY STOP"
    
    print_warning "Initiating emergency stop..."
    
    # Kill all drone processes
    pkill -f "drone_mvp" || true
    pkill -f "mavros" || true
    
    # Send emergency land command if possible
    cd "$WORKSPACE_DIR"
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        
        print_info "Sending emergency land command..."
        ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{min_pitch: 0, yaw: 0, latitude: 0, longitude: 0, altitude: 0}" || true
    fi
    
    print_success "Emergency stop completed"
}

show_status() {
    print_header "SYSTEM STATUS"
    
    # Check if workspace is built
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        print_success "Workspace: Built"
    else
        print_error "Workspace: Not built"
    fi
    
    # Check for running processes
    if pgrep -f "drone_mvp" > /dev/null; then
        print_success "Drone system: Running"
    else
        print_info "Drone system: Stopped"
    fi
    
    if pgrep -f "mavros" > /dev/null; then
        print_success "MAVROS: Running"
    else
        print_info "MAVROS: Stopped"
    fi
    
    # Check hardware
    if [ -f "/dev/ttyUSB0" ]; then
        print_success "Flight controller: Connected (/dev/ttyUSB0)"
    elif [ -f "/dev/ttyACM0" ]; then
        print_success "Flight controller: Connected (/dev/ttyACM0)"
    else
        print_warning "Flight controller: Not detected"
    fi
    
    # Check recent logs
    if [ -f "mission.log" ]; then
        local log_size=$(wc -c < mission.log)
        print_info "Mission log: $log_size bytes"
    fi
    
    # Check calibration results
    if [ -f "calibration_results.json" ]; then
        print_info "Last calibration: $(stat -c %y calibration_results.json | cut -d' ' -f1)"
    fi
}

view_logs() {
    print_header "SYSTEM LOGS"
    
    local log_type="${1:-mission}"
    
    case $log_type in
        mission)
            if [ -f "mission.log" ]; then
                print_info "Mission log (last 50 lines):"
                tail -50 mission.log
            else
                print_warning "No mission log found"
            fi
            ;;
        ros)
            print_info "ROS2 logs:"
            if [ -d "~/.ros/log" ]; then
                ls -la ~/.ros/log/ | tail -10
            else
                print_warning "No ROS2 logs found"
            fi
            ;;
        calibration)
            if [ -f "calibration_results.json" ]; then
                print_info "Calibration results:"
                cat calibration_results.json
            else
                print_warning "No calibration results found"
            fi
            ;;
        *)
            print_error "Unknown log type: $log_type"
            print_info "Available: mission, ros, calibration"
            ;;
    esac
}

clean_system() {
    print_header "CLEANING SYSTEM"
    
    cd "$WORKSPACE_DIR"
    
    print_info "Cleaning build files..."
    rm -rf build/ install/ log/
    
    print_info "Cleaning logs..."
    rm -f mission.log calibration_results.json
    
    print_info "Cleaning cache..."
    rm -rf ~/.ros/log/*
    
    print_success "System cleaned"
}

# Parse command line arguments
COMMAND=""
FCU_URL=""
GCS_URL=""
CAMERA=""
VERBOSE=false
DRY_RUN=false
SIM_MODE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        setup|build|test-mavros|calibrate|simulate|monitor|mission|emergency|status|logs|clean|help)
            COMMAND="$1"
            shift
            ;;
        --fcu-url)
            FCU_URL="$2"
            shift 2
            ;;
        --gcs-url)
            GCS_URL="$2"
            shift 2
            ;;
        --camera)
            CAMERA="$2"
            shift 2
            ;;
        --sim)
            SIM_MODE=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Main execution
print_header "KAERTEI 2025 Drone System Manager"

if [ "$DRY_RUN" = "true" ]; then
    print_warning "DRY RUN MODE - Commands will be shown but not executed"
fi

case $COMMAND in
    setup)
        setup_system
        ;;
    build)
        build_workspace
        ;;
    test-mavros)
        test_mavros
        ;;
    calibrate)
        calibrate_hardware
        ;;
    simulate)
        run_simulation
        ;;
    monitor)
        start_monitor
        ;;
    mission)
        start_mission
        ;;
    emergency)
        emergency_stop
        ;;
    status)
        show_status
        ;;
    logs)
        view_logs "${2:-mission}"
        ;;
    clean)
        clean_system
        ;;
    help|"")
        show_help
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        show_help
        exit 1
        ;;
esac

print_success "Operation completed"
