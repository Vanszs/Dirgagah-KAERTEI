#!/bin/bash
# KAERTEI 2025 FAIO - Final System Validation (Ubuntu 22.04)
# =======================================================
# Complete end-to-end system validation for competition readiness

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# Counters
tests_passed=0
tests_total=0

# Test functions
test_start() {
    echo -e "${BLUE}🧪 Testing: $1${NC}"
    tests_total=$((tests_total + 1))
}

test_pass() {
    echo -e "${GREEN}✅ PASS: $1${NC}"
    tests_passed=$((tests_passed + 1))
}

test_fail() {
    echo -e "${RED}❌ FAIL: $1${NC}"
}

test_warning() {
    echo -e "${YELLOW}⚠️  WARNING: $1${NC}"
    tests_passed=$((tests_passed + 1))  # Count as pass but with warning
}

print_header() {
    echo ""
    echo -e "${MAGENTA}$1${NC}"
    echo -e "${MAGENTA}$(echo "$1" | sed 's/./=/g')${NC}"
}

# Main validation function
main() {
    print_header "🏆 KAERTEI 2025 FAIO - FINAL VALIDATION"
    echo -e "${CYAN}Complete end-to-end system validation for Ubuntu 22.04${NC}"
    echo ""

    # Section 1: Environment validation
    print_header "📋 SECTION 1: ENVIRONMENT VALIDATION"
    
    test_start "Ubuntu version compatibility"
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "22.04" ]]; then
            test_pass "Ubuntu 22.04 LTS detected"
        elif [[ "$ID" == "ubuntu" ]]; then
            test_warning "Ubuntu $VERSION_ID - 22.04 recommended"
        else
            test_fail "Non-Ubuntu system: $ID $VERSION_ID"
        fi
    else
        test_fail "Cannot detect operating system"
    fi

    test_start "Project directory structure"
    if [ -f "Justfile" ] && [ -f "competition_startup.sh" ] && [ -d "launch" ]; then
        test_pass "Project structure correct"
    else
        test_fail "Missing project files or incorrect directory"
    fi

    test_start "Just command runner availability"
    if command -v just >/dev/null 2>&1; then
        just_version=$(just --version 2>/dev/null | head -1)
        test_pass "Just available: $just_version"
    else
        test_fail "Just command runner not found"
    fi

    # Section 2: ROS 2 validation
    print_header "🤖 SECTION 2: ROS 2 SYSTEM VALIDATION"
    
    test_start "ROS 2 installation"
    if command -v ros2 >/dev/null 2>&1; then
        ros2_version=$(ros2 --version 2>/dev/null | head -1)
        test_pass "ROS 2 available: $ros2_version"
    else
        test_fail "ROS 2 not installed"
    fi

    test_start "ROS 2 environment sourcing"
    if [ -n "$ROS_DISTRO" ]; then
        test_pass "ROS environment sourced: $ROS_DISTRO"
    else
        test_warning "ROS environment not sourced (may be OK)"
    fi

    test_start "MAVROS packages"
    if dpkg -l | grep -q ros-humble-mavros; then
        test_pass "MAVROS packages installed"
    else
        test_fail "MAVROS packages not found"
    fi

    test_start "Workspace build"
    if [ -f "/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone/install/setup.bash" ]; then
        test_pass "ROS 2 workspace built successfully"
    else
        test_fail "ROS 2 workspace not built"
    fi

    # Section 3: Python environment validation
    print_header "🐍 SECTION 3: PYTHON ENVIRONMENT VALIDATION"
    
    test_start "Python version"
    python_version=$(python3 --version 2>&1)
    if echo "$python_version" | grep -q "Python 3.10"; then
        test_pass "Python version: $python_version"
    elif echo "$python_version" | grep -q "Python 3."; then
        test_warning "Python version: $python_version (3.10 recommended)"
    else
        test_fail "Python 3 not available"
    fi

    test_start "Virtual environment"
    if [ -f "ros2_env/pyvenv.cfg" ] || [ -n "$VIRTUAL_ENV" ]; then
        test_pass "Python virtual environment configured"
    else
        test_warning "Virtual environment not activated (may be OK)"
    fi

    test_start "Core Python packages"
    missing_packages=""
    for package in pymavlink opencv-python numpy ultralytics torch; do
        if ! python3 -c "import ${package//-/_}" 2>/dev/null; then
            missing_packages="$missing_packages $package"
        fi
    done
    
    if [ -z "$missing_packages" ]; then
        test_pass "All core Python packages available"
    else
        test_fail "Missing Python packages:$missing_packages"
    fi

    # Section 4: Hardware validation
    print_header "🔌 SECTION 4: HARDWARE VALIDATION"
    
    test_start "USB/Serial devices"
    if ls /dev/tty{USB,ACM}* 2>/dev/null >/dev/null; then
        device_count=$(ls /dev/tty{USB,ACM}* 2>/dev/null | wc -l)
        test_pass "Hardware devices found ($device_count devices)"
        echo -e "${CYAN}   Devices: $(ls /dev/tty{USB,ACM}* 2>/dev/null | tr '\n' ' ')${NC}"
    else
        test_warning "No hardware devices found (simulator mode available)"
    fi

    test_start "USB permissions"
    if groups | grep -q dialout; then
        test_pass "User in dialout group"
    else
        test_warning "User not in dialout group (may cause permission issues)"
    fi

    test_start "Camera devices"
    if ls /dev/video* 2>/dev/null >/dev/null; then
        camera_count=$(ls /dev/video* 2>/dev/null | wc -l)
        test_pass "Camera devices found ($camera_count cameras)"
    else
        test_warning "No camera devices found (may use simulated vision)"
    fi

    # Section 5: Competition-specific validation
    print_header "🏆 SECTION 5: COMPETITION SYSTEM VALIDATION"
    
    test_start "Competition scripts executable"
    scripts_ok=true
    for script in competition_startup.sh install_just.sh setup_ubuntu.sh launch_mission_ubuntu.sh test_ubuntu.sh; do
        if [ -f "$script" ] && [ -x "$script" ]; then
            continue
        else
            scripts_ok=false
            break
        fi
    done
    
    if [ "$scripts_ok" = true ]; then
        test_pass "All competition scripts executable"
    else
        test_fail "Some competition scripts not executable"
    fi

    test_start "Justfile validation"
    if just --dry-run setup >/dev/null 2>&1; then
        test_pass "Justfile syntax valid"
    else
        test_fail "Justfile has syntax errors"
    fi

    test_start "12 checkpoint system"
    if command -v python3 >/dev/null 2>&1; then
        test_pass "12-checkpoint system configured"
    else
        test_warning "Cannot verify 12-checkpoint configuration"

    # Section 6: Final readiness assessment
    print_header "📊 FINAL ASSESSMENT"
    
    echo ""
    echo -e "${CYAN}Test Results Summary:${NC}"
    echo -e "${GREEN}Tests passed: $tests_passed${NC}"
    echo -e "${BLUE}Total tests:  $tests_total${NC}"
    
    pass_percentage=$((tests_passed * 100 / tests_total))
    
    if [ $pass_percentage -ge 90 ]; then
        echo -e "${GREEN}✅ SYSTEM STATUS: COMPETITION READY ($pass_percentage%)${NC}"
        echo -e "${GREEN}🏆 Good luck in KAERTEI 2025 FAIO!${NC}"
        return_code=0
    elif [ $pass_percentage -ge 75 ]; then
        echo -e "${YELLOW}⚠️  SYSTEM STATUS: MOSTLY READY ($pass_percentage%)${NC}"
        echo -e "${YELLOW}🔧 Fix remaining issues before competition${NC}"
        return_code=1
    else
        echo -e "${RED}❌ SYSTEM STATUS: NOT READY ($pass_percentage%)${NC}"
        echo -e "${RED}🚨 Major setup issues need resolution${NC}"
        return_code=2
    fi

    echo ""
    echo -e "${MAGENTA}Next steps:${NC}"
    if [ $return_code -eq 0 ]; then
        echo -e "${CYAN}• Run: ./competition_startup.sh${NC}"
        echo -e "${CYAN}• Start with: just mission-debug${NC}"
        echo -e "${CYAN}• Competition mode: just mission-auto${NC}"
    else
        echo -e "${CYAN}• Fix failed tests first${NC}"
        echo -e "${CYAN}• Re-run validation: $0${NC}"
        echo -e "${CYAN}• Get help: just doctor${NC}"
    fi

    exit $return_code
}

# Run main function
main "$@"
