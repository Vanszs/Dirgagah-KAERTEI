#!/bin/bash
# KAERTEI 2025 FAIO - Script Overview and Verification
# ===================================================
# Shows all available scripts and their status

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

print_header() {
    echo ""
    echo -e "${MAGENTA}$1${NC}"
    echo -e "${MAGENTA}$(echo "$1" | sed 's/./=/g')${NC}"
}

check_script() {
    local script_name="$1"
    local description="$2"
    
    if [ -f "$script_name" ]; then
        if [ -x "$script_name" ]; then
            echo -e "${GREEN}✅ $script_name${NC} - $description"
        else
            echo -e "${YELLOW}⚠️  $script_name${NC} - $description (not executable)"
        fi
    else
        echo -e "${RED}❌ $script_name${NC} - $description (missing)"
    fi
}

main() {
    print_header "🏆 KAERTEI 2025 FAIO - SCRIPT OVERVIEW"
    
    echo -e "${CYAN}All scripts for Ubuntu 22.04 competition setup and execution${NC}"
    
    print_header "🚀 MAIN COMPETITION SCRIPTS"
    check_script "competition_startup.sh" "One-command competition startup menu"
    check_script "validate_system.sh" "Complete system validation (this script)"
    check_script "Justfile" "Just command runner configuration"
    
    print_header "⚙️ SETUP AND INSTALLATION SCRIPTS"
    check_script "install_just.sh" "Install Just command runner"
    check_script "setup_ubuntu.sh" "Complete Ubuntu 22.04 setup"
    check_script "doctor_ubuntu.py" "System diagnostics and troubleshooting"
    
    print_header "🧪 TESTING AND MISSION SCRIPTS"
    check_script "test_ubuntu.sh" "Run comprehensive system tests"
    check_script "launch_mission_ubuntu.sh" "Launch mission execution"
    check_script "validate_ubuntu.py" "Python-based validation tools"
    
    print_header "📋 CONFIGURATION FILES"
    check_script "requirements.txt" "Python package dependencies (Ubuntu 22.04 optimized)"
    check_script "package.xml" "ROS 2 package configuration"
    check_script "CMakeLists.txt" "Build system configuration"
    
    print_header "🎯 USAGE GUIDE"
    echo ""
    echo -e "${CYAN}Quick Start (Ubuntu 22.04):${NC}"
    echo -e "${GREEN}1. ./competition_startup.sh${NC}    # Interactive competition menu"
    echo -e "${GREEN}2. just setup${NC}                  # Complete system setup"
    echo -e "${GREEN}3. just test${NC}                   # Validate everything works"
    echo -e "${GREEN}4. just mission-debug${NC}          # Practice mission"
    echo -e "${GREEN}5. just mission-auto${NC}           # Competition mode"
    echo ""
    
    echo -e "${CYAN}System Validation:${NC}"
    echo -e "${GREEN}• ./validate_system.sh${NC}         # Complete system check"
    echo -e "${GREEN}• just doctor${NC}                  # Diagnose issues"
    echo -e "${GREEN}• just status${NC}                  # Quick status"
    echo ""
    
    echo -e "${CYAN}Emergency Procedures:${NC}"
    echo -e "${GREEN}• just emergency${NC}               # Emergency stop/reset"
    echo -e "${GREEN}• just clean${NC}                   # Clean and rebuild"
    echo -e "${GREEN}• just help${NC}                    # Show all commands"
    echo ""

    print_header "🔧 SYSTEM STATUS"
    
    # Check Just
    if command -v just >/dev/null 2>&1; then
        echo -e "${GREEN}✅ Just command runner installed${NC}"
    else
        echo -e "${YELLOW}⚠️  Just not installed - run: ./install_just.sh${NC}"
    fi
    
    # Check ROS 2
    if command -v ros2 >/dev/null 2>&1; then
        echo -e "${GREEN}✅ ROS 2 available${NC}"
    else
        echo -e "${YELLOW}⚠️  ROS 2 not installed - run: just setup${NC}"
    fi
    
    # Check Python packages
    if python3 -c "import pymavlink, cv2, numpy" 2>/dev/null; then
        echo -e "${GREEN}✅ Core Python packages available${NC}"
    else
        echo -e "${YELLOW}⚠️  Python packages missing - run: just setup${NC}"
    fi
    
    echo ""
    echo -e "${MAGENTA}🏆 Ready for KAERTEI 2025 FAIO Competition!${NC}"
    echo -e "${CYAN}For complete validation, run: ./validate_system.sh${NC}"
}

main "$@"
