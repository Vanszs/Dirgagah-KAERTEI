#!/bin/bash
# KAERTEI 2025 FAIO - Master Control Script
# One script to rule them all - consolidates all functionality

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Header
show_header() {
    echo -e "${BLUE}🚁 KAERTEI 2025 FAIO - Master Control System${NC}"
    echo -e "${BLUE}=============================================${NC}"
    echo -e "Environment: $(detect_environment)"
    echo -e "ROS 2: ${ROS_DISTRO:-Not sourced}"
    echo ""
}

# Detect environment
detect_environment() {
    if [ -f /.dockerenv ]; then
        echo "Docker Container"
    elif command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
        echo "Native Linux with Docker"
    else
        echo "Native Linux"
    fi
}

# Help menu
show_help() {
    echo -e "${GREEN}🚁 KAERTEI 2025 FAIO - Master Commands${NC}"
    echo ""
    echo "SETUP COMMANDS:"
    echo "  setup ubuntu        - Setup on Ubuntu/Debian"
    echo "  setup arch          - Setup on Arch Linux"
    echo "  setup docker        - Setup Docker environment"
    echo ""
    echo "MISSION COMMANDS:"
    echo "  mission debug        - Run mission in debug mode"
    echo "  mission auto         - Run autonomous mission"
    echo "  mission sim          - Run simulation"
    echo ""
    echo "TESTING COMMANDS:"
    echo "  test hardware        - Test hardware connections"
    echo "  test mavros          - Test MAVROS connectivity"  
    echo "  test dummy           - Test with dummy hardware"
    echo "  test system          - Complete system test"
    echo ""
    echo "DOCKER COMMANDS:"
    echo "  docker build         - Build Docker image"
    echo "  docker run           - Run in Docker container"
    echo "  docker status        - Show Docker status"
    echo ""
    echo "STATUS COMMANDS:"
    echo "  status               - Show system status"
    echo "  check                - Health check"
    echo ""
    echo "EXAMPLES:"
    echo "  ./kaertei_master.sh setup ubuntu"
    echo "  ./kaertei_master.sh mission debug"
    echo "  ./kaertei_master.sh docker run"
    echo "  ./kaertei_master.sh test system"
}

# Setup functions
setup_ubuntu() {
    echo -e "${YELLOW}🔧 Setting up Ubuntu environment...${NC}"
    
    # Update system
    sudo apt update && sudo apt upgrade -y
    
    # Install ROS 2 Humble
    sudo apt install -y software-properties-common curl gnupg lsb-release
    
    # Add ROS 2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS 2 packages
    sudo apt update
    sudo apt install -y ros-humble-desktop ros-humble-mavros ros-humble-mavros-extras python3-colcon-common-extensions
    
    # Install GeographicLib datasets
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    
    # Python dependencies
    pip3 install pymavlink numpy opencv-python pyserial pyyaml
    
    echo -e "${GREEN}✅ Ubuntu setup complete!${NC}"
}

setup_arch() {
    echo -e "${YELLOW}🔧 Setting up Arch Linux environment...${NC}"
    echo -e "${YELLOW}⚠️  For Arch Linux, Docker is recommended${NC}"
    echo -e "${BLUE}Running Docker setup instead...${NC}"
    setup_docker
}

setup_docker() {
    echo -e "${YELLOW}🔧 Setting up Docker environment...${NC}"
    
    # Check Docker
    if ! command -v docker >/dev/null; then
        echo -e "${RED}❌ Docker not installed${NC}"
        echo -e "${YELLOW}💡 Install Docker first: https://docs.docker.com/get-docker/${NC}"
        exit 1
    fi
    
    # Build image
    echo -e "${BLUE}🔨 Building Docker image...${NC}"
    docker build -t kaertei-2025-faio:latest .
    
    echo -e "${GREEN}✅ Docker setup complete!${NC}"
}

# Mission functions
mission_debug() {
    echo -e "${YELLOW}🚁 Starting DEBUG mission...${NC}"
    source_ros2
    python3 debug_v2.py --mode debug
}

mission_auto() {
    echo -e "${YELLOW}🚁 Starting AUTONOMOUS mission...${NC}"
    source_ros2
    python3 simulate_mission.py --mode autonomous
}

mission_sim() {
    echo -e "${YELLOW}🚁 Starting SIMULATION...${NC}"
    source_ros2
    python3 simulate_mission.py --mode simulation
}

# Test functions
test_hardware() {
    echo -e "${YELLOW}🔧 Testing hardware...${NC}"
    python3 debug_v2.py --detect-hardware
}

test_mavros() {
    echo -e "${YELLOW}🔧 Testing MAVROS...${NC}"
    source_ros2
    if ros2 pkg list | grep -q mavros; then
        echo -e "${GREEN}✅ MAVROS packages found${NC}"
        timeout 5s ros2 topic list | grep -q mavros || echo -e "${YELLOW}⚠️  MAVROS not running${NC}"
    else
        echo -e "${RED}❌ MAVROS not installed${NC}"
    fi
}

test_dummy() {
    echo -e "${YELLOW}🔧 Testing with dummy hardware...${NC}"
    python3 dummy_hardware.py
}

test_system() {
    echo -e "${YELLOW}🔧 Running complete system test...${NC}"
    test_mavros
    test_hardware
    echo -e "${GREEN}✅ System test complete${NC}"
}

# Docker functions
docker_build() {
    echo -e "${YELLOW}🔨 Building Docker image...${NC}"
    docker build -t kaertei-2025-faio:latest .
}

docker_run() {
    echo -e "${YELLOW}🚀 Running Docker container...${NC}"
    docker run --rm -it --privileged --network host \
        -v /dev:/dev -v "$(pwd):/workspace/host_data" \
        kaertei-2025-faio:latest
}

docker_status() {
    echo -e "${YELLOW}📊 Docker status...${NC}"
    if docker images | grep -q kaertei-2025-faio; then
        echo -e "${GREEN}✅ Docker image exists${NC}"
        docker images | grep kaertei-2025-faio
    else
        echo -e "${YELLOW}⚠️  Docker image not found${NC}"
    fi
}

# Status functions
show_status() {
    echo -e "${YELLOW}📊 System Status${NC}"
    echo -e "${YELLOW}===============${NC}"
    
    # Environment
    echo "Environment: $(detect_environment)"
    echo "ROS 2: ${ROS_DISTRO:-Not available}"
    
    # MAVROS
    if ros2 pkg list 2>/dev/null | grep -q mavros; then
        echo "MAVROS: Available"
    else
        echo "MAVROS: Not available"
    fi
    
    # Python packages
    if python3 -c "import pymavlink" 2>/dev/null; then
        echo "pymavlink: Available"
    else
        echo "pymavlink: Missing"
    fi
    
    if python3 -c "import numpy" 2>/dev/null; then
        echo "numpy: Available"
    else
        echo "numpy: Missing"
    fi
    
    if python3 -c "import cv2" 2>/dev/null; then
        echo "cv2: Available"
    else
        echo "cv2: Missing"
    fi
    
    # Docker
    if command -v docker >/dev/null && docker info >/dev/null 2>&1; then
        if docker images | grep -q kaertei-2025-faio; then
            echo "Docker: Available, image built"
        else
            echo "Docker: Available, image not built"
        fi
    else
        echo "Docker: Not available"
    fi
}

health_check() {
    echo -e "${YELLOW}🏥 Health Check${NC}"
    echo -e "${YELLOW}===============${NC}"
    
    local issues=0
    
    # Check ROS 2
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}❌ ROS 2 not sourced${NC}"
        issues=$((issues + 1))
    else
        echo -e "${GREEN}✅ ROS 2 $ROS_DISTRO${NC}"
    fi
    
    # Check workspace
    if [ -f "install/setup.bash" ]; then
        echo -e "${GREEN}✅ Workspace built${NC}"
    else
        echo -e "${YELLOW}⚠️  Workspace not built${NC}"
        issues=$((issues + 1))
    fi
    
    # Check hardware
    if ls /dev/tty* 2>/dev/null | grep -E "(USB|ACM)" >/dev/null; then
        echo -e "${GREEN}✅ Hardware devices found${NC}"
    else
        echo -e "${YELLOW}⚠️  No hardware devices (using dummy mode)${NC}"
    fi
    
    if [ $issues -eq 0 ]; then
        echo -e "${GREEN}🎉 System healthy!${NC}"
    else
        echo -e "${YELLOW}⚠️  $issues issue(s) found${NC}"
    fi
}

# Helper functions
source_ros2() {
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        if [ -f "install/setup.bash" ]; then
            source install/setup.bash
        fi
    fi
}

# Main script logic
main() {
    show_header
    
    case "${1:-help}" in
        "setup")
            case "$2" in
                "ubuntu") setup_ubuntu ;;
                "arch") setup_arch ;;
                "docker") setup_docker ;;
                *) echo "Usage: $0 setup [ubuntu|arch|docker]" ;;
            esac
            ;;
        "mission")
            case "$2" in
                "debug") mission_debug ;;
                "auto") mission_auto ;;
                "sim") mission_sim ;;
                *) echo "Usage: $0 mission [debug|auto|sim]" ;;
            esac
            ;;
        "test")
            case "$2" in
                "hardware") test_hardware ;;
                "mavros") test_mavros ;;
                "dummy") test_dummy ;;
                "system") test_system ;;
                *) echo "Usage: $0 test [hardware|mavros|dummy|system]" ;;
            esac
            ;;
        "docker")
            case "$2" in
                "build") docker_build ;;
                "run") docker_run ;;
                "status") docker_status ;;
                *) echo "Usage: $0 docker [build|run|status]" ;;
            esac
            ;;
        "status") show_status ;;
        "check") health_check ;;
        "help"|*) show_help ;;
    esac
}

# Run main function
main "$@"
