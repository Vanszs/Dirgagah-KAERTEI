#!/bin/bash
# KAERTEI 2025 FAIO - Competition Ready Setup Script
# Automated setup for Ubuntu, Arch, and Docker environments
# Zero to competition ready in one command!

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🏆 KAERTEI 2025 FAIO - Competition Ready Setup${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Detect OS
detect_os() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
        echo -e "${BLUE}🔍 Detected OS: $OS${NC}"
    else
        echo -e "${RED}❌ Cannot detect OS${NC}"
        exit 1
    fi
}

# Check if running as root
check_root() {
    if [ "$EUID" -eq 0 ]; then
        echo -e "${RED}❌ Do not run this script as root${NC}"
        echo "Run as normal user - script will ask for sudo when needed"
        exit 1
    fi
}

# Setup for Ubuntu - Competition Ready
setup_ubuntu() {
    echo -e "${YELLOW}� Setting up Ubuntu for KAERTEI 2025 competition...${NC}"
    
    # Update system
    echo "📦 Updating system packages..."
    sudo apt update && sudo apt upgrade -y
    
    # Install ROS 2 Humble if not present
    if ! command -v ros2 &> /dev/null; then
        echo -e "${YELLOW}🤖 Installing ROS 2 Humble...${NC}"
        sudo apt install software-properties-common curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        sudo apt update
        sudo apt install ros-humble-desktop -y
        sudo apt install python3-rosdep2 -y
        sudo rosdep init || true
        rosdep update
    else
        echo -e "${GREEN}✅ ROS 2 already installed${NC}"
    fi
    
    # Install MAVROS (competition requirement)
    echo "📡 Installing MAVROS..."
    sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
    
    # Install essential system packages
    echo "📦 Installing system dependencies..."
    sudo apt install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-opencv \
        python3-numpy \
        python3-yaml \
        python3-serial \
        v4l-utils \
        git \
        curl \
        nano
    
    # Install Python packages
    echo "🐍 Installing Python dependencies..."
    pip3 install --user -r requirements.txt
    
    # Install GeographicLib datasets (critical for GPS)
    echo "🗺️ Installing geographic datasets..."
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    
    # Setup user permissions for hardware access
    echo "🔐 Setting up hardware permissions..."
    sudo usermod -a -G dialout,video $USER
    
    # Auto-source ROS 2 in bashrc if not already present
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "🔧 Adding ROS 2 to bashrc..."
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    echo -e "${GREEN}✅ Ubuntu setup complete for competition!${NC}"
    echo -e "${YELLOW}⚠️  Please logout and login again for group changes to take effect${NC}"
}

# Setup for Arch Linux - Docker Recommended
setup_arch() {
    echo -e "${YELLOW}🏴‍☠️ Arch Linux detected - Using Docker for better compatibility...${NC}"
    echo -e "${BLUE}� Docker provides more stable ROS 2 environment on Arch${NC}"
    
    # Install Docker if not present
    if ! command -v docker &> /dev/null; then
        echo "🐳 Installing Docker..."
        sudo pacman -S docker docker-compose --noconfirm
        sudo systemctl enable docker
        sudo systemctl start docker
        sudo usermod -aG docker $USER
        echo -e "${YELLOW}⚠️  Please logout and login again for Docker group changes${NC}"
    fi
    
    # Install basic dependencies for host system
    echo "📦 Installing basic dependencies..."
    sudo pacman -S python python-pip git --noconfirm
    pip install --user pymavlink opencv-python numpy
    
    # Build Docker image
    setup_docker
    
    echo -e "${GREEN}✅ Arch Linux setup complete (Docker-based)${NC}"
}

# Setup Docker environment - Universal Solution
setup_docker() {
    echo -e "${YELLOW}🐳 Setting up Docker environment (universal solution)...${NC}"
    
    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        echo "📦 Installing Docker..."
        if [ "$OS" = "ubuntu" ]; then
            sudo apt update
            sudo apt install docker.io docker-compose -y
        elif [ "$OS" = "arch" ]; then
            sudo pacman -S docker docker-compose --noconfirm
        else
            echo -e "${RED}❌ Unsupported OS for Docker auto-install${NC}"
            echo "Please install Docker manually: https://docs.docker.com/get-docker/"
            exit 1
        fi
        
        # Start and enable Docker
        sudo systemctl enable docker
        sudo systemctl start docker
        sudo usermod -aG docker $USER
        
        echo -e "${YELLOW}⚠️  Please logout and login again for Docker group changes${NC}"
        echo -e "${BLUE}💡 After re-login, run: just docker-build${NC}"
    else
        echo -e "${GREEN}✅ Docker already installed${NC}"
    fi
    
    # Build Docker image if Dockerfile exists
    if [ -f "Dockerfile" ]; then
        echo "🔨 Building Docker image..."
        ./docker_runner.sh build
    else
        echo -e "${YELLOW}⚠️  Dockerfile not found - Docker image not built${NC}"
    fi
    
    echo -e "${GREEN}✅ Docker setup complete${NC}"
}

# Build workspace - Competition Ready
build_workspace() {
    echo -e "${YELLOW}🔨 Building ROS 2 workspace for competition...${NC}"
    
    # Navigate to workspace root
    cd ../../
    
    # Source ROS 2 environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo -e "${YELLOW}⚠️  ROS 2 not found - building without ROS 2 environment${NC}"
    fi
    
    # Build the package
    echo "🔧 Building drone_mvp package..."
    colcon build --packages-select drone_mvp --cmake-clean-first
    
    # Source the built workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo -e "${GREEN}✅ Workspace built and sourced successfully${NC}"
    else
        echo -e "${RED}❌ Workspace build failed${NC}"
        exit 1
    fi
    
    # Return to original directory
    cd src/drone_mvp/
}

# Post-setup verification
verify_setup() {
    echo -e "${YELLOW}🔍 Verifying competition readiness...${NC}"
    
    # Test Python dependencies
    python3 -c "import cv2, numpy as np, pymavlink; print('✅ Core Python dependencies OK')" || echo "❌ Python dependencies missing"
    
    # Test ROS 2 if available
    if command -v ros2 &> /dev/null; then
        echo "✅ ROS 2 available"
    else
        echo "⚠️  ROS 2 not in PATH (may be normal for Docker setup)"
    fi
    
    # Check hardware devices
    if ls /dev/tty{USB,ACM}* 2>/dev/null; then
        echo "✅ Hardware devices detected"
    else
        echo "⚠️  No hardware devices - will use dummy mode for testing"
    fi
    
    # Check configuration file
    if [ -f "config/hardware_config.conf" ]; then
        echo "✅ Configuration file found"
    else
        echo "❌ Configuration file missing"
    fi
    
    echo ""
    echo -e "${GREEN}🎉 COMPETITION SETUP VERIFICATION COMPLETE!${NC}"
}

# Show help
show_help() {
    echo -e "${GREEN}🏆 KAERTEI 2025 FAIO - Competition Setup${NC}"
    echo ""
    echo "Usage: $0 [platform]"
    echo ""
    echo "Platforms:"
    echo "  ubuntu    - Setup for Ubuntu (native, recommended)"
    echo "  arch      - Setup for Arch Linux (uses Docker)"
    echo "  docker    - Setup Docker environment (universal)"
    echo "  build     - Build ROS 2 workspace only"
    echo ""
    echo "🚀 COMPETITION WORKFLOW:"
    echo "  1. $0 ubuntu      # Setup platform"
    echo "  2. just test      # Verify system"
    echo "  3. just mission   # Run competition mission"
    echo ""
    echo "💡 For any platform, Docker is the most reliable option"
}

# Main execution
main() {
    detect_os
    check_root
    
    case "${1:-help}" in
        ubuntu)
            if [ "$OS" != "ubuntu" ]; then
                echo -e "${RED}❌ This is not Ubuntu system (detected: $OS)${NC}"
                echo -e "${BLUE}💡 Try: $0 docker${NC}"
                exit 1
            fi
            setup_ubuntu
            build_workspace
            verify_setup
            ;;
        arch)
            if [ "$OS" != "arch" ]; then
                echo -e "${RED}❌ This is not Arch Linux system (detected: $OS)${NC}"
                echo -e "${BLUE}💡 Try: $0 docker${NC}"
                exit 1
            fi
            setup_arch
            ;;
        docker)
            setup_docker
            ;;
        build)
            build_workspace
            ;;
        help|*)
            show_help
            ;;
    esac
    
    if [ "${1}" != "help" ] && [ "${1}" != "" ]; then
        echo ""
        echo -e "${GREEN}🎉 KAERTEI 2025 FAIO SETUP COMPLETE!${NC}"
        echo -e "${BLUE}============================================${NC}"
        echo ""
        echo -e "${YELLOW}🎯 NEXT STEPS FOR COMPETITION:${NC}"
        echo "  1. Test system:    just test"
        echo "  2. Run mission:    just mission"
        echo "  3. For help:       just help"
        echo ""
        echo -e "${YELLOW}📝 IMPORTANT REMINDERS:${NC}"
        echo "  • Update GPS coordinates in config/hardware_config.conf"
        echo "  • Test hardware connections before competition"
        echo "  • Have RC transmitter ready for manual override"
        echo ""
        echo -e "${GREEN}Good luck with KAERTEI 2025! 🏆${NC}"
    fi
}

# Run main function
main "$@"
