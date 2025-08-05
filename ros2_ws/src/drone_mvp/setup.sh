#!/bin/bash
# KAERTEI 2025 FAIO - System Setup Script
# Automated setup for Ubuntu, Arch, and Docker environments

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚁 KAERTEI 2025 FAIO - System Setup${NC}"
echo -e "${BLUE}====================================${NC}"

# Detect OS
detect_os() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
    else
        echo -e "${RED}❌ Cannot detect OS${NC}"
        exit 1
    fi
}

# Setup for Ubuntu
setup_ubuntu() {
    echo -e "${YELLOW}🔧 Setting up Ubuntu environment...${NC}"
    
    # Update system
    sudo apt update
    
    # Install ROS 2 Humble
    if ! command -v ros2 &> /dev/null; then
        echo -e "${YELLOW}📦 Installing ROS 2 Humble...${NC}"
        sudo apt install software-properties-common -y
        sudo add-apt-repository universe -y
        sudo apt update
        
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        sudo apt update
        sudo apt install ros-humble-desktop -y
        sudo apt install python3-rosdep2 -y
        sudo rosdep init || true
        rosdep update
    fi
    
    # Install MAVROS
    sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
    
    # Install dependencies
    sudo apt install python3-pip python3-colcon-common-extensions -y
    pip3 install -r requirements.txt
    
    # Install geographic datasets
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    
    # Source ROS 2
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    
    echo -e "${GREEN}✅ Ubuntu setup complete${NC}"
}

# Setup for Arch Linux
setup_arch() {
    echo -e "${YELLOW}🔧 Setting up Arch Linux environment...${NC}"
    
    # Install ROS 2 from AUR
    if ! command -v ros2 &> /dev/null; then
        echo -e "${YELLOW}📦 Installing ROS 2 Humble from AUR...${NC}"
        
        # Install yay if not present
        if ! command -v yay &> /dev/null; then
            sudo pacman -S --needed git base-devel -y
            git clone https://aur.archlinux.org/yay.git
            cd yay
            makepkg -si --noconfirm
            cd ..
            rm -rf yay
        fi
        
        # Install ROS 2
        yay -S ros-humble-desktop --noconfirm
        yay -S python-rosdep --noconfirm
        sudo rosdep init || true
        rosdep update
    fi
    
    # Install MAVROS
    yay -S ros-humble-mavros ros-humble-mavros-extras --noconfirm
    
    # Install Python dependencies
    sudo pacman -S python-pip --noconfirm
    pip3 install -r requirements.txt
    
    # Install geographic datasets
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    
    # Source ROS 2
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    
    echo -e "${GREEN}✅ Arch Linux setup complete${NC}"
}

# Setup Docker environment
setup_docker() {
    echo -e "${YELLOW}🐳 Setting up Docker environment...${NC}"
    
    # Install Docker
    if ! command -v docker &> /dev/null; then
        if [ "$OS" = "ubuntu" ]; then
            sudo apt install docker.io -y
        elif [ "$OS" = "arch" ]; then
            sudo pacman -S docker -y
        fi
        
        sudo systemctl enable docker
        sudo systemctl start docker
        sudo usermod -aG docker $USER
        
        echo -e "${YELLOW}⚠️  Please logout and login again for Docker group changes${NC}"
    fi
    
    # Build Docker image
    ./docker_runner.sh build
    
    echo -e "${GREEN}✅ Docker setup complete${NC}"
}

# Build workspace
build_workspace() {
    echo -e "${YELLOW}🔨 Building ROS 2 workspace...${NC}"
    
    cd ../../
    source /opt/ros/humble/setup.bash
    colcon build --packages-select drone_mvp
    source install/setup.bash
    
    echo -e "${GREEN}✅ Workspace built successfully${NC}"
}

# Show help
show_help() {
    echo "Usage: $0 [platform]"
    echo ""
    echo "Platforms:"
    echo "  ubuntu    - Setup for Ubuntu (native)"
    echo "  arch      - Setup for Arch Linux (native)"
    echo "  docker    - Setup Docker environment"
    echo "  build     - Build ROS 2 workspace only"
    echo ""
    echo "Example:"
    echo "  $0 ubuntu   # Setup native Ubuntu"
    echo "  $0 docker   # Setup Docker"
}

# Main execution
detect_os

case "${1:-help}" in
    ubuntu)
        if [ "$OS" != "ubuntu" ]; then
            echo -e "${RED}❌ This is not Ubuntu system${NC}"
            exit 1
        fi
        setup_ubuntu
        build_workspace
        ;;
    arch)
        if [ "$OS" != "arch" ]; then
            echo -e "${RED}❌ This is not Arch Linux system${NC}"
            exit 1
        fi
        setup_arch
        build_workspace
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

echo -e "${GREEN}🎉 Setup complete! You can now use:${NC}"
echo -e "  ${BLUE}./kaertei_master.sh${NC} - Main control interface"
echo -e "  ${BLUE}./quick_launcher.sh${NC} - Quick launcher"
echo -e "  ${BLUE}./docker_runner.sh${NC} - Docker operations"
