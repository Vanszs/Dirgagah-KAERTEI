#!/bin/bash
# KAERTEI 2025 FAIO - Ubuntu 22.04 LTS Complete Setup Script
# ==========================================================
# Zero to Competition Ready Setup for Ubuntu 22.04 LTS

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
}

success() {
    echo -e "${GREEN}✅ $1${NC}"
}

warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

error() {
    echo -e "${RED}❌ $1${NC}"
}

# Check if running on Ubuntu 22.04
check_ubuntu_version() {
    log "Checking Ubuntu version..."
    
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [[ "$VERSION_ID" == "22.04" ]]; then
            success "Ubuntu 22.04 LTS detected"
            return 0
        else
            warning "Not Ubuntu 22.04 LTS (detected: $PRETTY_NAME)"
            warning "This script is optimized for Ubuntu 22.04 LTS"
            read -p "Continue anyway? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
    else
        error "Cannot detect OS version"
        exit 1
    fi
}

# Update system packages
update_system() {
    log "Updating system packages..."
    
    sudo apt update
    sudo apt upgrade -y
    
    success "System packages updated"
}

# Install system dependencies
install_system_deps() {
    log "Installing system dependencies..."
    
    sudo apt install -y \
        curl wget git \
        build-essential cmake \
        python3 python3-pip python3-dev python3-venv \
        software-properties-common \
        lsb-release gnupg2 \
        ca-certificates \
        apt-transport-https \
        v4l-utils usbutils \
        can-utils i2c-tools \
        python3-serial python3-usb \
        nano vim htop tree \
        net-tools wireless-tools \
        ssh rsync \
        unzip zip p7zip-full \
        ffmpeg \
        chrony  # Time synchronization for GPS
    
    success "System dependencies installed"
}

# Install ROS 2 Humble
install_ros2() {
    log "Installing ROS 2 Humble Hawksbill..."
    
    # Check if already installed
    if command -v ros2 >/dev/null 2>&1; then
        success "ROS 2 already installed"
        return 0
    fi
    
    # Add ROS 2 apt repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    
    # Install ROS 2 packages
    sudo apt install -y \
        ros-humble-desktop \
        ros-humble-ros-base \
        python3-rosdep2 \
        python3-colcon-common-extensions \
        ros-dev-tools \
        python3-argcomplete \
        ros-humble-joint-state-publisher \
        ros-humble-robot-state-publisher \
        ros-humble-rviz2 \
        ros-humble-rqt*
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Add ROS 2 sourcing to bashrc
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "# ROS 2 Humble" >> ~/.bashrc
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        success "Added ROS 2 to bashrc"
    fi
    
    success "ROS 2 Humble installed successfully"
}

# Install MAVROS
install_mavros() {
    log "Installing MAVROS..."
    
    # Source ROS 2 environment
    source /opt/ros/humble/setup.bash
    
    # Check if already installed
    if ros2 pkg list 2>/dev/null | grep -q mavros; then
        success "MAVROS already installed"
    else
        # Install MAVROS packages
        sudo apt install -y \
            ros-humble-mavros \
            ros-humble-mavros-extras \
            ros-humble-mavros-msgs
        
        success "MAVROS packages installed"
    fi
    
    # Install GeographicLib datasets (critical for GPS functionality)
    log "Installing GeographicLib datasets..."
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    
    success "MAVROS installation completed"
}

# Install Python dependencies
install_python_deps() {
    log "Installing Python dependencies..."
    
    # Upgrade pip
    python3 -m pip install --user --upgrade pip
    
    # Install from requirements.txt
    if [ -f "requirements.txt" ]; then
        python3 -m pip install --user -r requirements.txt
        success "Python dependencies from requirements.txt installed"
    else
        warning "requirements.txt not found, installing essential packages..."
        
        # Install essential packages manually
        python3 -m pip install --user \
            pymavlink>=2.4.37 \
            opencv-python>=4.5.4 \
            numpy>=1.21.5 \
            ultralytics>=8.0.196 \
            torch>=1.13.1 \
            torchvision>=0.14.1 \
            scipy>=1.8.0 \
            matplotlib>=3.5.1 \
            PyYAML>=6.0 \
            pyserial>=3.5 \
            psutil>=5.9.0 \
            netifaces>=0.11.0
        
        success "Essential Python packages installed"
    fi
}

# Configure hardware permissions
setup_hardware_permissions() {
    log "Setting up hardware permissions..."
    
    # Add user to necessary groups
    sudo usermod -a -G dialout,video,tty,uucp,gpio,i2c,spi $USER
    
    # Create udev rules for hardware access
    sudo tee /etc/udev/rules.d/99-kaertei-hardware.rules > /dev/null <<EOF
# KAERTEI 2025 FAIO Hardware Rules
# Pixhawk/PX4 autopilot
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5740", GROUP="dialout", MODE="0666"

# Generic USB-Serial adapters
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"

# Camera devices
SUBSYSTEM=="video4linux", GROUP="video", MODE="0666"

# GPIO and I2C
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0666"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0666"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    success "Hardware permissions configured"
    warning "Please logout and login again for group changes to take effect"
}

# Build ROS 2 workspace
build_workspace() {
    log "Building ROS 2 workspace..."
    
    # Navigate to workspace directory
    WORKSPACE_DIR="/home/vanszs/Documents/ros2/ros2_ws"
    
    if [ ! -d "$WORKSPACE_DIR" ]; then
        error "Workspace directory not found: $WORKSPACE_DIR"
        return 1
    fi
    
    cd "$WORKSPACE_DIR"
    
    # Source ROS 2 environment
    source /opt/ros/humble/setup.bash
    
    # Install dependencies
    rosdep install --from-paths src --ignore-src -r -y
    
    # Clean previous builds
    rm -rf build/ install/ log/
    
    # Build the workspace
    colcon build --packages-select drone_mvp --cmake-clean-first --parallel-workers $(nproc)
    
    # Add workspace sourcing to bashrc
    if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
        echo "# KAERTEI 2025 Workspace" >> ~/.bashrc
        echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
        success "Added workspace to bashrc"
    fi
    
    # Source the workspace
    source install/setup.bash
    
    success "Workspace built successfully"
}

# Install Just command runner
install_just() {
    log "Installing Just command runner..."
    
    if command -v just >/dev/null 2>&1; then
        success "Just already installed"
        return 0
    fi
    
    # Install Just
    curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin
    
    # Add ~/bin to PATH if not already there
    if ! grep -q 'export PATH="$HOME/bin:$PATH"' ~/.bashrc; then
        echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
    fi
    
    export PATH="$HOME/bin:$PATH"
    
    success "Just command runner installed"
}

# Configure system services
configure_services() {
    log "Configuring system services..."
    
    # Enable time synchronization (important for GPS)
    sudo systemctl enable chrony
    sudo systemctl start chrony
    
    # Configure swappiness for better performance
    echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
    
    success "System services configured"
}

# Run system tests
run_tests() {
    log "Running system validation tests..."
    
    # Test ROS 2
    if command -v ros2 >/dev/null 2>&1; then
        source /opt/ros/humble/setup.bash
        ros2 --version
        success "ROS 2 test passed"
    else
        error "ROS 2 test failed"
    fi
    
    # Test MAVROS
    if ros2 pkg list 2>/dev/null | grep -q mavros; then
        success "MAVROS test passed"
    else
        error "MAVROS test failed"
    fi
    
    # Test Python packages
    python3 -c "import cv2; print('✅ OpenCV OK')" 2>/dev/null || echo "❌ OpenCV test failed"
    python3 -c "import numpy; print('✅ NumPy OK')" 2>/dev/null || echo "❌ NumPy test failed"
    python3 -c "import pymavlink; print('✅ PyMAVLink OK')" 2>/dev/null || echo "❌ PyMAVLink test failed"
    
    # Check hardware devices
    if ls /dev/tty{USB,ACM}* 2>/dev/null; then
        success "Hardware devices detected"
        ls -la /dev/tty{USB,ACM}*
    else
        warning "No hardware devices detected (normal if no hardware connected)"
    fi
    
    # Check camera devices
    if ls /dev/video* 2>/dev/null; then
        success "Camera devices detected"
        ls -la /dev/video*
    else
        warning "No camera devices detected"
    fi
    
    success "System validation completed"
}

# Main installation flow
main() {
    echo "🏆 KAERTEI 2025 FAIO - Ubuntu 22.04 LTS Setup"
    echo "============================================="
    echo ""
    echo "This script will install:"
    echo "• ROS 2 Humble Hawksbill"
    echo "• MAVROS with GeographicLib"
    echo "• Python dependencies (OpenCV, PyMAVLink, YOLOv8)"
    echo "• Hardware permissions and udev rules"
    echo "• Just command runner"
    echo "• Competition workspace build"
    echo ""
    
    read -p "Continue with installation? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Installation cancelled."
        exit 0
    fi
    
    echo ""
    log "Starting KAERTEI 2025 setup process..."
    
    # Run installation steps
    check_ubuntu_version
    update_system
    install_system_deps
    install_ros2
    install_mavros
    install_python_deps
    setup_hardware_permissions
    install_just
    configure_services
    build_workspace
    run_tests
    
    echo ""
    echo "🎉 KAERTEI 2025 FAIO Setup Complete!"
    echo "===================================="
    echo ""
    success "All components installed successfully"
    echo ""
    echo "📋 Next Steps:"
    echo "1. Logout and login again (for group permissions)"
    echo "2. Connect your hardware (Pixhawk, cameras)"
    echo "3. Run: just test-all"
    echo "4. Run: just mission-debug"
    echo ""
    echo "🔧 Quick Commands:"
    echo "• just help           # Show all available commands"
    echo "• just status         # Check system status"
    echo "• just doctor         # Diagnose any issues"
    echo "• just mission-debug  # Start competition mission"
    echo ""
    echo "🏆 Ready for KAERTEI 2025 FAIO Competition!"
    
    warning "Please REBOOT or logout/login for all changes to take effect"
}

# Run main function
main "$@"
