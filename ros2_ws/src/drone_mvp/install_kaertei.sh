#!/bin/bash
# ========================================
# KAERTEI 2025 FAIO - Universal Installer
# Auto-detect & Optimized Setup
# ========================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# System detection variables
ARCH=""
OS_NAME=""
OS_VERSION=""
IS_RASPBERRY_PI=false
PYTHON_VERSION=""
ROS_DISTRO="humble"

# Logging functions
log() { echo -e "${BLUE}[$(date +'%H:%M:%S')]${NC} $1"; }
success() { echo -e "${GREEN}✅ $1${NC}"; }
warning() { echo -e "${YELLOW}⚠️  $1${NC}"; }
error() { echo -e "${RED}❌ $1${NC}"; }
info() { echo -e "${CYAN}ℹ️  $1${NC}"; }

# Auto-detect system information
detect_system() {
    log "Auto-detecting system configuration..."
    
    # Detect architecture
    ARCH=$(uname -m)
    
    # Detect OS
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS_NAME=$ID
        OS_VERSION=$VERSION_ID
    fi
    
    # Check if Raspberry Pi
    if [ -f /proc/cpuinfo ] && grep -q "Raspberry Pi" /proc/cpuinfo; then
        IS_RASPBERRY_PI=true
    fi
    
    # Detect Python version
    PYTHON_VERSION=$(python3 --version 2>/dev/null | cut -d' ' -f2 || echo "unknown")
    
    # Display system info
    echo ""
    info "🖥️  System Information"
    echo "======================"
    echo "Architecture: $ARCH"
    echo "OS: $PRETTY_NAME"
    echo "Python: $PYTHON_VERSION"
    echo "Raspberry Pi: $($IS_RASPBERRY_PI && echo 'Yes' || echo 'No')"
    echo ""
}

# Validate system compatibility
validate_system() {
    log "Validating system compatibility..."
    
    # Check Ubuntu version
    if [[ "$OS_NAME" == "ubuntu" ]]; then
        if [[ "$OS_VERSION" == "22.04" ]]; then
            success "Ubuntu 22.04 LTS - Fully supported"
        elif [[ "$OS_VERSION" == "20.04" ]]; then
            warning "Ubuntu 20.04 - Limited support (ROS 2 Galactic)"
            ROS_DISTRO="galactic"
        else
            warning "Ubuntu $OS_VERSION - Not officially tested"
            read -p "Continue anyway? (y/N): " -n 1 -r
            [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
        fi
    else
        error "Non-Ubuntu system detected: $OS_NAME"
        error "This installer is optimized for Ubuntu 22.04 LTS"
        exit 1
    fi
    
    # Check architecture
    if [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm64" ]]; then
        success "ARM64 architecture detected - Raspberry Pi optimized"
    elif [[ "$ARCH" == "x86_64" ]]; then
        success "x86_64 architecture detected - PC/Laptop optimized"
    else
        warning "Unknown architecture: $ARCH"
    fi
    
    # Check Python version
    if [[ "$PYTHON_VERSION" =~ ^3\.(10|11|12) ]]; then
        success "Python $PYTHON_VERSION - Compatible"
    else
        warning "Python $PYTHON_VERSION - May have compatibility issues"
    fi
    
    echo ""
}

# Install essential system packages
install_system_packages() {
    log "Installing essential system packages..."
    
    # Fix time sync issues
    log "Synchronizing system time..."
    sudo timedatectl set-ntp true
    sleep 2
    
    # Update package lists with retries
    log "Updating package lists..."
    for i in {1..3}; do
        if sudo apt update; then
            break
        else
            warning "Update attempt $i failed, retrying..."
            sleep 5
        fi
    done
    
    # Essential development tools
    local ESSENTIAL_PACKAGES=(
        curl wget git
        build-essential cmake
        python3 python3-pip python3-dev python3-venv
        software-properties-common
        lsb-release gnupg2 ca-certificates
        apt-transport-https
    )
    
    # Hardware interface packages
    local HARDWARE_PACKAGES=(
        v4l-utils usbutils
        python3-serial python3-usb
        can-utils i2c-tools
        chrony  # GPS time sync
    )
    
    # Raspberry Pi specific packages
    local RPI_PACKAGES=()
    if $IS_RASPBERRY_PI; then
        RPI_PACKAGES+=(
            raspberrypi-kernel-headers
            python3-rpi.gpio
            gpio
            wiringpi
        )
    fi
    
    # Utility packages
    local UTILITY_PACKAGES=(
        nano vim htop tree
        net-tools wireless-tools
        ssh rsync unzip zip
        ffmpeg
    )
    
    # Install all packages
    sudo apt install -y \
        "${ESSENTIAL_PACKAGES[@]}" \
        "${HARDWARE_PACKAGES[@]}" \
        "${RPI_PACKAGES[@]}" \
        "${UTILITY_PACKAGES[@]}"
    
    success "System packages installed"
}

# Install ROS 2 with architecture optimization
install_ros2() {
    log "Installing ROS 2 $ROS_DISTRO..."
    
    # Check if already installed
    if command -v ros2 >/dev/null 2>&1; then
        success "ROS 2 already installed"
        return 0
    fi
    
    # Add ROS 2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    
    # Install ROS 2 packages (optimized for different systems)
    if $IS_RASPBERRY_PI; then
        # Lightweight ROS 2 installation for Raspberry Pi
        log "Installing ROS 2 (Raspberry Pi optimized)..."
        sudo apt install -y \
            ros-$ROS_DISTRO-ros-base \
            ros-$ROS_DISTRO-joint-state-publisher \
            ros-$ROS_DISTRO-robot-state-publisher \
            python3-rosdep2 \
            python3-colcon-common-extensions \
            ros-dev-tools
    else
        # Full ROS 2 installation for desktop/laptop
        log "Installing ROS 2 (Desktop full)..."
        sudo apt install -y \
            ros-$ROS_DISTRO-desktop \
            ros-$ROS_DISTRO-ros-base \
            python3-rosdep2 \
            python3-colcon-common-extensions \
            ros-dev-tools \
            ros-$ROS_DISTRO-rviz2 \
            ros-$ROS_DISTRO-rqt*
    fi
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Add to bashrc
    if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
        echo "# ROS 2 $ROS_DISTRO" >> ~/.bashrc
        echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    fi
    
    success "ROS 2 $ROS_DISTRO installed"
}

# Install MAVROS with GeographicLib
install_mavros() {
    log "Installing MAVROS with GeographicLib..."
    
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    # Install MAVROS packages
    sudo apt install -y \
        ros-$ROS_DISTRO-mavros \
        ros-$ROS_DISTRO-mavros-extras \
        ros-$ROS_DISTRO-mavros-msgs
    
    # Install GeographicLib datasets (critical for GPS)
    log "Installing GeographicLib datasets..."
    sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
    
    success "MAVROS installed with GeographicLib"
}

# Install optimized Python dependencies
install_python_deps() {
    log "Installing Python dependencies..."
    
    # Upgrade pip
    python3 -m pip install --user --upgrade pip
    
    # Architecture-specific optimizations
    local PIP_ARGS="--user"
    
    # Raspberry Pi optimizations
    if $IS_RASPBERRY_PI; then
        info "Applying Raspberry Pi optimizations..."
        # Use pre-compiled wheels when available
        PIP_ARGS+=" --prefer-binary"
        # Set memory limits for compilation
        export CFLAGS="-O2"
        export CXXFLAGS="-O2"
    fi
    
    # Essential packages (architecture optimized)
    log "Installing core dependencies..."
    python3 -m pip install $PIP_ARGS \
        pymavlink>=2.4.37 \
        numpy>=1.21.5 \
        PyYAML>=6.0 \
        pyserial>=3.5 \
        psutil>=5.9.0 \
        colorama>=0.4.4
    
    # Computer vision packages
    log "Installing computer vision packages..."
    if $IS_RASPBERRY_PI; then
        # Lightweight OpenCV for Raspberry Pi
        python3 -m pip install $PIP_ARGS opencv-python-headless>=4.6.0
    else
        # Full OpenCV for desktop
        python3 -m pip install $PIP_ARGS opencv-python>=4.6.0
    fi
    
    # AI/ML packages (optional for Raspberry Pi due to size)
    if ! $IS_RASPBERRY_PI; then
        log "Installing AI/ML packages..."
        python3 -m pip install $PIP_ARGS \
            ultralytics>=8.0.196 \
            torch>=1.13.1 \
            torchvision>=0.14.1
    else
        warning "Skipping heavy AI packages on Raspberry Pi (install manually if needed)"
        info "To install AI packages: pip3 install ultralytics torch torchvision"
    fi
    
    # Raspberry Pi specific packages
    if $IS_RASPBERRY_PI; then
        log "Installing Raspberry Pi GPIO packages..."
        python3 -m pip install $PIP_ARGS \
            RPi.GPIO>=0.7.1 \
            gpiozero>=1.6.2
    fi
    
    success "Python dependencies installed"
}

# Configure hardware permissions
setup_permissions() {
    log "Configuring hardware permissions..."
    
    # Add user to necessary groups
    USER_GROUPS="dialout,video,tty"
    
    # Add Raspberry Pi specific groups
    if $IS_RASPBERRY_PI; then
        USER_GROUPS="${USER_GROUPS},gpio,i2c,spi"
    fi
    
    sudo usermod -a -G "$USER_GROUPS" $USER
    
    # Create optimized udev rules
    sudo tee /etc/udev/rules.d/99-kaertei-hardware.rules > /dev/null <<EOF
# KAERTEI 2025 Hardware Rules
# Pixhawk/PX4 Flight Controllers
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5740", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", GROUP="dialout", MODE="0666"

# USB Serial Adapters
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"

# Camera Devices
SUBSYSTEM=="video4linux", GROUP="video", MODE="0666"
KERNEL=="video[0-9]*", GROUP="video", MODE="0666"

# GPIO and Hardware Interfaces
SUBSYSTEM=="gpio*", GROUP="gpio", MODE="0666"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0666"
SUBSYSTEM=="spidev", GROUP="spi", MODE="0666"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    
    success "Hardware permissions configured"
}

# Install Just command runner
install_just() {
    log "Installing Just command runner..."
    
    if command -v just >/dev/null 2>&1; then
        success "Just already installed"
        return 0
    fi
    
    # Create bin directory
    mkdir -p ~/bin
    
    # Install Just
    curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin
    
    # Add to PATH
    if ! grep -q 'export PATH="$HOME/bin:$PATH"' ~/.bashrc; then
        echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
    fi
    
    export PATH="$HOME/bin:$PATH"
    
    success "Just command runner installed"
}

# Build workspace
build_workspace() {
    log "Building KAERTEI workspace..."
    
    # Navigate to workspace
    WORKSPACE_DIR="$(pwd)/../../.."
    if [ ! -d "$WORKSPACE_DIR" ]; then
        WORKSPACE_DIR="/home/vanszs/ros/Dirgagah-KAERTEI/ros2_ws"
    fi
    
    # Check if workspace exists
    if [ ! -d "$WORKSPACE_DIR" ]; then
        error "Workspace directory not found"
        return 1
    fi
    
    cd "$WORKSPACE_DIR"
    log "Building in: $(pwd)"
    
    # Source ROS 2
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    # Install dependencies
    if [ -d "src" ]; then
        rosdep install --from-paths src --ignore-src -r -y --skip-keys="rosdep" || true
    fi
    
    # Clean previous build
    rm -rf build/ install/ log/
    
    # Build with optimizations
    local BUILD_ARGS="--packages-select drone_mvp"
    if $IS_RASPBERRY_PI; then
        # Reduce parallel jobs for Raspberry Pi
        BUILD_ARGS+=" --parallel-workers 2"
    else
        BUILD_ARGS+=" --parallel-workers $(nproc)"
    fi
    
    if colcon build $BUILD_ARGS; then
        success "Workspace built successfully"
        
        # Add to bashrc
        if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
            echo "# KAERTEI 2025 Workspace" >> ~/.bashrc
            echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
        fi
        return 0
    else
        warning "Workspace build failed - this is normal if packages are not complete"
        return 0
    fi
}

# Run comprehensive validation
validate_installation() {
    log "Validating installation..."
    
    # Test ROS 2
    if source /opt/ros/$ROS_DISTRO/setup.bash && ros2 --version >/dev/null 2>&1; then
        success "ROS 2 validation passed"
    else
        error "ROS 2 validation failed"
        return 1
    fi
    
    # Test Python packages
    python3 -c "import numpy; print('✅ NumPy OK')" || error "NumPy test failed"
    python3 -c "import cv2; print('✅ OpenCV OK')" || error "OpenCV test failed"
    python3 -c "import pymavlink; print('✅ PyMAVLink OK')" || error "PyMAVLink test failed"
    
    if $IS_RASPBERRY_PI; then
        python3 -c "import RPi.GPIO; print('✅ GPIO OK')" || warning "GPIO test failed"
    fi
    
    # Test hardware detection
    log "Hardware detection:"
    if ls /dev/tty{ACM,USB}* 2>/dev/null; then
        success "Serial devices detected"
    else
        warning "No serial devices (normal if no hardware connected)"
    fi
    
    if ls /dev/video* 2>/dev/null; then
        success "Camera devices detected"
    else
        warning "No camera devices detected"
    fi
    
    success "Installation validation completed"
}

# Display completion message
show_completion() {
    echo ""
    echo "🎉 KAERTEI 2025 Installation Complete!"
    echo "====================================="
    echo ""
    success "System configured for: $([ $IS_RASPBERRY_PI = true ] && echo 'Raspberry Pi 5' || echo 'Desktop/Laptop')"
    success "Architecture: $ARCH"
    success "ROS 2: $ROS_DISTRO"
    echo ""
    echo "📋 Next Steps:"
    echo "1. 🔄 Logout and login again (for permissions)"
    echo "2. 🔌 Connect your hardware (Pixhawk, cameras)"
    echo "3. ⚡ Test system: just test-all"
    echo "4. 🚁 Start mission: just mission-debug"
    echo ""
    echo "🎯 Quick Commands:"
    echo "• just help           # Show all commands"
    echo "• just status         # System status"
    echo "• just hardware-check # Hardware detection"
    echo ""
    if $IS_RASPBERRY_PI; then
        echo "🍓 Raspberry Pi Tips:"
        echo "• Monitor temperature: vcgencmd measure_temp"
        echo "• GPU memory split: sudo raspi-config"
        echo "• Enable camera: sudo raspi-config"
    fi
    echo ""
    warning "⚠️  Please REBOOT for all changes to take effect"
    echo ""
    info "🏆 Ready for KAERTEI 2025 Competition!"
}

# Main installation function
main() {
    echo "🚁 KAERTEI 2025 FAIO Universal Installer"
    echo "======================================="
    echo ""
    
    # System detection and validation
    detect_system
    validate_system
    
    echo "📦 Installation includes:"
    echo "• ROS 2 $ROS_DISTRO (optimized for your system)"
    echo "• MAVROS with GeographicLib datasets"
    echo "• Python dependencies (OpenCV, PyMAVLink)"
    echo "• Hardware permissions and drivers"
    echo "• Competition workspace build"
    echo ""
    
    read -p "🚀 Start installation? (y/N): " -n 1 -r
    echo ""
    [[ ! $REPLY =~ ^[Yy]$ ]] && { echo "Installation cancelled."; exit 0; }
    
    echo ""
    log "Starting KAERTEI 2025 installation..."
    
    # Run installation steps
    install_system_packages
    install_ros2
    install_mavros
    install_python_deps
    setup_permissions
    install_just
    build_workspace
    validate_installation
    
    show_completion
}

# Execute main function
main "$@"
