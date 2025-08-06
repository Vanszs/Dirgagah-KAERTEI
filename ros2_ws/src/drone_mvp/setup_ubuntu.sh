#!/bin/bash
# KAERTEI 2025 FAIO - Ubuntu 22.04 Competition Setup
# Zero to competition ready in one command!

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}🏆 KAERTEI 2025 FAIO - Ubuntu 22.04 Competition Setup${NC}"
echo -e "${BLUE}====================================================${NC}"
echo ""

# Print with colors
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

# Check if running as root
check_root() {
    if [ "$EUID" -eq 0 ]; then
        print_error "❌ Do not run this script as root"
        echo "   Run as normal user - script will ask for sudo when needed"
        exit 1
    fi
}

# Verify Ubuntu 22.04
verify_ubuntu() {
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "22.04" ]]; then
            print_success "✅ Ubuntu 22.04 LTS detected - Perfect!"
            return 0
        elif [[ "$ID" == "ubuntu" ]]; then
            print_warning "⚠️  Ubuntu $VERSION_ID detected - Ubuntu 22.04 LTS recommended"
            print_warning "   Some features may not work optimally."
            return 0
        else
            print_error "❌ This script is optimized for Ubuntu 22.04 LTS only"
            print_error "   Detected: $ID $VERSION_ID"
            exit 1
        fi
    else
        print_error "❌ Cannot detect OS - /etc/os-release not found"
        exit 1
    fi
}

# Setup ROS 2 Humble for Ubuntu 22.04
setup_ros2() {
    if ! command -v ros2 &> /dev/null; then
        print_step "🤖 Installing ROS 2 Humble Hawksbill..."
        
        # Add ROS 2 repository
        sudo apt install software-properties-common curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        # Install ROS 2
        sudo apt update
        sudo apt install ros-humble-desktop -y
        sudo apt install python3-rosdep2 -y
        
        # Initialize rosdep
        if ! rosdep --version &> /dev/null || [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
            sudo rosdep init || true
        fi
        rosdep update
        
        print_success "✅ ROS 2 Humble installed successfully!"
    else
        print_success "✅ ROS 2 already installed"
    fi
}

# Setup MAVROS
setup_mavros() {
    print_step "📡 Installing MAVROS..."
    sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
    
    # Install GeographicLib datasets (critical for GPS)
    print_info "🗺️ Installing geographic datasets..."
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    
    print_success "✅ MAVROS installed successfully!"
}

# Setup Python environment
setup_python() {
    print_step "🐍 Setting up Python environment..."
    
    # Install Python system packages
    sudo apt install -y \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        python3-opencv \
        python3-numpy \
        python3-yaml \
        python3-serial \
        python3-setuptools \
        python3-dev
    
    # Create and activate virtual environment if it doesn't exist
    if [ ! -d "ros2_env" ]; then
        print_info "Creating Python virtual environment..."
        python3 -m venv ros2_env
    fi
    
    # Install Python packages
    if [ -f "requirements.txt" ]; then
        print_info "Installing Python dependencies..."
        source ros2_env/bin/activate
        pip install --upgrade pip setuptools wheel
        pip install -r requirements.txt
        deactivate
    else
        print_info "Installing essential Python packages..."
        source ros2_env/bin/activate
        pip install --upgrade pip setuptools wheel
        pip install pymavlink opencv-python numpy ultralytics torch torchvision
        deactivate
    fi
    
    print_success "✅ Python environment ready!"
}

# Setup hardware permissions and udev rules
setup_hardware() {
    print_step "🔌 Setting up hardware access..."
    
    # Add user to necessary groups
    sudo usermod -a -G dialout,video,plugdev $USER
    
    # Install v4l-utils for camera
    sudo apt install -y v4l-utils
    
    # Create udev rules for PX4 and common flight controllers
    sudo tee /etc/udev/rules.d/50-px4.rules > /dev/null <<EOF
# PX4 Flight Controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="px4fmu", GROUP="dialout", MODE="0664"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5740", SYMLINK+="px4fmu", GROUP="dialout", MODE="0664"

# ArduPilot Flight Controllers
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ardupilot", GROUP="dialout", MODE="0664"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0032", SYMLINK+="ardupilot", GROUP="dialout", MODE="0664"

# Generic USB-to-Serial adapters
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0664"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", GROUP="dialout", MODE="0664"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_success "✅ Hardware permissions configured!"
}

# Setup development tools
setup_dev_tools() {
    print_step "🛠️ Installing development tools..."
    
    sudo apt install -y \
        git \
        curl \
        wget \
        nano \
        htop \
        tree \
        build-essential \
        cmake \
        gdb \
        valgrind \
        net-tools
    
    # Install Just if not present
    if ! command -v just &> /dev/null; then
        print_info "Installing Just command runner..."
        curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin
        echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
    fi
    
    print_success "✅ Development tools installed!"
}

# Setup shell environment
setup_environment() {
    print_step "🐚 Setting up shell environment..."
    
    # Auto-source ROS 2 in bashrc if not already present
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        print_info "Adding ROS 2 to bashrc..."
        echo "" >> ~/.bashrc
        echo "# ROS 2 Humble" >> ~/.bashrc
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    # Add workspace sourcing
    WORKSPACE_SETUP="/home/vanszs/Documents/ros2/ros2_ws/install/setup.bash"
    if ! grep -q "$WORKSPACE_SETUP" ~/.bashrc && [ -f "$WORKSPACE_SETUP" ]; then
        print_info "Adding workspace to bashrc..."
        echo "source $WORKSPACE_SETUP" >> ~/.bashrc
    fi
    
    # Add Python virtual environment
    if ! grep -q "ros2_env/bin/activate" ~/.bashrc && [ -d "ros2_env" ]; then
        echo "" >> ~/.bashrc
        echo "# KAERTEI 2025 Python Environment" >> ~/.bashrc
        echo "source /home/vanszs/Documents/ros2/ros2_env/bin/activate" >> ~/.bashrc
    fi
    
    # Set up useful aliases
    if ! grep -q "alias kaertei" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# KAERTEI 2025 Aliases" >> ~/.bashrc
        echo "alias kaertei='cd /home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp'" >> ~/.bashrc
        echo "alias mission='just mission-debug'" >> ~/.bashrc
        echo "alias mission-auto='just mission-auto'" >> ~/.bashrc
        echo "alias qgc='QGroundControl'" >> ~/.bashrc
    fi
    
    print_success "✅ Shell environment configured!"
}

# Post-setup verification
verify_setup() {
    print_step "🔍 Verifying installation..."
    
    # Check ROS 2
    if command -v ros2 >/dev/null 2>&1; then
        print_success "✅ ROS 2 Humble available"
    else
        print_error "❌ ROS 2 not found"
        return 1
    fi
    
    # Check MAVROS
    if dpkg -l | grep -q ros-humble-mavros; then
        print_success "✅ MAVROS installed"
    else
        print_error "❌ MAVROS not found"
        return 1
    fi
    
    # Check Python dependencies
    source ros2_env/bin/activate 2>/dev/null || true
    if python3 -c "import cv2, numpy as np, pymavlink" 2>/dev/null; then
        print_success "✅ Python dependencies OK"
    else
        print_warning "⚠️  Some Python dependencies missing"
    fi
    deactivate 2>/dev/null || true
    
    # Check hardware access
    if ls /dev/tty{USB,ACM}* 2>/dev/null >/dev/null; then
        print_success "✅ Hardware devices found"
    else
        print_warning "⚠️  No hardware devices (will use dummy mode)"
    fi
    
    print_success "✅ Verification complete!"
}

# Competition readiness check
competition_ready_check() {
    print_step "🏆 Competition readiness check..."
    
    # Essential competition components
    local ready=true
    
    # ROS 2 workspace
    if [ -d "ros2_ws/src/drone_mvp" ]; then
        print_success "✅ Drone MVP package found"
    else
        print_error "❌ Drone MVP package missing"
        ready=false
    fi
    
    # Mission scripts
    if [ -f "ros2_ws/src/drone_mvp/run_checkpoint_mission.sh" ]; then
        print_success "✅ Mission launcher available"
    else
        print_error "❌ Mission launcher missing"
        ready=false
    fi
    
    # Configuration
    if [ -f "ros2_ws/src/drone_mvp/config/hardware_config.conf" ]; then
        print_success "✅ Hardware configuration found"
    else
        print_error "❌ Hardware configuration missing"
        ready=false
    fi
    
    # Just command runner
    if command -v just >/dev/null 2>&1; then
        print_success "✅ Just command runner available"
    else
        print_warning "⚠️  Just not in PATH - add ~/bin to PATH"
    fi
    
    if $ready; then
        echo ""
        print_success "🎉 COMPETITION READY!"
        echo ""
        echo -e "${GREEN}Next steps:${NC}"
        echo "1. Restart terminal (or run: source ~/.bashrc)"
        echo "2. Navigate to project: cd /home/vanszs/Documents/ros2/ros2_ws/src/drone_mvv"
        echo "3. Test system: just test"
        echo "4. Run practice: just mission-debug"
        echo "5. Competition: just mission-auto"
        echo ""
        echo -e "${CYAN}Emergency commands:${NC}"
        echo "• just doctor        # Diagnose issues"
        echo "• just emergency     # Emergency procedures"
        echo "• just checklist     # Competition checklist"
    else
        print_error "❌ Setup incomplete - some components missing"
        echo "   Please check errors above and re-run setup"
    fi
}

# Main Ubuntu 22.04 setup function
main() {
    print_step "🐧 KAERTEI 2025 - Ubuntu 22.04 Competition Setup"
    echo
    
    # Initial checks
    check_root
    verify_ubuntu
    
    # Core setup
    print_info "📦 Updating system packages..."
    sudo apt update && sudo apt upgrade -y
    
    # Install essential packages first
    print_info "🔧 Installing essential packages..."
    sudo apt install -y curl wget gnupg lsb-release software-properties-common \
                       build-essential git python3-pip python3-venv cmake \
                       udev net-tools htop tree
    
    # Setup components
    setup_ros2
    setup_mavros
    setup_python
    setup_hardware
    setup_dev_tools
    setup_environment
    
    # Post-setup verification
    verify_setup
    
    # Final competition readiness check
    competition_ready_check
    
    echo ""
    print_success "🎉 Ubuntu 22.04 setup complete for KAERTEI 2025!"
    print_warning "⚠️  Please restart your terminal or run: source ~/.bashrc"
}

# Handle command line arguments
case "${1:-ubuntu}" in
    ubuntu)
        main
        ;;
    *)
        print_error "❌ Only Ubuntu 22.04 is supported"
        print_info "Usage: $0 [ubuntu]"
        exit 1
        ;;
esac
