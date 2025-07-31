#!/bin/bash

# KAERTEI 2025 FAIO Drone Setup Script
# This script sets up the complete drone system

set -e

echo "============================================"
echo "KAERTEI 2025 FAIO Drone System Setup"
echo "============================================"

# Check if we're in the correct directory
if [ ! -f "package.xml" ]; then
    echo "Error: Please run this script from the drone_mvp package directory"
    exit 1
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS 2 is installed
print_status "Checking ROS 2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS 2 not found. Please install ROS 2 Humble first."
    exit 1
fi

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_status "ROS 2 Humble sourced"
else
    print_error "ROS 2 Humble setup.bash not found"
    exit 1
fi

# Check Python version
print_status "Checking Python version..."
python_version=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
if [ "$(printf '%s\n' "3.8" "$python_version" | sort -V | head -n1)" != "3.8" ]; then
    print_error "Python 3.8 or higher required. Found: $python_version"
    exit 1
fi

# Install Python dependencies
print_status "Installing Python dependencies..."
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
    if [ $? -eq 0 ]; then
        print_status "Python dependencies installed successfully"
    else
        print_warning "Some Python dependencies failed to install. Please check manually."
    fi
else
    print_warning "requirements.txt not found"
fi

# Install additional ROS 2 packages
print_status "Installing additional ROS 2 packages..."
sudo apt update

# Essential packages
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rqt-image-view \
    ros-humble-rqt-console \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-robot-state-publisher

# MAVROS packages for ArduPilot communication
print_status "Installing MAVROS packages..."
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-mavros-extras

# Install MAVROS geographic data
print_status "Installing MAVROS geographic data..."
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Hardware-specific packages (Raspberry Pi)
if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    print_status "Raspberry Pi detected, installing GPIO libraries..."
    pip3 install RPi.GPIO adafruit-circuitpython-vl53l0x adafruit-blinka
fi

# Build the workspace
print_status "Building the workspace..."
cd ../../..  # Go to workspace root
colcon build --packages-select drone_mvp --symlink-install

if [ $? -eq 0 ]; then
    print_status "Package built successfully"
else
    print_error "Build failed. Please check the error messages above."
    exit 1
fi

# Source the workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    print_status "Workspace sourced"
fi

# Create desktop shortcut (optional)
create_desktop_shortcut() {
    local desktop_file="$HOME/Desktop/KAERTEI_Drone_Launch.desktop"
    cat > "$desktop_file" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=KAERTEI 2025 Drone System
Comment=Launch KAERTEI 2025 FAIO Drone System
Exec=gnome-terminal -- bash -c "cd $(pwd) && source install/setup.bash && ros2 launch drone_mvp drone.launch.py; exec bash"
Icon=applications-engineering
Terminal=true
Categories=Development;
EOF
    chmod +x "$desktop_file"
    print_status "Desktop shortcut created: $desktop_file"
}

# Ask user if they want desktop shortcut
read -p "Create desktop shortcut for easy launching? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    create_desktop_shortcut
fi

# Create launch script
print_status "Creating launch scripts..."
cat > "launch_drone.sh" << 'EOF'
#!/bin/bash
# Quick launch script for KAERTEI 2025 Drone System

set -e

# Colors
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}Launching KAERTEI 2025 FAIO Drone System...${NC}"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: Workspace not built. Please run setup.sh first."
    exit 1
fi

# Launch the system
ros2 launch drone_mvp drone.launch.py
EOF

chmod +x "launch_drone.sh"

# Create monitoring script
cat > "monitor_drone.sh" << 'EOF'
#!/bin/bash
# Monitoring script for KAERTEI 2025 Drone System

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Available monitoring commands:"
echo "1. ros2 topic list                    - List all topics"
echo "2. ros2 topic echo /mission/state     - Monitor mission state"
echo "3. ros2 topic echo /sensors/status    - Monitor sensor status"
echo "4. ros2 topic echo /gps/moving_status - Monitor GPS status"
echo "5. rqt_console                        - Open log console"
echo "6. rqt_graph                          - View node graph"

echo ""
echo "Starting rqt_console for monitoring..."
rqt_console
EOF

chmod +x "monitor_drone.sh"

print_status "Setup completed successfully!"
print_status "Created scripts:"
print_status "  - launch_drone.sh: Quick launch script"
print_status "  - monitor_drone.sh: Monitoring tools"

echo ""
echo "============================================"
echo -e "${GREEN}Setup Summary:${NC}"
echo "✓ ROS 2 environment verified"
echo "✓ Python dependencies installed"
echo "✓ Package built successfully"
echo "✓ Launch scripts created"
echo ""
echo -e "${GREEN}To launch the drone system:${NC}"
echo "  ./launch_drone.sh"
echo ""
echo -e "${GREEN}To monitor the system:${NC}"
echo "  ./monitor_drone.sh"
echo ""
echo -e "${GREEN}Manual launch command:${NC}"
echo "  ros2 launch drone_mvp drone.launch.py"
echo "============================================"
