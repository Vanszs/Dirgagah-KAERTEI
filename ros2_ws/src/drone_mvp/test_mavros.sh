#!/bin/bash

# MAVROS Connection Test Script for KAERTEI 2025 Drone System

echo "============================================"
echo "MAVROS Connection Test"
echo "============================================"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if MAVROS is installed
if ! ros2 pkg list | grep -q mavros; then
    print_error "MAVROS not installed. Please install with:"
    echo "sudo apt install ros-humble-mavros ros-humble-mavros-msgs ros-humble-mavros-extras"
    exit 1
fi

print_status "MAVROS package found"

# Check for USB devices
echo ""
echo "Available USB devices:"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB devices found"

# Default connection parameters
FCU_URL="${1:-serial:///dev/ttyUSB0:57600}"
GCS_URL="${2:-udp://@127.0.0.1:14550}"

echo ""
echo "Testing connection with:"
echo "  FCU URL: $FCU_URL"
echo "  GCS URL: $GCS_URL"
echo ""

# Function to test MAVROS
test_mavros() {
    echo "Starting MAVROS..."
    
    # Launch MAVROS in background
    ros2 launch mavros apm.launch fcu_url:="$FCU_URL" gcs_url:="$GCS_URL" &
    MAVROS_PID=$!
    
    echo "MAVROS PID: $MAVROS_PID"
    echo "Waiting 10 seconds for connection..."
    sleep 10
    
    # Test connection
    echo ""
    echo "Testing MAVROS topics..."
    
    # Check if MAVROS state topic is available
    if ros2 topic list | grep -q "/mavros/state"; then
        print_status "MAVROS state topic available"
        
        # Get current state
        echo "Current MAVROS state:"
        timeout 5 ros2 topic echo /mavros/state --once || print_warning "Could not read MAVROS state"
        
    else
        print_error "MAVROS state topic not available"
    fi
    
    # Check GPS topic
    if ros2 topic list | grep -q "/mavros/global_position/global"; then
        print_status "GPS topic available"
        
        echo "GPS status:"
        timeout 5 ros2 topic echo /mavros/global_position/global --once || print_warning "Could not read GPS data"
        
    else
        print_error "GPS topic not available"
    fi
    
    # List all MAVROS topics
    echo ""
    echo "All MAVROS topics:"
    ros2 topic list | grep mavros
    
    # Cleanup
    echo ""
    echo "Stopping MAVROS..."
    kill $MAVROS_PID 2>/dev/null
    wait $MAVROS_PID 2>/dev/null
}

# Function to test services
test_services() {
    echo ""
    echo "Testing MAVROS services..."
    
    # Launch MAVROS in background
    ros2 launch mavros apm.launch fcu_url:="$FCU_URL" gcs_url:="$GCS_URL" &
    MAVROS_PID=$!
    
    sleep 10
    
    # Check arming service
    if ros2 service list | grep -q "/mavros/cmd/arming"; then
        print_status "Arming service available"
    else
        print_error "Arming service not available"
    fi
    
    # Check set mode service
    if ros2 service list | grep -q "/mavros/set_mode"; then
        print_status "Set mode service available"
    else
        print_error "Set mode service not available"
    fi
    
    # List all MAVROS services
    echo ""
    echo "All MAVROS services:"
    ros2 service list | grep mavros
    
    # Cleanup
    kill $MAVROS_PID 2>/dev/null
    wait $MAVROS_PID 2>/dev/null
}

# Function to test our drone nodes
test_drone_nodes() {
    echo ""
    echo "Testing drone system integration..."
    
    # Source the workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    else
        print_error "Workspace not built. Please run: colcon build"
        return 1
    fi
    
    # Launch our system
    echo "Launching drone system..."
    ros2 launch drone_mvp drone.launch.py fcu_url:="$FCU_URL" gcs_url:="$GCS_URL" &
    SYSTEM_PID=$!
    
    sleep 15
    
    # Check if our nodes are running
    echo "Checking drone nodes..."
    
    if ros2 node list | grep -q "mission_node"; then
        print_status "Mission node running"
    else
        print_error "Mission node not running"
    fi
    
    if ros2 node list | grep -q "gps_monitor"; then
        print_status "GPS monitor running"
    else
        print_error "GPS monitor not running"
    fi
    
    if ros2 node list | grep -q "flight_mode_switcher"; then
        print_status "Flight mode switcher running"
    else
        print_error "Flight mode switcher not running"
    fi
    
    # Test topic communication
    echo ""
    echo "Testing topic communication..."
    
    if ros2 topic list | grep -q "/mission/state"; then
        print_status "Mission state topic available"
        timeout 3 ros2 topic echo /mission/state --once || print_warning "No mission state data"
    fi
    
    if ros2 topic list | grep -q "/gps/moving_status"; then
        print_status "GPS status topic available"
        timeout 3 ros2 topic echo /gps/moving_status --once || print_warning "No GPS status data"
    fi
    
    # Cleanup
    echo ""
    echo "Stopping drone system..."
    kill $SYSTEM_PID 2>/dev/null
    wait $SYSTEM_PID 2>/dev/null
}

# Main test sequence
echo "Starting tests..."

# Test 1: Basic MAVROS connection
echo ""
echo "=== Test 1: Basic MAVROS Connection ==="
test_mavros

# Test 2: MAVROS services
echo ""
echo "=== Test 2: MAVROS Services ==="
test_services

# Test 3: Drone system integration
echo ""
echo "=== Test 3: Drone System Integration ==="
test_drone_nodes

echo ""
echo "============================================"
print_status "Test completed!"
echo ""
echo "If any tests failed, check:"
echo "1. USB connection to Pixhawk"
echo "2. Correct baud rate (57600 or 115200)"
echo "3. ArduPilot firmware compatibility"
echo "4. USB port permissions: sudo chmod 666 /dev/ttyUSB0"
echo ""
echo "Manual MAVROS launch:"
echo "ros2 launch mavros apm.launch fcu_url:=\"$FCU_URL\" gcs_url:=\"$GCS_URL\""
echo "============================================"
