#!/bin/bash
# KAERTEI 2025 FAIO - Competition Startup Script (Ubuntu 22.04)
# =============================================================
# One-command startup and validation for competition day

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Print functions
print_header() { echo -e "${BLUE}$1${NC}"; }
print_success() { echo -e "${GREEN}✅ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠️  $1${NC}"; }
print_error() { echo -e "${RED}❌ $1${NC}"; }
print_info() { echo -e "${CYAN}ℹ️  $1${NC}"; }

print_header "🏆 KAERTEI 2025 FAIO - Competition Startup"
print_header "=========================================="
echo ""

# Step 1: Check if we're in the right directory
print_info "Checking project directory..."
if [ ! -f "Justfile" ]; then
    print_error "Not in the correct directory!"
    print_info "Please run this script from: kaertei_drone/"
    exit 1
fi
print_success "Project directory confirmed"

# Step 2: Check Ubuntu version
print_info "Checking Ubuntu version..."
if [ -f /etc/os-release ]; then
    source /etc/os-release
    if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "22.04" ]]; then
        print_success "Ubuntu 22.04 LTS detected"
    elif [[ "$ID" == "ubuntu" ]]; then
        print_warning "Ubuntu $VERSION_ID - 22.04 LTS recommended"
    else
        print_warning "Non-Ubuntu system detected: $ID $VERSION_ID"
    fi
else
    print_warning "Cannot detect operating system"
fi

# Step 3: Install Just if needed
print_info "Checking Just command runner..."
if ! command -v just >/dev/null 2>&1; then
    print_warning "Just not found - installing..."
    ./install_just.sh
    export PATH="$HOME/bin:$PATH"
else
    print_success "Just command runner available"
fi

# Step 4: Check if system is set up
print_info "Checking system setup..."
system_setup=true

# Check ROS 2
if ! command -v ros2 >/dev/null 2>&1; then
    print_warning "ROS 2 not found"
    system_setup=false
fi

# Check Python dependencies
if ! python3 -c "import pymavlink, cv2, numpy" 2>/dev/null; then
    print_warning "Python dependencies missing"
    system_setup=false
fi

# Check workspace build
if [ ! -f "/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone/install/setup.bash" ]; then
    print_warning "ROS 2 workspace not built"
    system_setup=false
fi

# Step 5: Setup system if needed
if [ "$system_setup" = false ]; then
    print_warning "System setup incomplete"
    echo ""
    echo "Would you like to run complete setup? (This may take 10-15 minutes)"
    read -p "Run setup? (y/N): " setup_confirm
    
    if [[ $setup_confirm =~ ^[Yy]$ ]]; then
        print_info "Running complete system setup..."
        just setup
    else
        print_error "Setup cancelled - system may not work properly"
        exit 1
    fi
else
    print_success "System setup complete"
fi

# Step 6: Hardware check
print_info "Checking hardware connectivity..."
if ls /dev/tty{USB,ACM}* 2>/dev/null >/dev/null; then
    print_success "Hardware devices found:"
    ls /dev/tty{USB,ACM}* | head -3
else
    print_warning "No hardware devices found - will use dummy mode"
fi

# Step 7: Run system validation
print_info "Running system validation..."
if just test >/dev/null 2>&1; then
    print_success "System validation passed"
else
    print_error "System validation failed"
    print_info "Running diagnostic..."
    just doctor
    exit 1
fi

# Step 8: Competition menu
echo ""
print_header "🎯 COMPETITION READY!"
print_header "===================="
echo ""
echo "Choose your action:"
echo "1. 🐛 Debug Mission (Practice with step-by-step control)"
echo "2. 🚀 Autonomous Mission (Competition mode)"
echo "3. 🧪 Run System Tests"
echo "4. 🏥 System Diagnosis"
echo "5. 📋 Competition Checklist"
echo "6. 🆘 Emergency Procedures"
echo "7. ❌ Exit"
echo ""

while true; do
    read -p "Select option (1-7): " choice
    
    case $choice in
        1)
            print_info "Starting debug mission..."
            just mission-debug
            break
            ;;
        2)
            print_warning "⚠️  AUTONOMOUS COMPETITION MODE"
            echo "This will run the full autonomous mission without manual control."
            read -p "Are you sure? (y/N): " confirm
            if [[ $confirm =~ ^[Yy]$ ]]; then
                just mission-auto
            else
                print_info "Autonomous mission cancelled"
            fi
            break
            ;;
        3)
            print_info "Running system tests..."
            just test
            ;;
        4)
            print_info "Running system diagnosis..."
            just doctor
            ;;
        5)
            just checklist
            ;;
        6)
            just emergency
            ;;
        7)
            print_info "Exiting..."
            exit 0
            ;;
        *)
            print_error "Invalid option. Please choose 1-7."
            ;;
    esac
    
    echo ""
    echo "Choose another action or press 7 to exit:"
done

print_success "Competition startup complete!"
print_info "Good luck in KAERTEI 2025 FAIO! 🚁✨"
