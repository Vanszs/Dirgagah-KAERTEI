#!/bin/bash
# KAERTEI 2025 FAIO - Ubuntu 22.04 Competition Test Suite
# Comprehensive system validation for competition readiness

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

echo -e "${BLUE}🧪 KAERTEI 2025 FAIO - Competition Test Suite${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Test Results Array
declare -a test_results=()

# Function to add test result
add_test_result() {
    local test_name="$1"
    local result="$2"  # true/false
    test_results+=("$test_name:$result")
}

# Test Ubuntu version
test_ubuntu() {
    print_step "🐧 Testing Ubuntu 22.04..."
    
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "22.04" ]]; then
            print_success "✅ Ubuntu 22.04 LTS - Perfect!"
            add_test_result "Ubuntu 22.04" "true"
            return 0
        elif [[ "$ID" == "ubuntu" ]]; then
            print_warning "⚠️  Ubuntu $VERSION_ID - 22.04 LTS recommended"
            add_test_result "Ubuntu Version" "partial"
            return 0
        else
            print_error "❌ Unsupported OS: $ID $VERSION_ID"
            add_test_result "Ubuntu Version" "false"
            return 1
        fi
    else
        print_error "❌ Cannot detect OS"
        add_test_result "Ubuntu Version" "false"
        return 1
    fi
}

# Test ROS 2 Humble
test_ros2() {
    print_step "🤖 Testing ROS 2 Humble..."
    
    if command -v ros2 >/dev/null 2>&1; then
        print_success "✅ ROS 2 command available"
        
        # Check version
        if ros2 --version | grep -q "humble"; then
            print_success "✅ ROS 2 Humble confirmed"
            add_test_result "ROS 2 Humble" "true"
        else
            print_warning "⚠️  ROS 2 available but not Humble"
            add_test_result "ROS 2 Humble" "partial"
        fi
        return 0
    else
        print_error "❌ ROS 2 not found"
        add_test_result "ROS 2 Humble" "false"
        return 1
    fi
}

# Test MAVROS
test_mavros() {
    print_step "📡 Testing MAVROS..."
    
    if dpkg -l | grep -q ros-humble-mavros; then
        print_success "✅ MAVROS installed"
        add_test_result "MAVROS" "true"
        return 0
    else
        print_error "❌ MAVROS not installed"
        add_test_result "MAVROS" "false"
        return 1
    fi
}

# Test Python environment
test_python() {
    print_step "🐍 Testing Python environment..."
    
    # Test virtual environment
    local venv_path="/home/vanszs/Documents/ros2/ros2_env"
    if [ -d "$venv_path" ]; then
        print_success "✅ Python virtual environment found"
        
        # Activate and test packages
        source "$venv_path/bin/activate"
        
        # Test core packages
        local packages=("cv2" "numpy" "pymavlink" "serial")
        local failed_packages=()
        
        for package in "${packages[@]}"; do
            if python3 -c "import $package" 2>/dev/null; then
                print_success "   ✅ $package"
            else
                print_error "   ❌ $package missing"
                failed_packages+=("$package")
            fi
        done
        
        deactivate
        
        if [ ${#failed_packages[@]} -eq 0 ]; then
            add_test_result "Python Environment" "true"
            return 0
        else
            add_test_result "Python Environment" "false"
            return 1
        fi
    else
        print_error "❌ Python virtual environment not found"
        add_test_result "Python Environment" "false"
        return 1
    fi
}

# Test workspace
test_workspace() {
    print_step "📁 Testing ROS 2 workspace..."
    
    local workspace_path="/home/vanszs/Documents/ros2/ros2_ws"
    
    if [ ! -d "$workspace_path" ]; then
        print_error "❌ Workspace not found"
        add_test_result "Workspace" "false"
        return 1
    fi
    
    # Check drone_mvp package
    if [ -d "$workspace_path/src/drone_mvp" ]; then
        print_success "✅ drone_mvp package found"
    else
        print_error "❌ drone_mvp package missing"
        add_test_result "Workspace" "false"
        return 1
    fi
    
    # Check if built
    if [ -f "$workspace_path/install/setup.bash" ]; then
        print_success "✅ Workspace built"
        add_test_result "Workspace" "true"
        return 0
    else
        print_warning "⚠️  Workspace not built"
        add_test_result "Workspace" "partial"
        return 0
    fi
}

# Test mission files
test_mission_files() {
    print_step "🎯 Testing mission files..."
    
    local base_path="/home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp"
    local required_files=(
        "checkpoint_mission_mavros.py"
        "checkpoint_mission_node.py" 
        "config/hardware_config.conf"
        "Justfile"
        "setup_ubuntu.sh"
        "launch_mission_ubuntu.sh"
        "validate_ubuntu.py"
        "doctor_ubuntu.py"
    )
    
    local missing_files=()
    
    for file in "${required_files[@]}"; do
        if [ -f "$base_path/$file" ]; then
            print_success "   ✅ $file"
        else
            print_error "   ❌ $file missing"
            missing_files+=("$file")
        fi
    done
    
    if [ ${#missing_files[@]} -eq 0 ]; then
        add_test_result "Mission Files" "true"
        return 0
    else
        add_test_result "Mission Files" "false"
        return 1
    fi
}

# Test 26 checkpoints
test_checkpoints() {
    print_step "🎯 Testing 26 checkpoints..."
    
    # Run Python validation
    if python3 validate_ubuntu.py >/dev/null 2>&1; then
        print_success "✅ 26 checkpoints validated"
        add_test_result "26 Checkpoints" "true"
        return 0
    else
        print_error "❌ Checkpoint validation failed"
        add_test_result "26 Checkpoints" "false"
        return 1
    fi
}

# Test hardware
test_hardware() {
    print_step "🔌 Testing hardware access..."
    
    # Check serial devices
    if ls /dev/tty{USB,ACM}* 2>/dev/null >/dev/null; then
        print_success "✅ Hardware devices found"
        hardware_available=true
    else
        print_warning "⚠️  No hardware - dummy mode available"
        hardware_available=false
    fi
    
    # Check permissions
    import pwd, grp 2>/dev/null || python3 -c "
import pwd, grp
import os
user = pwd.getpwuid(os.getuid()).pw_name
user_groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
required_groups = ['dialout', 'video']
missing_groups = [g for g in required_groups if g not in user_groups]
if not missing_groups:
    print('permissions_ok')
else:
    print('permissions_missing')
" > /tmp/perm_check
    
    if grep -q "permissions_ok" /tmp/perm_check; then
        print_success "✅ User permissions OK"
        add_test_result "Hardware Access" "true"
    else
        print_error "❌ Missing permissions"
        add_test_result "Hardware Access" "false"
    fi
    
    rm -f /tmp/perm_check
    return 0
}

# Test development tools
test_dev_tools() {
    print_step "🛠️ Testing development tools..."
    
    local tools=("git" "colcon")
    local missing_tools=()
    
    for tool in "${tools[@]}"; do
        if command -v "$tool" >/dev/null 2>&1; then
            print_success "   ✅ $tool"
        else
            print_error "   ❌ $tool missing"
            missing_tools+=("$tool")
        fi
    done
    
    # Check Just (optional but recommended)
    if command -v just >/dev/null 2>&1; then
        print_success "   ✅ just (command runner)"
    else
        print_warning "   ⚠️  just missing (recommended)"
    fi
    
    if [ ${#missing_tools[@]} -eq 0 ]; then
        add_test_result "Development Tools" "true"
        return 0
    else
        add_test_result "Development Tools" "false"
        return 1
    fi
}

# Generate test report
generate_report() {
    print_step "📊 Generating test report..."
    
    echo ""
    echo -e "${CYAN}🏆 KAERTEI 2025 FAIO - Test Results${NC}"
    echo -e "${CYAN}===================================${NC}"
    
    local total_tests=0
    local passed_tests=0
    local partial_tests=0
    local failed_tests=0
    
    for result in "${test_results[@]}"; do
        IFS=':' read -r test_name test_status <<< "$result"
        total_tests=$((total_tests + 1))
        
        case "$test_status" in
            "true")
                print_success "✅ $test_name"
                passed_tests=$((passed_tests + 1))
                ;;
            "partial")
                print_warning "⚠️  $test_name (partial)"
                partial_tests=$((partial_tests + 1))
                ;;
            "false")
                print_error "❌ $test_name"
                failed_tests=$((failed_tests + 1))
                ;;
        esac
    done
    
    echo ""
    echo -e "${CYAN}📈 Summary:${NC}"
    echo "   Total Tests: $total_tests"
    echo "   Passed: $passed_tests"
    echo "   Partial: $partial_tests" 
    echo "   Failed: $failed_tests"
    
    echo ""
    if [ $failed_tests -eq 0 ]; then
        if [ $partial_tests -eq 0 ]; then
            print_success "🎉 ALL TESTS PASSED! Competition ready!"
            echo ""
            echo -e "${GREEN}Next steps:${NC}"
            echo "   just mission-debug    # Practice run"
            echo "   just mission-auto     # Competition run"
            return 0
        else
            print_warning "⚠️  MOSTLY READY - Minor issues detected"
            echo ""
            echo -e "${YELLOW}Recommendations:${NC}"
            echo "   just doctor          # Check issues"
            echo "   just setup           # Fix setup issues"
            return 0
        fi
    else
        print_error "❌ TESTS FAILED - System not competition ready"
        echo ""
        echo -e "${YELLOW}Fix issues with:${NC}"
        echo "   just doctor          # Diagnose problems"
        echo "   just setup           # Complete setup"
        echo "   just clean && just setup  # Reset everything"
        return 1
    fi
}

# Main test execution
main() {
    # Change to project directory
    local project_dir="/home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp"
    if [ -d "$project_dir" ]; then
        cd "$project_dir"
        print_info "📁 Working directory: $project_dir"
    else
        print_warning "⚠️  Project directory not found - using current directory"
    fi
    
    echo ""
    
    # Run all tests
    local tests=(
        "test_ubuntu"
        "test_ros2"
        "test_mavros"
        "test_python"
        "test_workspace"
        "test_mission_files"
        "test_checkpoints"
        "test_hardware"
        "test_dev_tools"
    )
    
    for test_func in "${tests[@]}"; do
        $test_func || true  # Continue even if test fails
        echo ""
    done
    
    # Generate final report
    generate_report
}

# Handle command line arguments
case "${1:-all}" in
    "all"|"")
        main
        ;;
    "quick")
        print_info "🚀 Quick test mode"
        test_ubuntu
        test_ros2
        test_python
        test_workspace
        generate_report
        ;;
    "hardware")
        print_info "🔌 Hardware test mode"
        test_hardware
        generate_report
        ;;
    *)
        print_error "❌ Unknown test mode: $1"
        echo "Usage: $0 [all|quick|hardware]"
        exit 1
        ;;
esac
