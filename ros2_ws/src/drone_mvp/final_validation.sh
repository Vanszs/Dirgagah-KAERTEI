#!/bin/bash

# Final System Validation for KAERTEI 2025 FAIO Drone System
# This script validates that everything is correctly configured

echo "🚁 KAERTEI 2025 FAIO - Final System Validation"
echo "=============================================="

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_check() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✅ $2${NC}"
    else
        echo -e "${RED}❌ $2${NC}"
    fi
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

check_count=0
pass_count=0

# Check 1: File structure
print_info "Checking package structure..."
required_files=(
    "package.xml"
    "CMakeLists.txt"
    "setup.sh"
    "drone_manager.sh"
    "README.md"
    "drone_mvp/mission_node.py"
    "launch/drone.launch.py"
    "config/mavros_config.yaml"
)

for file in "${required_files[@]}"; do
    check_count=$((check_count + 1))
    if [ -f "$file" ]; then
        print_check 0 "File exists: $file"
        pass_count=$((pass_count + 1))
    else
        print_check 1 "Missing file: $file"
    fi
done

# Check 2: Executable permissions
print_info "Checking executable permissions..."
executable_files=(
    "setup.sh"
    "drone_manager.sh"
    "test_mavros.sh"
    "quick_start.sh"
    "calibrate_hardware.py"
    "monitor.py"
    "simulate_mission.py"
)

for file in "${executable_files[@]}"; do
    check_count=$((check_count + 1))
    if [ -x "$file" ]; then
        print_check 0 "Executable: $file"
        pass_count=$((pass_count + 1))
    else
        print_check 1 "Not executable: $file"
    fi
done

# Check 3: ArduPilot references (no PX4)
print_info "Checking ArduPilot/MAVROS configuration..."
check_count=$((check_count + 1))
if ! grep -r "px4.launch.py" . --exclude-dir=.git >/dev/null 2>&1; then
    print_check 0 "No PX4 references found (ArduPilot only)"
    pass_count=$((pass_count + 1))
else
    print_check 1 "Found PX4 references - should use ArduPilot"
    echo "  Files with PX4 references:"
    grep -r "px4.launch.py" . --exclude-dir=.git | head -3
fi

# Check 4: MAVROS topics
print_info "Checking MAVROS topic usage..."
mavros_topics=(
    "/mavros/set_mode"
    "/mavros/cmd/arming"
    "/mavros/state"
    "/mavros/global_position/global"
)

for topic in "${mavros_topics[@]}"; do
    check_count=$((check_count + 1))
    if grep -r "$topic" . --exclude-dir=.git >/dev/null 2>&1; then
        print_check 0 "MAVROS topic found: $topic"
        pass_count=$((pass_count + 1))
    else
        print_check 1 "MAVROS topic missing: $topic"
    fi
done

# Check 5: KAERTEI branding (no KRTI)
print_info "Checking KAERTEI branding..."
check_count=$((check_count + 1))
krti_count=$(grep -r "KRTI" . --exclude-dir=.git --exclude="*.md" | wc -l)
if [ "$krti_count" -eq 0 ]; then
    print_check 0 "No KRTI references found (KAERTEI only)"
    pass_count=$((pass_count + 1))
else
    print_check 1 "Found $krti_count KRTI references - should be KAERTEI"
    echo "  Files with KRTI references:"
    grep -r "KRTI" . --exclude-dir=.git --exclude="*.md" | head -3
fi

# Check 6: Python node structure
print_info "Checking Python nodes..."
python_nodes=(
    "drone_mvp/mission_node.py"
    "drone_mvp/flight_mode_switcher.py"
    "drone_mvp/vision_detector_node.py"
    "drone_mvp/gps_monitor.py"
    "drone_mvp/sensor_monitor.py"
)

for node in "${python_nodes[@]}"; do
    check_count=$((check_count + 1))
    if [ -f "$node" ] && head -1 "$node" | grep -q "#!/usr/bin/env python3"; then
        print_check 0 "Python node OK: $node"
        pass_count=$((pass_count + 1))
    else
        print_check 1 "Python node issue: $node"
    fi
done

# Summary
echo ""
echo "=============================================="
echo "📊 VALIDATION SUMMARY"
echo "=============================================="
echo -e "Total checks: ${BLUE}$check_count${NC}"
echo -e "Passed: ${GREEN}$pass_count${NC}"
echo -e "Failed: ${RED}$((check_count - pass_count))${NC}"

success_rate=$(( (pass_count * 100) / check_count ))
echo -e "Success rate: ${BLUE}$success_rate%${NC}"

if [ $success_rate -ge 90 ]; then
    echo -e "${GREEN}🎉 SYSTEM VALIDATION PASSED!${NC}"
    echo -e "${GREEN}✅ Ready for KAERTEI 2025 FAIO Competition${NC}"
    exit_code=0
elif [ $success_rate -ge 75 ]; then
    echo -e "${YELLOW}⚠️  SYSTEM MOSTLY READY${NC}"
    echo -e "${YELLOW}🔧 Minor issues detected - review failed checks${NC}"
    exit_code=1
else
    echo -e "${RED}❌ SYSTEM NOT READY${NC}"
    echo -e "${RED}🚨 Major issues detected - fix critical problems${NC}"
    exit_code=2
fi

echo ""
echo "=============================================="
echo "🚀 Next Steps:"
echo "1. Fix any failed checks above"
echo "2. Run: ./setup.sh (install dependencies)"
echo "3. Run: ./drone_manager.sh calibrate (test hardware)"
echo "4. Run: ./drone_manager.sh mission (start mission)"
echo "=============================================="

exit $exit_code
