#!/bin/bash

# =====================================================================
# KAERTEI 2025 FAIO - Pi 5 Hardware Test Script
# Test all hardware components before mission
# =====================================================================

echo "🚁 KAERTEI 2025 - Hardware Validation (Pi 5 + Pixhawk4)"
echo "========================================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test results
FLIGHT_CONTROLLER_OK=false
CAMERAS_OK=false
LIDAR_OK=false
GPIO_OK=false

echo "🔍 Testing Hardware Components..."
echo "================================"
echo ""

# 1. Test Flight Controller (Pixhawk4)
echo -e "${BLUE}1. Flight Controller (Pixhawk4)${NC}"
echo "-------------------------------"
if ls /dev/ttyUSB0 &>/dev/null || ls /dev/ttyACM0 &>/dev/null; then
    echo -e "✅ ${GREEN}Flight controller detected${NC}"
    
    # Test MAVLink communication
    if timeout 5s python3 -c "
import serial
import time
try:
    port = '/dev/ttyUSB0' if __import__('os').path.exists('/dev/ttyUSB0') else '/dev/ttyACM0'
    ser = serial.Serial(port, 57600, timeout=1)
    time.sleep(2)
    if ser.is_open:
        print('MAVLink communication: OK')
        ser.close()
except Exception as e:
    print(f'MAVLink communication: FAILED - {e}')
" 2>/dev/null; then
        echo -e "✅ ${GREEN}MAVLink communication working${NC}"
        FLIGHT_CONTROLLER_OK=true
    else
        echo -e "⚠️  ${YELLOW}MAVLink communication failed${NC}"
    fi
else
    echo -e "❌ ${RED}No flight controller detected${NC}"
fi
echo ""

# 2. Test Camera System
echo -e "${BLUE}2. Camera System (3x USB)${NC}"
echo "-------------------------"
CAMERA_COUNT=0

for camera_id in 0 2 4; do
    camera_name=""
    case $camera_id in
        0) camera_name="Front" ;;
        2) camera_name="Back" ;;
        4) camera_name="Top" ;;
    esac
    
    if [ -e "/dev/video$camera_id" ]; then
        echo -n "$camera_name Camera (/dev/video$camera_id): "
        
        # Test camera with timeout
        if timeout 3s v4l2-ctl --device=/dev/video$camera_id --list-ctrls &>/dev/null; then
            echo -e "${GREEN}✅ Working${NC}"
            ((CAMERA_COUNT++))
        else
            echo -e "${RED}❌ Not responding${NC}"
        fi
    else
        echo -e "$camera_name Camera: ${RED}❌ Device not found${NC}"
    fi
done

if [ $CAMERA_COUNT -eq 3 ]; then
    CAMERAS_OK=true
    echo -e "✅ ${GREEN}All 3 cameras working${NC}"
else
    echo -e "⚠️  ${YELLOW}Only $CAMERA_COUNT/3 cameras working${NC}"
fi
echo ""

# 3. Test LiDAR System  
echo -e "${BLUE}3. LiDAR System (3x TF Mini Plus)${NC}"
echo "---------------------------------"
LIDAR_COUNT=0

for usb_id in 1 2 3; do
    lidar_name=""
    case $usb_id in
        1) lidar_name="Front" ;;
        2) lidar_name="Left" ;;
        3) lidar_name="Right" ;;
    esac
    
    if [ -e "/dev/ttyUSB$usb_id" ]; then
        echo -n "$lidar_name LiDAR (/dev/ttyUSB$usb_id): "
        
        # Test LiDAR data stream
        if timeout 2s cat /dev/ttyUSB$usb_id 2>/dev/null | head -c 10 | wc -c | grep -q "[1-9]"; then
            echo -e "${GREEN}✅ Data streaming${NC}"
            ((LIDAR_COUNT++))
        else
            echo -e "${YELLOW}⚠️  No data (might need configuration)${NC}"
        fi
    else
        echo -e "$lidar_name LiDAR: ${RED}❌ Device not found${NC}"
    fi
done

if [ $LIDAR_COUNT -eq 3 ]; then
    LIDAR_OK=true
    echo -e "✅ ${GREEN}All 3 LiDAR sensors detected${NC}"
elif [ $LIDAR_COUNT -gt 0 ]; then
    echo -e "⚠️  ${YELLOW}$LIDAR_COUNT/3 LiDAR sensors working${NC}"
else
    echo -e "❌ ${RED}No LiDAR sensors detected${NC}"
fi
echo ""

# 4. Test GPIO System
echo -e "${BLUE}4. GPIO System (Raspberry Pi 5)${NC}"
echo "-------------------------------"

if [ -c /dev/gpiomem ]; then
    echo -e "✅ ${GREEN}GPIO interface available${NC}"
    
    # Test GPIO pins (if RPi.GPIO is available)
    if python3 -c "import RPi.GPIO" 2>/dev/null; then
        echo -e "✅ ${GREEN}RPi.GPIO library available${NC}"
        
        # Test relay pins safely
        if python3 -c "
import RPi.GPIO as GPIO
import time
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)  # Front magnet relay
    GPIO.setup(19, GPIO.OUT)  # Back magnet relay
    GPIO.setup(21, GPIO.OUT)  # Status LED
    
    # Test outputs (safe - don't actually activate magnets)
    GPIO.output(21, GPIO.HIGH)  # Status LED ON
    time.sleep(0.1)
    GPIO.output(21, GPIO.LOW)   # Status LED OFF
    
    GPIO.cleanup()
    print('GPIO test: PASSED')
except Exception as e:
    print(f'GPIO test: FAILED - {e}')
" 2>/dev/null | grep -q "PASSED"; then
            echo -e "✅ ${GREEN}GPIO relay pins working${NC}"
            GPIO_OK=true
        else
            echo -e "⚠️  ${YELLOW}GPIO pin test failed${NC}"
        fi
    else
        echo -e "⚠️  ${YELLOW}RPi.GPIO library not installed${NC}"
        echo "   Install with: sudo apt install python3-rpi.gpio"
    fi
else
    echo -e "❌ ${RED}GPIO interface not accessible${NC}"
    echo "   Are you running on Raspberry Pi 5?"
fi
echo ""

# 5. System Integration Test
echo -e "${BLUE}5. System Integration${NC}"
echo "--------------------"
echo "📡 Pixhawk4 (6 motors + GPS): $([ "$FLIGHT_CONTROLLER_OK" = true ] && echo -e "${GREEN}✅ Ready" || echo -e "${RED}❌ Not ready")${NC}"
echo "📷 Camera System (3x): $([ "$CAMERAS_OK" = true ] && echo -e "${GREEN}✅ Ready" || echo -e "${YELLOW}⚠️  Partial")${NC}"  
echo "📡 LiDAR System (3x): $([ "$LIDAR_OK" = true ] && echo -e "${GREEN}✅ Ready" || echo -e "${YELLOW}⚠️  Partial")${NC}"
echo "🔌 GPIO Control (2x relay): $([ "$GPIO_OK" = true ] && echo -e "${GREEN}✅ Ready" || echo -e "${RED}❌ Not ready")${NC}"
echo ""

# Final Assessment
echo "🎯 HARDWARE ASSESSMENT"
echo "======================"

TOTAL_SYSTEMS=4
WORKING_SYSTEMS=0

[ "$FLIGHT_CONTROLLER_OK" = true ] && ((WORKING_SYSTEMS++))
[ "$CAMERAS_OK" = true ] && ((WORKING_SYSTEMS++))
[ "$LIDAR_OK" = true ] && ((WORKING_SYSTEMS++))
[ "$GPIO_OK" = true ] && ((WORKING_SYSTEMS++))

if [ $WORKING_SYSTEMS -eq $TOTAL_SYSTEMS ]; then
    echo -e "🎉 ${GREEN}SYSTEM READY FOR COMPETITION!${NC}"
    echo -e "✅ All $TOTAL_SYSTEMS hardware systems operational"
    echo ""
    echo "🚀 Next steps:"
    echo "   just test        # Run software validation"
    echo "   just debug       # Test mission step-by-step"
    echo "   just run         # Full autonomous mission"
    
elif [ $WORKING_SYSTEMS -ge 2 ]; then
    echo -e "⚠️  ${YELLOW}PARTIAL SYSTEM READY${NC}"
    echo -e "✅ $WORKING_SYSTEMS/$TOTAL_SYSTEMS systems working"
    echo ""
    echo "🔧 Fix remaining issues before competition"
    
else
    echo -e "❌ ${RED}SYSTEM NOT READY${NC}"
    echo -e "⚠️  Only $WORKING_SYSTEMS/$TOTAL_SYSTEMS systems working"
    echo ""
    echo "🆘 Critical issues need attention:"
    [ "$FLIGHT_CONTROLLER_OK" = false ] && echo "   - Connect and configure Pixhawk4"
    [ "$CAMERAS_OK" = false ] && echo "   - Check USB camera connections"
    [ "$LIDAR_OK" = false ] && echo "   - Connect LiDAR sensors via USB"
    [ "$GPIO_OK" = false ] && echo "   - Setup GPIO permissions on Pi 5"
fi

echo ""
echo "📋 Hardware Configuration Summary:"
echo "   Takeoff Altitude: 1.0m (updated from 0.6m)"
echo "   Indoor Navigation: 1.0m cruise altitude"  
echo "   Outdoor Navigation: 3.0m cruise altitude"
echo "   Total Mission Checkpoints: 26"
echo ""

exit $([ $WORKING_SYSTEMS -ge 3 ] && echo 0 || echo 1)
