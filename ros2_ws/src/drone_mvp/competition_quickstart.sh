#!/bin/bash
# KAERTEI 2025 FAIO - Competition Day Quick Start
# ===============================================
# For emergency use on competition day

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🏆 KAERTEI 2025 FAIO - Competition Day Quick Start${NC}"
echo -e "${BLUE}=================================================${NC}"
echo ""

# Quick system check
echo -e "${YELLOW}🔍 Quick System Check...${NC}"

# Check if just is installed
if ! command -v just &> /dev/null; then
    echo -e "${RED}❌ 'just' command not found${NC}"
    echo "Installing just..."
    curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin
    export PATH="$HOME/bin:$PATH"
fi

# Check if we're in the right directory
if [ ! -f "Justfile" ]; then
    echo -e "${RED}❌ Not in the right directory${NC}"
    echo "Please run from: ros2_ws/src/drone_mvp/"
    exit 1
fi

echo -e "${GREEN}✅ Environment OK${NC}"
echo ""

# Competition workflow
echo -e "${YELLOW}🎯 COMPETITION WORKFLOW:${NC}"
echo ""
echo "1. 🔧 System Setup (if needed):"
echo "   just setup"
echo ""
echo "2. 🧪 Test Everything:"
echo "   just test"
echo ""
echo "3. 🎮 Practice Run (Debug Mode):"
echo "   just mission-debug"
echo ""
echo "4. 🚀 Competition Run (Autonomous):"
echo "   just mission-auto"
echo ""

echo -e "${YELLOW}📋 PRE-FLIGHT CHECKLIST:${NC}"
echo "   □ Hardware connected (/dev/ttyUSB0 or /dev/ttyACM0)"
echo "   □ Cameras working (front, back, top)"
echo "   □ GPS coordinates updated in config/hardware_config.conf"
echo "   □ Electromagnets tested"
echo "   □ Battery charged"
echo "   □ RC transmitter ready (manual override)"
echo ""

echo -e "${GREEN}🎉 Ready for KAERTEI 2025! Good luck! 🍀${NC}"
echo ""
echo "For detailed help: just help"
