#!/bin/bash
# KAERTEI 2025 FAIO - Quick Launcher
# Simple interface for common operations

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚁 KAERTEI 2025 FAIO - Quick Launcher${NC}"
echo -e "${BLUE}====================================${NC}"

# Quick menu
show_menu() {
    echo ""
    echo "Quick Options:"
    echo "1. 🔧 Setup System (Ubuntu)"
    echo "2. 🚁 Run Debug Mission"
    echo "3. 🧪 Test Hardware"
    echo "4. 🐳 Docker Mode"
    echo "5. 📊 System Status"
    echo "6. ❓ Full Help"
    echo ""
    read -p "Choose option (1-6): " choice
    
    case $choice in
        1) ./kaertei_master.sh setup ubuntu ;;
        2) ./kaertei_master.sh mission debug ;;
        3) ./kaertei_master.sh test hardware ;;
        4) ./kaertei_master.sh docker run ;;
        5) ./kaertei_master.sh status ;;
        6) ./kaertei_master.sh help ;;
        *) echo -e "${RED}Invalid option${NC}" ;;
    esac
}

# Check if master script exists
if [ ! -f "kaertei_master.sh" ]; then
    echo -e "${RED}❌ Master script not found${NC}"
    exit 1
fi

# Make executable
chmod +x kaertei_master.sh

# Run menu or direct command
if [ $# -eq 0 ]; then
    show_menu
else
    ./kaertei_master.sh "$@"
fi
