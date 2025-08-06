#!/bin/bash
# KAERTEI 2025 FAIO - Just Wrapper for Fish Shell Compatibility
# Usage: ./just.sh <command>

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

case "${1:-help}" in
    help|"")
        echo -e "${BLUE}🚁 KAERTEI 2025 FAIO - Quick Commands${NC}"
        echo -e "${BLUE}====================================${NC}"
        echo ""
        echo -e "${GREEN}🚀 QUICK START:${NC}"
        echo "   ./just.sh setup-ubuntu           # Setup Ubuntu dependencies"
        echo "   ./just.sh docker-build           # Build Docker image"
        echo "   ./just.sh mission-debug-docker   # Run debug mission in Docker"
        echo ""
        echo -e "${YELLOW}🎯 MISSION COMMANDS:${NC}"
        echo "   ./just.sh mission-debug          # Debug mode (native)"
        echo "   ./just.sh mission-auto           # Auto mode (native)"
        echo "   ./just.sh mission-debug-docker   # Debug mode (Docker)"
        echo "   ./just.sh mission-auto-docker    # Auto mode (Docker)"
        echo ""
        echo -e "${PURPLE}🔧 DEVELOPMENT:${NC}"
        echo "   ./just.sh test-all               # Run all tests"
        echo "   ./just.sh clean-workspace        # Clean build artifacts"
        echo "   ./just.sh status                 # Show system status"
        echo "   ./just.sh docker-clean           # Clean Docker resources"
        ;;
    
    setup-ubuntu)
        echo -e "${BLUE}🐧 Setting up Ubuntu system dependencies...${NC}"
        bash ./setup.sh ubuntu
        echo -e "${GREEN}✅ Ubuntu setup completed!${NC}"
        ;;
    
    docker-build)
        echo -e "${BLUE}🐳 Building Docker image...${NC}"
        bash ./docker_runner.sh build
        echo -e "${GREEN}✅ Docker image built successfully!${NC}"
        ;;
    
    docker-run)
        echo -e "${BLUE}🚀 Starting Docker container...${NC}"
        bash ./docker_runner.sh run
        ;;
    
    docker-clean)
        echo -e "${YELLOW}🗑️ Cleaning Docker resources...${NC}"
        docker stop kaertei-container 2>/dev/null || true
        docker rm kaertei-container 2>/dev/null || true
        docker rmi kaertei-2025-faio 2>/dev/null || true
        echo -e "${GREEN}✅ Docker resources cleaned!${NC}"
        ;;
    
    mission-debug)
        echo -e "${YELLOW}🐛 Running mission in DEBUG mode (Native)...${NC}"
        bash ./run_direct.sh debug
        ;;
    
    mission-auto)
        echo -e "${GREEN}🚀 Running mission in AUTO mode (Native)...${NC}"
        bash ./run_direct.sh auto
        ;;
    
    mission-debug-docker)
        echo -e "${YELLOW}🐛 Running mission in DEBUG mode (Docker)...${NC}"
        docker run --rm -it \
            --privileged \
            --network host \
            -v /dev:/dev \
            kaertei-2025-faio \
            bash -c "cd /workspace/ros2_ws && source setup_env.sh && ./run_direct.sh debug"
        ;;
    
    mission-auto-docker)
        echo -e "${GREEN}🚀 Running mission in AUTO mode (Docker)...${NC}"
        docker run --rm -it \
            --privileged \
            --network host \
            -v /dev:/dev \
            kaertei-2025-faio \
            bash -c "cd /workspace/ros2_ws && source setup_env.sh && ./run_direct.sh auto"
        ;;
    
    test-all)
        echo -e "${BLUE}🧪 Running comprehensive tests...${NC}"
        echo "Testing system dependencies..."
        python3 -c "import rclpy, cv2, numpy as np; print('✅ Core dependencies OK')" || echo "❌ Missing dependencies"
        python3 -c "import pymavlink; print('✅ MAVLink OK')" || echo "❌ PyMAVLink missing"
        echo "Testing hardware configuration..."
        python3 -c "from drone_mvp.hardware_config import HardwareConfig; hw = HardwareConfig(); hw.print_summary()" 2>/dev/null || echo "⚠️ Hardware config not available"
        echo -e "${GREEN}✅ All tests completed!${NC}"
        ;;
    
    clean-workspace)
        echo -e "${YELLOW}🧹 Cleaning workspace...${NC}"
        cd /home/vanszs/Documents/ros2/ros2_ws
        rm -rf build/ install/ log/
        echo -e "${GREEN}✅ Workspace cleaned!${NC}"
        ;;
    
    status)
        echo -e "${BLUE}📊 System status...${NC}"
        bash ./kaertei_master.sh status
        ;;
    
    info)
        echo -e "${BLUE}ℹ️ KAERTEI 2025 FAIO System Information${NC}"
        echo "======================================"
        echo "🐧 OS: $(lsb_release -d 2>/dev/null || echo 'Unknown')"
        echo "🐍 Python: $(python3 --version)"
        echo "🤖 ROS 2: ${ROS_DISTRO:-Not sourced}"
        echo "🐳 Docker: $(docker --version 2>/dev/null || echo 'Not installed')"
        echo "📁 Project: $(pwd)"
        ;;
    
    list)
        echo -e "${BLUE}📋 Available commands:${NC}"
        echo ""
        echo "setup-ubuntu, docker-build, docker-run, docker-clean"
        echo "mission-debug, mission-auto, mission-debug-docker, mission-auto-docker"
        echo "test-all, clean-workspace, status, info, help"
        ;;
    
    *)
        echo -e "${RED}❌ Unknown command: $1${NC}"
        echo "Usage: ./just.sh <command>"
        echo "Run './just.sh help' for available commands"
        exit 1
        ;;
esac
