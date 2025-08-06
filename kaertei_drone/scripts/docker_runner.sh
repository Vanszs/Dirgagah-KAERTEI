#!/bin/bash
# KAERTEI 2025 FAIO - Docker Setup and Runner
# Complete Docker setup for hexacopter system

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🐳 KAERTEI 2025 FAIO - Docker Setup${NC}"
echo -e "${BLUE}===================================${NC}"

# Check if Docker is installed and running
check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}❌ Docker not installed${NC}"
        echo "Please install Docker first:"
        echo "  Ubuntu: sudo apt install docker.io"
        echo "  Arch: sudo pacman -S docker"
        exit 1
    fi
    
    if ! docker info >/dev/null 2>&1; then
        echo -e "${RED}❌ Docker daemon not running${NC}"
        echo "Please start Docker service:"
        echo "  sudo systemctl start docker"
        exit 1
    fi
    
    echo -e "${GREEN}✅ Docker is ready${NC}"
}

# Build Docker image
build_image() {
    echo -e "${YELLOW}🔨 Building Docker image...${NC}"
    docker build -t kaertei-2025-faio .
    echo -e "${GREEN}✅ Docker image built successfully${NC}"
}

# Run Docker container
run_container() {
    echo -e "${YELLOW}🚀 Starting Docker container...${NC}"
    
    # Stop existing container if running
    docker stop kaertei-container 2>/dev/null || true
    docker rm kaertei-container 2>/dev/null || true
    
    # Run new container with hardware access
    docker run -it --rm \
        --name kaertei-container \
        --privileged \
        --network host \
        -v /dev:/dev \
        -v $(pwd):/workspace/host \
        -w /workspace/kaertei_drone \
        kaertei-2025-faio
}

# Test container
test_container() {
    echo -e "${YELLOW}🧪 Testing Docker container...${NC}"
    docker run --rm \
        --privileged \
        -v /dev:/dev \
        kaertei-2025-faio \
        bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
}

# Show status
show_status() {
    echo -e "${BLUE}📊 Docker Status${NC}"
    echo "================"
    
    if docker images | grep -q kaertei-2025-faio; then
        echo -e "${GREEN}✅ Image: kaertei-2025-faio exists${NC}"
    else
        echo -e "${RED}❌ Image: kaertei-2025-faio not found${NC}"
    fi
    
    if docker ps | grep -q kaertei-container; then
        echo -e "${GREEN}✅ Container: kaertei-container running${NC}"
    else
        echo -e "${YELLOW}⏸️  Container: kaertei-container not running${NC}"
    fi
}

# Show help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build     - Build Docker image"
    echo "  run       - Run Docker container interactively"
    echo "  test      - Test Docker container"
    echo "  status    - Show Docker status"
    echo "  help      - Show this help"
    echo ""
    echo "Example:"
    echo "  $0 build    # Build image first"
    echo "  $0 run      # Run container"
}

# Main execution
case "${1:-help}" in
    build)
        check_docker
        build_image
        ;;
    run)
        check_docker
        run_container
        ;;
    test)
        check_docker
        test_container
        ;;
    status)
        check_docker
        show_status
        ;;
    help|*)
        show_help
        ;;
esac
