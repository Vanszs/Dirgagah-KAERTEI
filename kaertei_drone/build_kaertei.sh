#!/bin/bash

# KAERTEI 2025 - Independent Build Script
# Builds the clean kaertei_drone package as standalone system

echo "🔧 KAERTEI 2025 - Independent Build System"
echo "=========================================="

# Set project directory
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
INSTALL_DIR="$PROJECT_DIR/install"

cd "$PROJECT_DIR"

echo "📁 Project Directory: $PROJECT_DIR"
echo "🔨 Build Directory: $BUILD_DIR" 
echo "📦 Install Directory: $INSTALL_DIR"

# Source ROS2 environment
echo "🔄 Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

# Clean previous build if requested
if [[ "$1" == "clean" ]]; then
    echo "🧹 Cleaning previous build..."
    rm -rf build install
    echo "✅ Clean completed"
fi

# Create build directories
mkdir -p build install

echo "🔨 Building kaertei_drone package..."

# Build with colcon
colcon build \
    --build-base build \
    --install-base install \
    --packages-select kaertei_drone \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    
    echo "🔧 Setting up environment..."
    # Create environment setup script
    cat > setup_kaertei.sh << 'EOF'
#!/bin/bash
# KAERTEI 2025 Environment Setup
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "✅ KAERTEI environment ready!"
EOF
    chmod +x setup_kaertei.sh
    
    echo "🎯 Build Summary:"
    echo "   - Package: kaertei_drone v2.0.0"
    echo "   - Built nodes: $(ls install/kaertei_drone/lib/kaertei_drone/ 2>/dev/null | wc -l) executables"
    echo "   - Launch files: $(ls install/kaertei_drone/share/kaertei_drone/launch/ 2>/dev/null | wc -l) files"
    
    echo ""
    echo "🚀 Usage:"
    echo "   ./setup_kaertei.sh                    # Setup environment"
    echo "   ./run_kaertei.sh debug checkpoint     # Launch debug mode"
    echo "   ./run_kaertei.sh auto simple          # Launch simple mode"
    
else
    echo "❌ Build failed!"
    echo "Check the error messages above for details."
    exit 1
fi
