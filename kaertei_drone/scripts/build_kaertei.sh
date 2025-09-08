#!/bin/bash

# KAERTEI 2025 - Independent Build Script
# Builds the clean kaertei_drone package as standalone system

echo "🔧 KAERTEI 2025 - Independent Build System"
echo "=========================================="

# Set directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"   # kaertei_drone
BUILD_DIR="$PROJECT_DIR/build"
INSTALL_DIR="$PROJECT_DIR/install"

cd "$PROJECT_DIR"

echo "📁 Project Directory: $PROJECT_DIR"
echo "🔨 Build Directory: $BUILD_DIR" 
echo "📦 Install Directory: $INSTALL_DIR"

# Source ROS2 environment
echo "🔄 Setting up ROS2 environment (Foxy)..."
source /opt/ros/foxy/setup.bash

# Verify Python packaging compatibility (setuptools vs importlib_metadata)
echo "🧪 Checking Python packaging compatibility..."
python3 - <<'PY'
import sys
ok = True
try:
    import importlib_metadata as imd  # backport used by setuptools on py3.8
    has_entrypoints = hasattr(imd, 'EntryPoints')
except Exception as e:
    has_entrypoints = False

if not has_entrypoints:
    sys.exit("MISSING_ENTRYPOINTS")
print("✅ Packaging OK")
PY

if [[ $? -ne 0 ]]; then
    echo "⚠️  Detected old importlib_metadata (missing EntryPoints). Fixing..."
    python3 -m pip install --user --upgrade importlib_metadata >/dev/null || true
    # Re-check
    python3 - <<'PY'
import sys
try:
    import importlib_metadata as imd
    assert hasattr(imd, 'EntryPoints')
    print('✅ Packaging OK after upgrade')
except Exception:
    print('❌ Packaging still incompatible. Run:')
    print('   python3 -m pip install --user --upgrade importlib_metadata setuptools wheel')
    sys.exit(1)
PY
    if [[ $? -ne 0 ]]; then
        echo "❌ Build aborted due to Python packaging mismatch."
        exit 1
    fi
fi

# Verify packaging compatibility: canonicalize_version supports strip_trailing_zero
echo "🧪 Checking packaging canonicalize_version..."
python3 - <<'PY'
import sys
try:
    from packaging.utils import canonicalize_version
    # Call with kw-only arg expected by newer setuptools
    canonicalize_version("1.0.0", strip_trailing_zero=False)
    print('✅ packaging OK')
except TypeError:
    sys.exit('MISSING_PACKAGING_FEATURE')
except Exception as e:
    print('❌ packaging check error:', e)
    sys.exit(1)
PY

if [[ $? -ne 0 ]]; then
    echo "⚠️  Detected old packaging (no strip_trailing_zero). Upgrading..."
    python3 -m pip install --user --upgrade packaging >/dev/null || true
    python3 - <<'PY'
import sys
try:
    from packaging.utils import canonicalize_version
    canonicalize_version("1.0.0", strip_trailing_zero=False)
    print('✅ packaging OK after upgrade')
except Exception:
    print('❌ Packaging still incompatible. Run:')
    print('   python3 -m pip install --user --upgrade packaging setuptools wheel')
    sys.exit(1)
PY
    if [[ $? -ne 0 ]]; then
        echo "❌ Build aborted due to packaging version mismatch."
        exit 1
    fi
fi

# Clean previous build if requested
if [[ "$1" == "clean" ]]; then
    echo "🧹 Cleaning previous build..."
    rm -rf build install
    echo "✅ Clean completed"
fi

# Create build directories
mkdir -p "$BUILD_DIR" "$INSTALL_DIR"

echo "🔨 Building kaertei_drone package..."

# Build with colcon
colcon build \
    --build-base "$BUILD_DIR" \
    --install-base "$INSTALL_DIR" \
    --packages-select kaertei_drone \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    
    # Ensure ROS 2 launch finds executables under lib/<pkg>
    LIBEXEC_DIR="$INSTALL_DIR/kaertei_drone/lib/kaertei_drone"
    BIN_DIR="$INSTALL_DIR/kaertei_drone/bin"
    if [ -d "$BIN_DIR" ]; then
        mkdir -p "$LIBEXEC_DIR"
        for exe in "$BIN_DIR"/*; do
            name="$(basename "$exe")"
            # Create/update symlink into lib/<pkg> so launch_ros can resolve it
            ln -sf "../../bin/$name" "$LIBEXEC_DIR/$name"
        done
        echo "🔗 Ensured libexec wrappers in $LIBEXEC_DIR"
    fi
    
    echo "🔧 Setting up environment..."
    # Create environment setup script
    cat > "$SCRIPT_DIR/setup_kaertei.sh" << 'EOF'
#!/bin/bash
# KAERTEI 2025 Environment Setup
source /opt/ros/foxy/setup.bash
source "$(cd "$(dirname "$0")/.." && pwd)/install/setup.bash"
echo "✅ KAERTEI environment ready!"
EOF
    chmod +x "$SCRIPT_DIR/setup_kaertei.sh"
    
    echo "🎯 Build Summary:"
    echo "   - Package: kaertei_drone v2.0.0"
    echo "   - Built nodes: $(ls install/kaertei_drone/bin/ 2>/dev/null | wc -l) executables"
    echo "   - Launch files: $(ls install/kaertei_drone/share/kaertei_drone/launch/ 2>/dev/null | wc -l) files"
    
    echo ""
    echo "🚀 Usage:"
    echo "   ./setup_kaertei.sh                    # Setup environment"
    echo "   ./run_kaertei.sh debug                # Launch debug mode"
    echo "   ./run_kaertei.sh auto                 # Launch autonomous mode"

    # Append workspace setup to ~/.bashrc if not present
    BASHRC="$HOME/.bashrc"
    WORK_SETUP="source $PROJECT_DIR/install/setup.bash"
    if ! grep -q "${WORK_SETUP//\//\/}" "$BASHRC" 2>/dev/null; then
        echo "" >> "$BASHRC"
        echo "# KAERTEI workspace (Foxy)" >> "$BASHRC"
        echo "$WORK_SETUP" >> "$BASHRC"
        echo "✅ Appended workspace setup to ~/.bashrc"
    fi
    
else
    echo "❌ Build failed!"
    echo "Check the error messages above for details."
    exit 1
fi
