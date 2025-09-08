#!/bin/bash
# Fix NumPy version conflict with cv_bridge
# Script to downgrade NumPy to a compatible version (1.24.x)

echo "🛠️ Fixing NumPy version for ROS2 Humble compatibility..."
echo "==================================================="
echo "Current NumPy version:"
python3 -c "import numpy; print(numpy.__version__)"

echo -e "\n📦 Installing NumPy 1.24.3 (compatible with cv_bridge)..."
pip install numpy==1.24.3

echo -e "\n✅ New NumPy version:"
python3 -c "import numpy; print(numpy.__version__)"
echo ""
echo "⚠️  Note: If using a venv or conda environment, make sure to"
echo "   run this script in the same environment used by ROS."
echo ""
echo "🚀 Now rebuild your workspace with: just build"