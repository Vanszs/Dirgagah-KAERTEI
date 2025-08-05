#!/bin/bash
# KAERTEI 2025 FAIO - Direct Python Execution (No Build Required)
# This script runs the checkpoint mission directly using Python path

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚁 KAERTEI 2025 FAIO - Direct Python Execution${NC}"
echo -e "${BLUE}=================================================${NC}"

# Default modes
EXECUTION_MODE="debug"
COMM_MODE="mavlink"

# Parse command line arguments
if [ "$1" == "auto" ]; then
    EXECUTION_MODE="auto"
elif [ "$1" == "debug" ] || [ -z "$1" ]; then
    EXECUTION_MODE="debug"
elif [ "$1" != "" ]; then
    echo "❌ Invalid execution mode: $1"
    echo "Usage: $0 [debug|auto]"
    exit 1
fi

echo "⚙️ Configuration:"
echo "   Execution Mode: $EXECUTION_MODE"
echo "   Communication: $COMM_MODE (Direct MAVLink)"
echo ""

if [ "$EXECUTION_MODE" == "debug" ]; then
    echo -e "${YELLOW}🐛 DEBUG mode - Manual 'next' input required for each checkpoint${NC}"
else
    echo -e "${GREEN}🚀 AUTO mode - Autonomous execution${NC}"
fi

echo "📡 Using direct MAVLink communication (no MAVROS required)"
echo ""

# Setup environment
export ROS_DOMAIN_ID=0
export PYTHONPATH="$(pwd):$(pwd)/drone_mvp:$PYTHONPATH"

# Source ROS 2 if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS 2 Humble sourced"
else
    echo "⚠️  ROS 2 not found, using Python-only mode"
fi

echo ""
echo "🚀 Starting checkpoint mission system..."
echo "   📁 Python path: $PYTHONPATH"
echo ""

# Run the checkpoint mission node directly
if [ "$EXECUTION_MODE" == "debug" ]; then
    python3 -c "
import sys
sys.path.insert(0, '$(pwd)')
sys.path.insert(0, '$(pwd)/drone_mvp')

try:
    from drone_mvp.checkpoint_mission_node import CheckpointMissionNode
    import rclpy
    
    print('🎯 Initializing checkpoint mission in DEBUG mode...')
    rclpy.init()
    
    node = CheckpointMissionNode()
    node.debug_mode = True
    
    print('✅ Mission node ready!')
    print('💡 Type \"next\" to proceed through checkpoints')
    
    rclpy.spin(node)
    
except ImportError as e:
    print(f'❌ Import error: {e}')
    print('🐳 Try running in Docker: ./docker_runner.sh run')
    sys.exit(1)
except Exception as e:
    print(f'❌ Error: {e}')
    sys.exit(1)
finally:
    try:
        rclpy.shutdown()
    except:
        pass
"
else
    python3 -c "
import sys
sys.path.insert(0, '$(pwd)')
sys.path.insert(0, '$(pwd)/drone_mvp')

try:
    from drone_mvp.checkpoint_mission_node import CheckpointMissionNode
    import rclpy
    
    print('🎯 Initializing checkpoint mission in AUTO mode...')
    rclpy.init()
    
    node = CheckpointMissionNode()
    node.debug_mode = False
    
    print('✅ Mission node ready!')
    print('🚀 Running autonomous mission...')
    
    rclpy.spin(node)
    
except ImportError as e:
    print(f'❌ Import error: {e}')
    print('🐳 Try running in Docker: ./docker_runner.sh run')
    sys.exit(1)
except Exception as e:
    print(f'❌ Error: {e}')
    sys.exit(1)
finally:
    try:
        rclpy.shutdown()
    except:
        pass
"
fi
