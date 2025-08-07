#!/bin/bash
"""
Quick setup script untuk PX4 Waypoint Configuration
Mempermudah setting waypoint untuk berbagai skenario
"""

echo "🛰️  KAERTEI 2025 FAIO - PX4 Waypoint Setup"
echo "==========================================="

# Function untuk setup venue
setup_venue() {
    case $1 in
        "simple")
            echo "🏠 Setting up Simple 3-Waypoint Configuration..."
            python3 -c "
from drone_mvp.px4_waypoint_config import PX4WaypointConfig
config = PX4WaypointConfig()
config.set_venue_preset('simple_3wp')
print('✅ Simple 3-waypoint setup complete!')
"
            ;;
        "test")
            echo "🧪 Setting up Test Configuration..."
            python3 -c "
from drone_mvp.px4_waypoint_config import PX4WaypointConfig
config = PX4WaypointConfig()
config.set_venue_preset('indoor_test')
print('✅ Test configuration setup complete!')
"
            ;;
        "outdoor")
            echo "🌤️  Setting up Outdoor Test Configuration..."
            python3 -c "
from drone_mvp.px4_waypoint_config import PX4WaypointConfig
config = PX4WaypointConfig()
config.set_venue_preset('outdoor_test')
print('✅ Outdoor test configuration setup complete!')
"
            ;;
        "competition")
            echo "🏆 Setting up Competition Configuration..."
            python3 -c "
from drone_mvp.px4_waypoint_config import PX4WaypointConfig
config = PX4WaypointConfig()
config.set_venue_preset('competition_venue')
print('✅ Competition configuration setup complete!')
"
            ;;
        *)
            echo "❌ Unknown venue: $1"
            echo "Available venues: simple, test, outdoor, competition"
            exit 1
            ;;
    esac
}

# Function untuk custom waypoint setting
setup_custom() {
    echo "🔧 Custom Waypoint Configuration"
    echo "Format: sequence_name waypoint1,waypoint2,waypoint3"
    echo "Example: exit_to_pickup 1,2,3"
    
    read -p "Enter sequence name: " seq_name
    read -p "Enter waypoints (comma-separated): " waypoints
    
    python3 -c "
from kaertei_drone.navigation.px4_waypoint_config import PX4WaypointConfig
config = PX4WaypointConfig()
waypoints = [int(x.strip()) for x in '$waypoints'.split(',')]
if config.update_sequence_waypoints('$seq_name', waypoints):
    print(f'✅ Updated $seq_name: {waypoints}')
else:
    print(f'❌ Failed to update $seq_name')
"
}

# Main menu
if [ $# -eq 0 ]; then
    echo ""
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  venue <simple|test|outdoor|competition>  - Setup venue presets"
    echo "  custom                                   - Custom waypoint configuration"
    echo "  show                                     - Show current configuration"
    echo "  test                                     - Run waypoint test"
    echo ""
    echo "Examples:"
    echo "  $0 venue simple        # Setup simple 3-waypoint config"
    echo "  $0 venue competition   # Setup full competition config"
    echo "  $0 custom              # Interactive custom setup"
    echo "  $0 show                # Show current waypoint config"
    exit 0
fi

case $1 in
    "venue")
        if [ -z "$2" ]; then
            echo "❌ Please specify venue: simple, test, outdoor, competition"
            exit 1
        fi
        setup_venue $2
        ;;
    "custom")
        setup_custom
        ;;
    "show")
        echo "📋 Current Waypoint Configuration:"
        python3 -c "
from drone_mvp.px4_waypoint_config import PX4WaypointConfig
config = PX4WaypointConfig()
config.print_config()
"
        ;;
    "test")
        echo "🧪 Running PX4 Waypoint Test..."
        cd ~/ros/Dirgagah-KAERTEI
        python3 test_px4_waypoints.py
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo "Run '$0' without arguments for help"
        exit 1
        ;;
esac

echo ""
echo "🎯 Setup complete! You can now:"
echo "   1. Run 'ros2 launch drone_mvp kaertei_pi5_system.launch.py' to start system"
echo "   2. Use '$0 show' to view current configuration"  
echo "   3. Use '$0 test' to test waypoint communication"
echo ""
