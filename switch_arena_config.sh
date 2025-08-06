#!/bin/bash
# KAERTEI 2025 - Quick Arena Configuration Script

set -e

ARENA_TYPE="$1"
CONFIG_FILE="kaertei_drone/config/hardware_config.conf"
BACKUP_FILE="kaertei_drone/config/hardware_config.conf.backup"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() { echo -e "${BLUE}$1${NC}"; }
print_success() { echo -e "${GREEN}✅ $1${NC}"; }
print_error() { echo -e "${RED}❌ $1${NC}"; }
print_info() { echo -e "${YELLOW}ℹ️  $1${NC}"; }

print_header "🏟️ KAERTEI 2025 - Arena Configuration Switch"
print_header "=============================================="

# Validate argument
if [[ "$ARENA_TYPE" != "left" && "$ARENA_TYPE" != "right" ]]; then
    print_error "Invalid arena type: $ARENA_TYPE"
    echo "Usage: $0 [left|right]"
    echo ""
    echo "Examples:"
    echo "  $0 left   # Configure for LEFT turn arena"  
    echo "  $0 right  # Configure for RIGHT turn arena"
    exit 1
fi

# Check if config file exists
if [[ ! -f "$CONFIG_FILE" ]]; then
    print_error "Config file not found: $CONFIG_FILE"
    print_info "Please run from project root directory"
    exit 1
fi

# Create backup
cp "$CONFIG_FILE" "$BACKUP_FILE"
print_info "Created backup: $BACKUP_FILE"

# Update configuration
print_info "Configuring for arena type: $ARENA_TYPE"

if [[ "$ARENA_TYPE" == "left" ]]; then
    # Set to LEFT turn
    sed -i 's/turn_direction = right/turn_direction = left/' "$CONFIG_FILE"
    
    print_success "Arena configured for LEFT TURN"
    print_info "Checkpoint 9 will execute:"
    echo "  - Move LEFT (positive Y velocity)"
    echo "  - Move FORWARD" 
    
elif [[ "$ARENA_TYPE" == "right" ]]; then
    # Set to RIGHT turn  
    sed -i 's/turn_direction = left/turn_direction = right/' "$CONFIG_FILE"
    
    print_success "Arena configured for RIGHT TURN"
    print_info "Checkpoint 9 will execute:"
    echo "  - Move RIGHT (negative Y velocity)"
    echo "  - Move FORWARD"
fi

# Verify configuration
CURRENT_CONFIG=$(grep "turn_direction" "$CONFIG_FILE" | head -1)
print_info "Current configuration: $CURRENT_CONFIG"

echo ""
print_header "🎯 Next Steps:"
echo "1. Test configuration: ./run_checkpoint_mission.sh debug mavros"
echo "2. Monitor CP-09 (NAVIGATE_TURN_DIRECTION) behavior"  
echo "3. Verify turn direction matches arena layout"
echo "4. Restore backup if needed: cp $BACKUP_FILE $CONFIG_FILE"

print_success "Arena configuration complete!"
