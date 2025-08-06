#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Mission Checkpoint Validator
Validates that all 26 checkpoints are correctly implemented
"""

import sys
import os
import importlib.util

# Add current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def validate_checkpoints():
    """Validate that all 26 checkpoints are implemented"""
    print("🔍 KAERTEI 2025 FAIO - Checkpoint Validation")
    print("=" * 50)
    
    # Expected 26 checkpoints
    expected_checkpoints = [
        "INIT",
        "TAKEOFF", 
        "SEARCH_ITEM_1_FRONT",
        "ALIGN_ITEM_1",
        "PICKUP_ITEM_1",
        "SEARCH_ITEM_2_BACK",
        "ALIGN_ITEM_2", 
        "PICKUP_ITEM_2",
        "NAVIGATE_TURN_DIRECTION",
        "SEARCH_DROPZONE",
        "DROP_ITEM_1_FRONT",
        "ASCEND_AFTER_DROP_1",
        "ALIGN_DROP_2_BACK",
        "DROP_ITEM_2_BACK", 
        "FIND_EXIT",
        "ASCEND_TO_OUTDOOR",
        "AUTO_WAYPOINT_1",
        "MANUAL_SEARCH_OUTDOOR",
        "PICKUP_OUTDOOR",
        "ASCEND_TO_WAYPOINT_2",
        "AUTO_WAYPOINT_2",
        "MANUAL_SEARCH_DROP_OUTDOOR",
        "DROP_OUTDOOR",
        "ASCEND_TO_WAYPOINT_3",
        "AUTO_WAYPOINT_3_LANDING",
        "COMPLETED"
    ]
    
    validation_results = {}
    
    # Test both mission node implementations
    mission_files = [
        'drone_mvp/checkpoint_mission_node.py',
        'drone_mvp/checkpoint_mission_mavros.py'
    ]
    
    for mission_file in mission_files:
        print(f"\n📋 Validating {mission_file}...")
        
        if not os.path.exists(mission_file):
            print(f"❌ File not found: {mission_file}")
            validation_results[mission_file] = False
            continue
            
        try:
            # Read file content
            with open(mission_file, 'r') as f:
                content = f.read()
            
            # Check if MissionCheckpoint enum exists
            if 'class MissionCheckpoint(Enum):' not in content:
                print(f"❌ MissionCheckpoint enum not found in {mission_file}")
                validation_results[mission_file] = False
                continue
            
            # Validate each checkpoint
            missing_checkpoints = []
            found_checkpoints = []
            
            for checkpoint in expected_checkpoints:
                if f'{checkpoint} = "{checkpoint}"' in content:
                    found_checkpoints.append(checkpoint)
                else:
                    missing_checkpoints.append(checkpoint)
            
            if missing_checkpoints:
                print(f"❌ Missing checkpoints in {mission_file}:")
                for cp in missing_checkpoints:
                    print(f"   - {cp}")
                validation_results[mission_file] = False
            else:
                print(f"✅ All 26 checkpoints found in {mission_file}")
                validation_results[mission_file] = True
                
            print(f"📊 Found {len(found_checkpoints)}/26 checkpoints")
            
        except Exception as e:
            print(f"❌ Error validating {mission_file}: {e}")
            validation_results[mission_file] = False
    
    # Overall validation result
    print("\n" + "=" * 50)
    print("📋 VALIDATION SUMMARY:")
    
    all_valid = True
    for file, valid in validation_results.items():
        status = "✅ PASS" if valid else "❌ FAIL"
        print(f"   {file}: {status}")
        if not valid:
            all_valid = False
    
    if all_valid:
        print("\n🎉 ALL CHECKPOINTS VALIDATED SUCCESSFULLY!")
        print("✅ Ready for 26-checkpoint competition mission")
        return True
    else:
        print("\n🚨 VALIDATION FAILED!")
        print("❌ Some checkpoints are missing or incorrectly implemented")
        return False

def validate_configuration():
    """Validate hardware configuration"""
    print("\n🔧 Validating hardware configuration...")
    
    config_file = 'config/hardware_config.conf'
    if not os.path.exists(config_file):
        print(f"❌ Configuration file not found: {config_file}")
        return False
    
    try:
        with open(config_file, 'r') as f:
            config_content = f.read()
        
        # Check essential configuration parameters
        essential_params = [
            'debug_mode',
            'indoor_altitude',
            'outdoor_altitude',
            'turn_direction',
            'connection_port',
            'waypoint_1_lat',
            'waypoint_1_lon'
        ]
        
        missing_params = []
        for param in essential_params:
            if param not in config_content:
                missing_params.append(param)
        
        if missing_params:
            print("❌ Missing essential configuration parameters:")
            for param in missing_params:
                print(f"   - {param}")
            return False
        else:
            print("✅ Hardware configuration validated")
            return True
            
    except Exception as e:
        print(f"❌ Error validating configuration: {e}")
        return False

def main():
    """Main validation function"""
    try:
        # Validate checkpoints
        checkpoints_valid = validate_checkpoints()
        
        # Validate configuration
        config_valid = validate_configuration()
        
        # Overall result
        if checkpoints_valid and config_valid:
            print("\n🏆 KAERTEI 2025 FAIO SYSTEM VALIDATION PASSED!")
            print("✅ Ready for competition!")
            sys.exit(0)
        else:
            print("\n🚨 SYSTEM VALIDATION FAILED!")
            print("❌ Please fix the issues above before competition")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Validation error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
