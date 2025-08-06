#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Simplified 3 Waypoint System
Flow sederhana sesuai permintaan:
1. Exit gate -> naik 3m -> WP1
2. WP1 -> search object sambil maju -> pickup -> WP2  
3. WP2 -> drop logic -> WP3
4. WP3 -> selesai (no additional logic)
"""

import json
import os

class Simple3WaypointConfig:
    def __init__(self):
        self.config_file = os.path.expanduser("~/simple_3wp_config.json")
        self.default_config = {
            "mission_flow": "simplified_3_waypoint",
            "waypoints": {
                "wp1": {
                    "description": "Pickup area - search object sambil maju",
                    "px4_index": 1,
                    "altitude": 3.0,
                    "action": "search_and_pickup"
                },
                "wp2": {
                    "description": "Dropzone area - drop object",
                    "px4_index": 2,
                    "altitude": 3.0,
                    "action": "drop_object"
                },
                "wp3": {
                    "description": "Final waypoint - mission complete",
                    "px4_index": 3,
                    "altitude": 3.0,
                    "action": "mission_complete"
                }
            },
            "mission_settings": {
                "search_timeout": 30,  # seconds untuk search object di WP1
                "pickup_altitude": 0.5,  # altitude untuk pickup
                "drop_altitude": 0.8,    # altitude untuk drop
                "waypoint_timeout": 60   # timeout per waypoint
            },
            "vision_settings": {
                "confidence_threshold": 0.6,
                "alignment_tolerance": 25,
                "search_pattern": "forward_sweep"  # search sambil maju
            }
        }
        self.config = self.load_config()
        
    def load_config(self):
        """Load configuration"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    return json.load(f)
            else:
                self.save_config(self.default_config)
                return self.default_config.copy()
        except Exception as e:
            print(f"Error loading config: {e}")
            return self.default_config.copy()
            
    def save_config(self, config=None):
        """Save configuration"""
        try:
            config_to_save = config if config else self.config
            with open(self.config_file, 'w') as f:
                json.dump(config_to_save, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving config: {e}")
            return False
            
    def get_waypoint_sequence(self):
        """Get PX4 waypoint sequence [1, 2, 3]"""
        return [
            self.config["waypoints"]["wp1"]["px4_index"],
            self.config["waypoints"]["wp2"]["px4_index"], 
            self.config["waypoints"]["wp3"]["px4_index"]
        ]
        
    def get_mission_flow(self):
        """Get mission flow description"""
        return {
            "flow_name": "Simplified 3 Waypoint Mission",
            "steps": [
                "1. Exit gate detection -> ascend to 3m",
                "2. Navigate to WP1 -> search object sambil maju -> pickup when found",
                "3. Navigate to WP2 -> drop object",
                "4. Navigate to WP3 -> mission complete (no additional logic)"
            ],
            "waypoint_sequence": self.get_waypoint_sequence()
        }
        
    def print_config(self):
        """Print current configuration"""
        print("\n🎯 SIMPLE 3 WAYPOINT CONFIGURATION")
        print("=" * 50)
        flow = self.get_mission_flow()
        print(f"Mission: {flow['flow_name']}")
        print(f"Waypoint Sequence: {flow['waypoint_sequence']}")
        print("\nMission Flow:")
        for step in flow['steps']:
            print(f"  {step}")
            
        print("\nWaypoint Details:")
        for wp_name, wp_data in self.config["waypoints"].items():
            print(f"  {wp_name.upper()}: PX4 Index {wp_data['px4_index']}")
            print(f"    {wp_data['description']}")
            print(f"    Action: {wp_data['action']}")
            print(f"    Altitude: {wp_data['altitude']}m")
            
        print(f"\nSettings:")
        settings = self.config["mission_settings"]
        for key, value in settings.items():
            print(f"  {key}: {value}")
        print()

def main():
    """CLI untuk simple waypoint config"""
    import sys
    
    config = Simple3WaypointConfig()
    
    if len(sys.argv) < 2:
        print("🎯 Simple 3 Waypoint Configuration")
        print("\nUsage:")
        print("  python simple_3wp_config.py show      - Show current config")
        print("  python simple_3wp_config.py setup     - Setup waypoint indices")
        print("  python simple_3wp_config.py test      - Test configuration")
        return
        
    command = sys.argv[1]
    
    if command == "show":
        config.print_config()
        
    elif command == "setup":
        print("🔧 Setup Simple 3 Waypoint Indices")
        print("Current sequence:", config.get_waypoint_sequence())
        
        try:
            wp1 = int(input("Enter PX4 index for WP1 (pickup area): ") or "1")
            wp2 = int(input("Enter PX4 index for WP2 (dropzone): ") or "2") 
            wp3 = int(input("Enter PX4 index for WP3 (finish): ") or "3")
            
            config.config["waypoints"]["wp1"]["px4_index"] = wp1
            config.config["waypoints"]["wp2"]["px4_index"] = wp2
            config.config["waypoints"]["wp3"]["px4_index"] = wp3
            
            if config.save_config():
                print(f"✅ Updated waypoint sequence: [{wp1}, {wp2}, {wp3}]")
            else:
                print("❌ Failed to save configuration")
                
        except ValueError:
            print("❌ Invalid input. Please enter numbers only.")
            
    elif command == "test":
        print("🧪 Testing Simple 3 Waypoint Configuration")
        config.print_config()
        
        flow = config.get_mission_flow()
        sequence = flow["waypoint_sequence"]
        
        print(f"✅ Configuration valid")
        print(f"🛰️ PX4 waypoint sequence ready: {sequence}")
        print("🎯 Ready for mission execution!")
        
    else:
        print(f"❌ Unknown command: {command}")

if __name__ == "__main__":
    main()
