#!/usr/bin/env python3
"""
PX4 Waypoint Configuration System
Sistem konfigurasi waypoint yang fleksibel untuk PX4 navigation
Bisa setting berapa waypoint yang mau digunakan untuk setiap sequence
"""

import json
import os
from typing import List, Dict, Any

class PX4WaypointConfig:
    def __init__(self):
        self.config_file = os.path.expanduser("~/px4_waypoint_sequences.json")
        self.default_config = {
            "competition_venue": "simple_3wp",
            "waypoint_sequences": {
                "exit_to_pickup": {
                    "description": "From exit gate to pickup area (WP1)",
                    "waypoints": [1],
                    "enabled": True,
                    "search_after_reach": True
                },
                "pickup_to_drop": {
                    "description": "From pickup to dropzone (WP2)", 
                    "waypoints": [2],
                    "enabled": True,
                    "search_after_reach": False
                },
                "drop_to_finish": {
                    "description": "From dropzone to finish (WP3 - final)",
                    "waypoints": [3],
                    "enabled": True,
                    "search_after_reach": False,
                    "is_final": True
                }
            },
            "settings": {
                "waypoint_timeout": 120,
                "waypoint_radius": 3.0,
                "auto_mission_altitude": 3.0,
                "retry_attempts": 2,
                "search_while_moving": True,
                "pickup_descent_altitude": 1.0
            }
        }
        self.config = self.load_config()
        
    def load_config(self) -> Dict[str, Any]:
        """Load waypoint configuration from file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    # Merge with default config untuk backward compatibility
                    return self._merge_config(self.default_config, config)
            else:
                # Create default config file
                self.save_config(self.default_config)
                return self.default_config.copy()
        except Exception as e:
            print(f"Error loading config: {e}")
            return self.default_config.copy()
            
    def save_config(self, config: Dict[str, Any] = None) -> bool:
        """Save waypoint configuration to file"""
        try:
            config_to_save = config if config else self.config
            with open(self.config_file, 'w') as f:
                json.dump(config_to_save, f, indent=4)
            print(f"✅ Waypoint config saved to {self.config_file}")
            return True
        except Exception as e:
            print(f"❌ Error saving config: {e}")
            return False
            
    def _merge_config(self, default: Dict, loaded: Dict) -> Dict:
        """Merge loaded config with default config"""
        result = default.copy()
        for key, value in loaded.items():
            if isinstance(value, dict) and key in result:
                result[key] = self._merge_config(result[key], value)
            else:
                result[key] = value
        return result
        
    def get_sequence(self, sequence_name: str) -> List[int]:
        """Get waypoint sequence by name"""
        if sequence_name in self.config["waypoint_sequences"]:
            seq_data = self.config["waypoint_sequences"][sequence_name]
            if seq_data.get("enabled", False):
                return seq_data["waypoints"]
        return []
        
    def set_sequence(self, sequence_name: str, waypoints: List[int], 
                    description: str = "", enabled: bool = True) -> bool:
        """Set waypoint sequence"""
        try:
            self.config["waypoint_sequences"][sequence_name] = {
                "description": description or f"Custom sequence: {sequence_name}",
                "waypoints": waypoints,
                "enabled": enabled
            }
            return self.save_config()
        except Exception as e:
            print(f"❌ Error setting sequence {sequence_name}: {e}")
            return False
            
    def update_sequence_waypoints(self, sequence_name: str, waypoints: List[int]) -> bool:
        """Update waypoints for existing sequence"""
        if sequence_name in self.config["waypoint_sequences"]:
            self.config["waypoint_sequences"][sequence_name]["waypoints"] = waypoints
            return self.save_config()
        return False
        
    def enable_sequence(self, sequence_name: str, enabled: bool = True) -> bool:
        """Enable/disable sequence"""
        if sequence_name in self.config["waypoint_sequences"]:
            self.config["waypoint_sequences"][sequence_name]["enabled"] = enabled
            return self.save_config()
        return False
        
    def get_all_sequences(self) -> Dict[str, Dict]:
        """Get all waypoint sequences"""
        return self.config["waypoint_sequences"]
        
    def get_enabled_sequences(self) -> Dict[str, List[int]]:
        """Get only enabled sequences"""
        enabled = {}
        for name, seq_data in self.config["waypoint_sequences"].items():
            if seq_data.get("enabled", False):
                enabled[name] = seq_data["waypoints"]
        return enabled
        
    def set_venue_preset(self, venue: str):
        """Set venue-specific waypoint presets"""
        venue_presets = {
            "simple_3wp": {
                "exit_to_pickup": [1],
                "pickup_to_drop": [2], 
                "drop_to_finish": [3]
            },
            "indoor_test": {
                "exit_to_pickup": [1],
                "pickup_to_drop": [2],
                "drop_to_finish": [3]
            },
            "outdoor_test": {
                "exit_to_pickup": [1],
                "pickup_to_drop": [2],
                "drop_to_finish": [3]
            },
            "competition_venue": {
                "exit_to_pickup": [1],
                "pickup_to_drop": [2],
                "drop_to_finish": [3]
            }
        }
        
        if venue in venue_presets:
            preset = venue_presets[venue]
            self.config["competition_venue"] = venue
            
            for seq_name, waypoints in preset.items():
                if seq_name in self.config["waypoint_sequences"]:
                    self.config["waypoint_sequences"][seq_name]["waypoints"] = waypoints
                    
            self.save_config()
            print(f"✅ Applied venue preset: {venue}")
            return True
        else:
            print(f"❌ Unknown venue preset: {venue}")
            return False
            
    def print_config(self):
        """Print current configuration"""
        print("\n🛰️  PX4 WAYPOINT CONFIGURATION")
        print(f"Venue: {self.config.get('competition_venue', 'default')}")
        print("\nSequences:")
        
        for name, seq_data in self.config["waypoint_sequences"].items():
            status = "✅" if seq_data.get("enabled", False) else "❌"
            waypoints = seq_data.get("waypoints", [])
            description = seq_data.get("description", "")
            
            print(f"  {status} {name}: {waypoints}")
            print(f"      {description}")
            
        print(f"\nSettings:")
        settings = self.config.get("settings", {})
        for key, value in settings.items():
            print(f"  {key}: {value}")
        print()

def main():
    """CLI untuk konfigurasi waypoints"""
    import sys
    
    config = PX4WaypointConfig()
    
    if len(sys.argv) < 2:
        print("🛰️  PX4 Waypoint Configuration Tool")
        print("\nUsage:")
        print("  python px4_waypoint_config.py show                    - Show current config")
        print("  python px4_waypoint_config.py set <seq> <wp1,wp2>     - Set sequence waypoints")
        print("  python px4_waypoint_config.py enable <seq>            - Enable sequence")  
        print("  python px4_waypoint_config.py disable <seq>           - Disable sequence")
        print("  python px4_waypoint_config.py venue <venue>           - Apply venue preset")
        print("\nVenue presets: indoor_test, outdoor_test, competition_venue, simple_3wp")
        print("\nSequences: exit_to_pickup, pickup_to_drop, drop_to_finish")
        return
        
    command = sys.argv[1]
    
    if command == "show":
        config.print_config()
        
    elif command == "set" and len(sys.argv) >= 4:
        seq_name = sys.argv[2]
        waypoints_str = sys.argv[3]
        waypoints = [int(x.strip()) for x in waypoints_str.split(",")]
        
        if config.update_sequence_waypoints(seq_name, waypoints):
            print(f"✅ Updated {seq_name}: {waypoints}")
        else:
            print(f"❌ Failed to update {seq_name}")
            
    elif command == "enable" and len(sys.argv) >= 3:
        seq_name = sys.argv[2]
        if config.enable_sequence(seq_name, True):
            print(f"✅ Enabled {seq_name}")
            
    elif command == "disable" and len(sys.argv) >= 3:
        seq_name = sys.argv[2]
        if config.enable_sequence(seq_name, False):
            print(f"❌ Disabled {seq_name}")
            
    elif command == "venue" and len(sys.argv) >= 3:
        venue = sys.argv[2]
        config.set_venue_preset(venue)
        
    else:
        print("❌ Invalid command or missing arguments")

if __name__ == "__main__":
    main()
