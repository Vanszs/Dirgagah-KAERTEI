#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Waypoint Configuration Tool
Tool untuk mengkustomisasi koordinat waypoint untuk lokasi kompetisi
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

class WaypointConfigurator(Node):
    """Tool untuk mengkonfigurasi waypoint coordinates"""
    
    def __init__(self):
        super().__init__('waypoint_configurator')
        
        # Publisher untuk waypoint updates
        self.update_pub = self.create_publisher(String, '/waypoint_navigator/update_waypoint', 10)
        
        # Predefined waypoint sequences (default Jakarta coordinates)
        self.default_waypoints = {
            'exit_to_pickup': [
                {'name': 'exit_transition', 'lat': -6.365200, 'lon': 106.824800, 'alt': 30.0},
                {'name': 'approach_pickup', 'lat': -6.365000, 'lon': 106.825000, 'alt': 30.0},
                {'name': 'pickup_position', 'lat': -6.364900, 'lon': 106.825100, 'alt': 20.0}
            ],
            'pickup_to_drop': [
                {'name': 'departure_pickup', 'lat': -6.364900, 'lon': 106.825100, 'alt': 30.0},
                {'name': 'transit_to_drop', 'lat': -6.364700, 'lon': 106.825300, 'alt': 30.0},
                {'name': 'approach_dropzone', 'lat': -6.364500, 'lon': 106.825500, 'alt': 30.0},
                {'name': 'drop_position', 'lat': -6.364400, 'lon': 106.825600, 'alt': 20.0}
            ],
            'drop_to_finish': [
                {'name': 'departure_drop', 'lat': -6.364400, 'lon': 106.825600, 'alt': 30.0},
                {'name': 'transit_home', 'lat': -6.365000, 'lon': 106.825000, 'alt': 30.0},
                {'name': 'approach_home', 'lat': -6.365300, 'lon': 106.824700, 'alt': 30.0},
                {'name': 'landing_position', 'lat': -6.365500, 'lon': 106.824500, 'alt': 10.0}
            ]
        }
        
        self.get_logger().info("🛰️ Waypoint Configurator ready")
        self.get_logger().info("Use CLI commands to update waypoints for competition venue")
    
    def print_help(self):
        """Print usage help"""
        help_text = """
🛰️ KAERTEI 2025 FAIO - Waypoint Configuration Tool

USAGE:
    python3 waypoint_config.py <command> [arguments]

COMMANDS:
    show                           - Show all current waypoints
    show <sequence>                - Show specific sequence waypoints  
    update <sequence> <index> <lat> <lon> [alt]
                                   - Update specific waypoint
    save <filename>                - Save current config to file
    load <filename>                - Load config from file
    competition <venue_name>       - Load preset for competition venue

SEQUENCES:
    exit_to_pickup                 - From exit gate to pickup area
    pickup_to_drop                 - From pickup to dropzone
    drop_to_finish                 - From dropzone to landing

EXAMPLES:
    python3 waypoint_config.py show
    python3 waypoint_config.py show exit_to_pickup
    python3 waypoint_config.py update exit_to_pickup 0 -6.123456 106.789012 30.0
    python3 waypoint_config.py save competition_venue.json
    python3 waypoint_config.py competition bandung
"""
        print(help_text)
    
    def show_waypoints(self, sequence_name=None):
        """Show current waypoint configuration"""
        if sequence_name:
            if sequence_name in self.default_waypoints:
                print(f"\n🛰️ Waypoints for sequence: {sequence_name}")
                print("="*60)
                for i, wp in enumerate(self.default_waypoints[sequence_name]):
                    print(f"[{i}] {wp['name']:<20} | {wp['lat']:>10.6f} | {wp['lon']:>11.6f} | {wp['alt']:>6.1f}m")
            else:
                print(f"❌ Unknown sequence: {sequence_name}")
        else:
            print("\n🛰️ All Waypoint Sequences")
            print("="*80)
            for seq_name, waypoints in self.default_waypoints.items():
                print(f"\n📍 {seq_name.upper()}")
                print("-"*50)
                for i, wp in enumerate(waypoints):
                    print(f"[{i}] {wp['name']:<20} | {wp['lat']:>10.6f} | {wp['lon']:>11.6f} | {wp['alt']:>6.1f}m")
    
    def update_waypoint(self, sequence_name, waypoint_index, lat, lon, alt=None):
        """Update specific waypoint coordinates"""
        try:
            waypoint_index = int(waypoint_index)
            lat = float(lat)
            lon = float(lon)
            if alt is not None:
                alt = float(alt)
        except ValueError:
            print("❌ Invalid numeric values")
            return False
        
        if sequence_name not in self.default_waypoints:
            print(f"❌ Unknown sequence: {sequence_name}")
            return False
        
        waypoints = self.default_waypoints[sequence_name]
        if waypoint_index >= len(waypoints):
            print(f"❌ Waypoint index {waypoint_index} out of range (0-{len(waypoints)-1})")
            return False
        
        # Store old values
        old_wp = waypoints[waypoint_index].copy()
        
        # Update coordinates
        waypoints[waypoint_index]['lat'] = lat
        waypoints[waypoint_index]['lon'] = lon
        if alt is not None:
            waypoints[waypoint_index]['alt'] = alt
        
        print(f"✅ Updated waypoint {waypoint_index} in {sequence_name}")
        print(f"Old: {old_wp['lat']:>10.6f}, {old_wp['lon']:>11.6f}, {old_wp['alt']:>6.1f}m")
        print(f"New: {lat:>10.6f}, {lon:>11.6f}, {waypoints[waypoint_index]['alt']:>6.1f}m")
        
        return True
    
    def save_config(self, filename):
        """Save current waypoint configuration to file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.default_waypoints, f, indent=2)
            print(f"✅ Configuration saved to: {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save config: {e}")
            return False
    
    def load_config(self, filename):
        """Load waypoint configuration from file"""
        try:
            with open(filename, 'r') as f:
                self.default_waypoints = json.load(f)
            print(f"✅ Configuration loaded from: {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to load config: {e}")
            return False
    
    def load_competition_preset(self, venue_name):
        """Load preset coordinates for known competition venues"""
        presets = {
            'bandung': {
                'exit_to_pickup': [
                    {'name': 'exit_transition', 'lat': -6.914744, 'lon': 107.609810, 'alt': 30.0},
                    {'name': 'approach_pickup', 'lat': -6.914644, 'lon': 107.609910, 'alt': 30.0},
                    {'name': 'pickup_position', 'lat': -6.914544, 'lon': 107.610010, 'alt': 20.0}
                ],
                'pickup_to_drop': [
                    {'name': 'departure_pickup', 'lat': -6.914544, 'lon': 107.610010, 'alt': 30.0},
                    {'name': 'transit_to_drop', 'lat': -6.914444, 'lon': 107.610110, 'alt': 30.0},
                    {'name': 'approach_dropzone', 'lat': -6.914344, 'lon': 107.610210, 'alt': 30.0},
                    {'name': 'drop_position', 'lat': -6.914244, 'lon': 107.610310, 'alt': 20.0}
                ],
                'drop_to_finish': [
                    {'name': 'departure_drop', 'lat': -6.914244, 'lon': 107.610310, 'alt': 30.0},
                    {'name': 'transit_home', 'lat': -6.914444, 'lon': 107.610110, 'alt': 30.0},
                    {'name': 'approach_home', 'lat': -6.914644, 'lon': 107.609910, 'alt': 30.0},
                    {'name': 'landing_position', 'lat': -6.914744, 'lon': 107.609810, 'alt': 10.0}
                ]
            },
            'surabaya': {
                'exit_to_pickup': [
                    {'name': 'exit_transition', 'lat': -7.250445, 'lon': 112.768845, 'alt': 30.0},
                    {'name': 'approach_pickup', 'lat': -7.250345, 'lon': 112.768945, 'alt': 30.0},
                    {'name': 'pickup_position', 'lat': -7.250245, 'lon': 112.769045, 'alt': 20.0}
                ],
                'pickup_to_drop': [
                    {'name': 'departure_pickup', 'lat': -7.250245, 'lon': 112.769045, 'alt': 30.0},
                    {'name': 'transit_to_drop', 'lat': -7.250145, 'lon': 112.769145, 'alt': 30.0},
                    {'name': 'approach_dropzone', 'lat': -7.250045, 'lon': 112.769245, 'alt': 30.0},
                    {'name': 'drop_position', 'lat': -7.249945, 'lon': 112.769345, 'alt': 20.0}
                ],
                'drop_to_finish': [
                    {'name': 'departure_drop', 'lat': -7.249945, 'lon': 112.769345, 'alt': 30.0},
                    {'name': 'transit_home', 'lat': -7.250145, 'lon': 112.769145, 'alt': 30.0},
                    {'name': 'approach_home', 'lat': -7.250345, 'lon': 112.768945, 'alt': 30.0},
                    {'name': 'landing_position', 'lat': -7.250445, 'lon': 112.768845, 'alt': 10.0}
                ]
            }
        }
        
        if venue_name.lower() in presets:
            self.default_waypoints = presets[venue_name.lower()]
            print(f"✅ Loaded preset for venue: {venue_name}")
            print("📍 Waypoints updated for competition location")
            return True
        else:
            available_venues = list(presets.keys())
            print(f"❌ Unknown venue: {venue_name}")
            print(f"Available presets: {', '.join(available_venues)}")
            return False

def main():
    """Main CLI interface"""
    if len(sys.argv) < 2:
        configurator = WaypointConfigurator()
        configurator.print_help()
        return
    
    configurator = WaypointConfigurator()
    command = sys.argv[1].lower()
    
    if command == 'help' or command == '-h' or command == '--help':
        configurator.print_help()
    
    elif command == 'show':
        if len(sys.argv) > 2:
            configurator.show_waypoints(sys.argv[2])
        else:
            configurator.show_waypoints()
    
    elif command == 'update':
        if len(sys.argv) < 6:
            print("❌ Usage: update <sequence> <index> <lat> <lon> [alt]")
            return
        
        sequence = sys.argv[2]
        index = sys.argv[3]
        lat = sys.argv[4]
        lon = sys.argv[5]
        alt = sys.argv[6] if len(sys.argv) > 6 else None
        
        configurator.update_waypoint(sequence, index, lat, lon, alt)
    
    elif command == 'save':
        if len(sys.argv) < 3:
            print("❌ Usage: save <filename>")
            return
        
        configurator.save_config(sys.argv[2])
    
    elif command == 'load':
        if len(sys.argv) < 3:
            print("❌ Usage: load <filename>")
            return
        
        configurator.load_config(sys.argv[2])
    
    elif command == 'competition':
        if len(sys.argv) < 3:
            print("❌ Usage: competition <venue_name>")
            return
        
        configurator.load_competition_preset(sys.argv[2])
    
    else:
        print(f"❌ Unknown command: {command}")
        configurator.print_help()

if __name__ == '__main__':
    main()
