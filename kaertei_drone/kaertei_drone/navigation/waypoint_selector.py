#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Intelligent Waypoint Selection
Handles left/right path selection based on competition setup and obstacles
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import configparser
import os

class WaypointSelector(Node):
    """
    Intelligent waypoint selection for indoor/outdoor navigation
    Chooses optimal path based on obstacles and mission requirements
    """
    
    def __init__(self):
        super().__init__('waypoint_selector')
        
        # Load configuration
        self.load_configuration()
        
        # Current state
        self.current_position = Point()
        self.obstacle_map = {}
        self.selected_path = "unknown"
        self.path_confirmed = False
        
        # Indoor waypoints (left/right paths)
        self.indoor_waypoints = {
            'left': [
                {'x': 2.0, 'y': 2.0, 'z': 1.0, 'description': 'Indoor WP1 Left'},
                {'x': 4.0, 'y': 2.0, 'z': 1.0, 'description': 'Indoor WP2 Left'},
                {'x': 6.0, 'y': 1.0, 'z': 1.0, 'description': 'Exit Gate Left'}
            ],
            'right': [
                {'x': 2.0, 'y': -2.0, 'z': 1.0, 'description': 'Indoor WP1 Right'},
                {'x': 4.0, 'y': -2.0, 'z': 1.0, 'description': 'Indoor WP2 Right'},
                {'x': 6.0, 'y': -1.0, 'z': 1.0, 'description': 'Exit Gate Right'}
            ],
            'center': [
                {'x': 2.0, 'y': 0.0, 'z': 1.0, 'description': 'Indoor WP1 Center'},
                {'x': 4.0, 'y': 0.0, 'z': 1.0, 'description': 'Indoor WP2 Center'},
                {'x': 6.0, 'y': 0.0, 'z': 1.0, 'description': 'Exit Gate Center'}
            ]
        }
        
        # Publishers
        self.path_selection_pub = self.create_publisher(String, '/waypoint/path_selection', 10)
        self.current_waypoint_pub = self.create_publisher(Point, '/waypoint/current_target', 10)
        self.path_status_pub = self.create_publisher(String, '/waypoint/path_status', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.position_sub = self.create_subscription(
            Point, '/drone/position', self.position_callback, 10)
        self.selection_request_sub = self.create_subscription(
            String, '/waypoint/selection_request', self.selection_request_callback, 10)
        
        # Path analysis timer
        self.analysis_timer = self.create_timer(2.0, self.analyze_path_options)
        
        self.get_logger().info("🧭 Waypoint Selector initialized")
        self.get_logger().info(f"🎯 Turn direction from config: {self.turn_direction}")
    
    def load_configuration(self):
        """Load waypoint configuration from hardware_config.conf"""
        config = configparser.ConfigParser()
        config_path = os.path.join(os.path.dirname(__file__), '../config/hardware_config.conf')
        
        try:
            config.read(config_path)
            
            # Load mission settings
            self.turn_direction = config.get('mission', 'turn_direction', fallback='right')
            self.indoor_altitude = float(config.get('mission', 'indoor_altitude', fallback=1.0))
            self.outdoor_altitude = float(config.get('mission', 'outdoor_altitude', fallback=3.0))
            
            # Load GPS waypoints
            self.outdoor_waypoints = []
            for i in range(1, 4):
                try:
                    lat = float(config.get('GPS_WAYPOINTS', f'waypoint_{i}_lat'))
                    lon = float(config.get('GPS_WAYPOINTS', f'waypoint_{i}_lon'))
                    alt = float(config.get('GPS_WAYPOINTS', f'waypoint_{i}_alt'))
                    
                    self.outdoor_waypoints.append({
                        'lat': lat, 'lon': lon, 'alt': alt,
                        'description': f'Outdoor Waypoint {i}'
                    })
                except:
                    self.get_logger().warn(f"⚠️  Could not load outdoor waypoint {i}")
            
            self.get_logger().info(f"✅ Configuration loaded: {len(self.outdoor_waypoints)} outdoor waypoints")
            
        except Exception as e:
            self.get_logger().error(f"❌ Configuration loading failed: {e}")
            # Use defaults
            self.turn_direction = 'right'
            self.indoor_altitude = 1.0
            self.outdoor_altitude = 3.0
            self.outdoor_waypoints = []
    
    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter invalid readings
        valid_indices = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        if len(ranges) == 0:
            return
        
        # Analyze path clearance in different directions
        self.obstacle_map = {
            'front': self.get_path_clearance(ranges, angles, -0.2, 0.2),     # ±11 degrees
            'left': self.get_path_clearance(ranges, angles, 0.5, 1.0),       # 30-60 degrees  
            'right': self.get_path_clearance(ranges, angles, -1.0, -0.5),    # -60 to -30 degrees
            'front_left': self.get_path_clearance(ranges, angles, 0.2, 0.5), # 11-30 degrees
            'front_right': self.get_path_clearance(ranges, angles, -0.5, -0.2) # -30 to -11 degrees
        }
    
    def get_path_clearance(self, ranges, angles, angle_min, angle_max):
        """Calculate path clearance in specified angle range"""
        mask = (angles >= angle_min) & (angles <= angle_max)
        if not np.any(mask):
            return {'min_distance': 0.0, 'avg_distance': 0.0, 'clear': False}
        
        sector_ranges = ranges[mask]
        min_distance = np.min(sector_ranges)
        avg_distance = np.mean(sector_ranges)
        clear = min_distance > 2.0  # 2 meter clearance required
        
        return {
            'min_distance': float(min_distance),
            'avg_distance': float(avg_distance),
            'clear': clear
        }
    
    def position_callback(self, msg):
        """Update current drone position"""
        self.current_position = msg
    
    def selection_request_callback(self, msg):
        """Handle path selection requests"""
        request = msg.data.upper()
        
        if request == "SELECT_INDOOR_PATH":
            self.select_indoor_path()
        elif request == "SELECT_OUTDOOR_PATH":
            self.select_outdoor_path()
        elif request == "CONFIRM_PATH":
            self.confirm_current_path()
        elif request == "RESET_SELECTION":
            self.reset_path_selection()
        else:
            self.get_logger().warn(f"⚠️  Unknown selection request: {request}")
    
    def select_indoor_path(self):
        """Intelligent indoor path selection based on obstacles and configuration"""
        self.get_logger().info("🧭 Analyzing indoor path options...")
        
        # Get obstacle information
        obstacles = self.obstacle_map
        
        # Score different path options
        path_scores = {}
        
        # Left path scoring
        left_score = 100
        if 'left' in obstacles and not obstacles['left']['clear']:
            left_score -= 50
        if 'front_left' in obstacles and not obstacles['front_left']['clear']:
            left_score -= 30
        
        # Right path scoring  
        right_score = 100
        if 'right' in obstacles and not obstacles['right']['clear']:
            right_score -= 50
        if 'front_right' in obstacles and not obstacles['front_right']['clear']:
            right_score -= 30
        
        # Center path scoring
        center_score = 80  # Generally less preferred
        if 'front' in obstacles and not obstacles['front']['clear']:
            center_score -= 60
        
        # Apply configuration preference
        if self.turn_direction == 'left':
            left_score += 20
        elif self.turn_direction == 'right':
            right_score += 20
        
        path_scores = {'left': left_score, 'right': right_score, 'center': center_score}
        
        # Select best path
        best_path = max(path_scores, key=path_scores.get)
        best_score = path_scores[best_path]
        
        self.selected_path = best_path
        self.path_confirmed = False
        
        self.get_logger().info(f"🎯 Selected indoor path: {best_path.upper()} (score: {best_score})")
        self.get_logger().info(f"   Path scores: {path_scores}")
        
        # Publish selection
        self.publish_path_selection(f"INDOOR_PATH_{best_path.upper()}")
        self.publish_path_status(f"PATH_SELECTED_{best_path.upper()}_SCORE_{best_score}")
        
        # Publish first waypoint
        if best_path in self.indoor_waypoints:
            first_wp = self.indoor_waypoints[best_path][0]
            wp_point = Point()
            wp_point.x = first_wp['x']
            wp_point.y = first_wp['y'] 
            wp_point.z = first_wp['z']
            self.current_waypoint_pub.publish(wp_point)
            
            self.get_logger().info(f"📍 First waypoint: {first_wp['description']} ({first_wp['x']}, {first_wp['y']}, {first_wp['z']})")
    
    def select_outdoor_path(self):
        """Select outdoor GPS waypoint sequence"""
        self.get_logger().info("🛰️  Selecting outdoor GPS waypoint sequence...")
        
        if not self.outdoor_waypoints:
            self.get_logger().error("❌ No outdoor waypoints configured!")
            self.publish_path_status("ERROR_NO_OUTDOOR_WAYPOINTS")
            return
        
        # For outdoor, we typically use the predefined GPS sequence
        self.selected_path = "gps_sequence"
        self.path_confirmed = False
        
        # Validate GPS waypoints
        valid_waypoints = []
        for i, wp in enumerate(self.outdoor_waypoints):
            if self.validate_gps_waypoint(wp):
                valid_waypoints.append(wp)
            else:
                self.get_logger().warn(f"⚠️  Invalid GPS waypoint {i+1}: {wp}")
        
        if len(valid_waypoints) < 2:
            self.get_logger().error("❌ Insufficient valid GPS waypoints!")
            self.publish_path_status("ERROR_INSUFFICIENT_GPS_WAYPOINTS")
            return
        
        self.outdoor_waypoints = valid_waypoints
        
        self.get_logger().info(f"✅ Outdoor path selected: {len(valid_waypoints)} GPS waypoints")
        for i, wp in enumerate(valid_waypoints):
            self.get_logger().info(f"   WP{i+1}: ({wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']:.1f}m)")
        
        # Publish selection
        self.publish_path_selection(f"OUTDOOR_PATH_GPS_SEQUENCE_{len(valid_waypoints)}_WAYPOINTS")
        self.publish_path_status(f"GPS_SEQUENCE_SELECTED_{len(valid_waypoints)}_WAYPOINTS")
    
    def validate_gps_waypoint(self, waypoint):
        """Validate GPS waypoint coordinates"""
        try:
            lat = float(waypoint['lat'])
            lon = float(waypoint['lon'])
            alt = float(waypoint['alt'])
            
            # Basic coordinate validation (Jakarta area)
            if not (-7.0 <= lat <= -6.0):
                return False
            if not (106.0 <= lon <= 107.0):
                return False
            if not (0.0 <= alt <= 100.0):
                return False
            
            return True
        except:
            return False
    
    def confirm_current_path(self):
        """Confirm and lock the currently selected path"""
        if self.selected_path == "unknown":
            self.get_logger().warn("⚠️  No path selected to confirm")
            return
        
        self.path_confirmed = True
        
        self.get_logger().info(f"✅ Path confirmed: {self.selected_path.upper()}")
        self.publish_path_status(f"PATH_CONFIRMED_{self.selected_path.upper()}")
    
    def reset_path_selection(self):
        """Reset path selection for reanalysis"""
        self.selected_path = "unknown"
        self.path_confirmed = False
        
        self.get_logger().info("🔄 Path selection reset")
        self.publish_path_status("SELECTION_RESET")
    
    def analyze_path_options(self):
        """Continuous analysis of path options"""
        if self.path_confirmed:
            return
        
        # Publish current analysis
        if self.obstacle_map:
            analysis = []
            for direction, data in self.obstacle_map.items():
                status = "CLEAR" if data['clear'] else "BLOCKED"
                analysis.append(f"{direction}:{status}:{data['min_distance']:.1f}m")
            
            analysis_str = "|".join(analysis)
            self.publish_path_status(f"ANALYSIS_{analysis_str}")
    
    def get_indoor_waypoints_for_path(self, path_name):
        """Get waypoint sequence for selected indoor path"""
        if path_name.lower() in self.indoor_waypoints:
            return self.indoor_waypoints[path_name.lower()]
        return []
    
    def get_outdoor_waypoints(self):
        """Get outdoor GPS waypoint sequence"""
        return self.outdoor_waypoints
    
    def publish_path_selection(self, selection):
        """Publish path selection result"""
        msg = String()
        msg.data = selection
        self.path_selection_pub.publish(msg)
    
    def publish_path_status(self, status):
        """Publish current path analysis status"""
        msg = String()
        msg.data = status
        self.path_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    waypoint_selector = WaypointSelector()
    
    try:
        rclpy.spin(waypoint_selector)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_selector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
