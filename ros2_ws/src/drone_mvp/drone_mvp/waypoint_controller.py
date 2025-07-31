#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String, Bool
from mavros_msgs.msg import State


class WaypointControllerNode(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        
        # Initialize parameters with default waypoints
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoints.outdoor_pickup.latitude', -6.123456),
                ('waypoints.outdoor_pickup.longitude', 106.123456),
                ('waypoints.outdoor_pickup.altitude', 30.0),
                ('waypoints.outdoor_drop.latitude', -6.123457),
                ('waypoints.outdoor_drop.longitude', 106.123457),
                ('waypoints.outdoor_drop.altitude', 100.0),
                ('waypoints.landing_zone.latitude', -6.123458),
                ('waypoints.landing_zone.longitude', 106.123458),
                ('waypoints.landing_zone.altitude', 0.0),
                ('hover_distance_threshold', 97.0),
                ('waypoint_reached_threshold', 5.0),
                ('update_rate', 2.0)
            ]
        )
        
        # Load waypoints from parameters
        self.waypoints = {
            'outdoor_pickup': {
                'latitude': self.get_parameter('waypoints.outdoor_pickup.latitude').value,
                'longitude': self.get_parameter('waypoints.outdoor_pickup.longitude').value,
                'altitude': self.get_parameter('waypoints.outdoor_pickup.altitude').value
            },
            'outdoor_drop': {
                'latitude': self.get_parameter('waypoints.outdoor_drop.latitude').value,
                'longitude': self.get_parameter('waypoints.outdoor_drop.longitude').value,
                'altitude': self.get_parameter('waypoints.outdoor_drop.altitude').value
            },
            'landing_zone': {
                'latitude': self.get_parameter('waypoints.landing_zone.latitude').value,
                'longitude': self.get_parameter('waypoints.landing_zone.longitude').value,
                'altitude': self.get_parameter('waypoints.landing_zone.altitude').value
            }
        }
        
        # QoS profile for ArduPilot MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.distance_pub = self.create_publisher(Float32, '/waypoint/distance', qos_profile)
        self.waypoint_status_pub = self.create_publisher(String, '/waypoint/status', qos_profile)
        self.target_waypoint_pub = self.create_publisher(Point, '/waypoint/target', qos_profile)
        self.hover_trigger_pub = self.create_publisher(Bool, '/waypoint/hover_trigger', qos_profile)
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        self.mavros_state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, qos_profile)
        self.mission_state_sub = self.create_subscription(
            String, '/mission/state', self.mission_state_callback, qos_profile)
        
        # Alternative GPS topics
        self.gps_raw_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self.gps_callback, qos_profile)
        
        # State variables
        self.current_position = None
        self.current_mission_state = None
        self.target_waypoint = None
        self.last_distance = float('inf')
        self.hover_triggered = False
        self.mavros_connected = False
        self.flight_mode = "UNKNOWN"
        
        # Timer for distance calculation
        update_rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0 / update_rate, self.update_distance_callback)
        
        self.get_logger().info("Waypoint Controller Node initialized")
        self.log_waypoints()
    
    def log_waypoints(self):
        """Log configured waypoints"""
        self.get_logger().info("Configured waypoints:")
        for name, waypoint in self.waypoints.items():
            self.get_logger().info(f"  {name}: ({waypoint['latitude']:.6f}, {waypoint['longitude']:.6f}, {waypoint['altitude']:.1f})")
    
    def gps_callback(self, msg):
        """Handle GPS position updates"""
        try:
            if msg.status.status >= 0:  # Valid GPS fix
                self.current_position = {
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude
                }
        except Exception as e:
            self.get_logger().error(f"Error processing GPS data: {e}")
    
    def mavros_state_callback(self, msg):
        """Handle MAVROS state updates"""
        try:
            self.mavros_connected = msg.connected
            self.flight_mode = msg.mode
            
            if not msg.connected:
                self.get_logger().warn("MAVROS not connected to flight controller")
                
        except Exception as e:
            self.get_logger().error(f"Error processing MAVROS state: {e}")
    
    def vehicle_position_callback(self, msg):
        """Handle ArduPilot vehicle position updates (legacy method kept for compatibility)"""
        # This method is kept for compatibility but not used with MAVROS
        # Position data comes through /mavros/global_position/global
        pass
    
    def mission_state_callback(self, msg):
        """Handle mission state updates to determine target waypoint"""
        self.current_mission_state = msg.data
        
        # Determine target waypoint based on mission state
        previous_target = self.target_waypoint
        
        if msg.data in ['OUTDOOR_GPS_MISSION', 'OUTDOOR_MANUAL_FALLBACK']:
            self.target_waypoint = 'outdoor_pickup'
        elif msg.data in ['OUTDOOR_RETURN', 'OUTDOOR_HOVER_SEARCH']:
            self.target_waypoint = 'outdoor_drop'
        elif msg.data == 'LANDING':
            self.target_waypoint = 'landing_zone'
        else:
            self.target_waypoint = None
        
        # Reset hover trigger when target changes
        if previous_target != self.target_waypoint:
            self.hover_triggered = False
            if self.target_waypoint:
                self.get_logger().info(f"Target waypoint changed to: {self.target_waypoint}")
    
    def update_distance_callback(self):
        """Calculate and publish distance to target waypoint"""
        try:
            if self.current_position is None or self.target_waypoint is None:
                return
            
            if self.target_waypoint not in self.waypoints:
                self.get_logger().error(f"Unknown target waypoint: {self.target_waypoint}")
                return
            
            # Get target waypoint coordinates
            target = self.waypoints[self.target_waypoint]
            
            # Calculate distance
            distance = self.calculate_distance(
                self.current_position['latitude'],
                self.current_position['longitude'],
                target['latitude'],
                target['longitude']
            )
            
            # Calculate altitude difference
            altitude_diff = abs(self.current_position['altitude'] - target['altitude'])
            
            # Calculate 3D distance
            distance_3d = math.sqrt(distance**2 + altitude_diff**2)
            
            # Update last distance
            self.last_distance = distance
            
            # Publish distance
            distance_msg = Float32()
            distance_msg.data = distance
            self.distance_pub.publish(distance_msg)
            
            # Publish target waypoint for visualization
            target_msg = Point()
            target_msg.x = target['latitude']
            target_msg.y = target['longitude']
            target_msg.z = target['altitude']
            self.target_waypoint_pub.publish(target_msg)
            
            # Check for hover trigger (specific distance threshold for outdoor drop)
            if (self.target_waypoint == 'outdoor_drop' and 
                not self.hover_triggered and 
                distance <= self.get_parameter('hover_distance_threshold').value):
                
                self.hover_triggered = True
                hover_msg = Bool()
                hover_msg.data = True
                self.hover_trigger_pub.publish(hover_msg)
                
                self.get_logger().info(f"Hover trigger activated at {distance:.1f}m from outdoor drop waypoint")
            
            # Check if waypoint is reached
            waypoint_reached_threshold = self.get_parameter('waypoint_reached_threshold').value
            if distance <= waypoint_reached_threshold:
                status = f"REACHED_{self.target_waypoint.upper()}"
            else:
                status = f"APPROACHING_{self.target_waypoint.upper()}"
            
            # Publish status
            status_msg = String()
            status_msg.data = f"{status}_DIST_{distance:.1f}M"
            self.waypoint_status_pub.publish(status_msg)
            
            # Debug logging (reduced frequency)
            if hasattr(self, '_last_log_time'):
                if self.get_clock().now().nanoseconds - self._last_log_time > 5e9:  # 5 seconds
                    self.get_logger().debug(f"Distance to {self.target_waypoint}: {distance:.1f}m")
                    self._last_log_time = self.get_clock().now().nanoseconds
            else:
                self._last_log_time = self.get_clock().now().nanoseconds
                
        except Exception as e:
            self.get_logger().error(f"Error updating distance: {e}")
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates using Haversine formula"""
        try:
            # Convert latitude and longitude from degrees to radians
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            # Haversine formula
            dlat = lat2_rad - lat1_rad
            dlon = lon2_rad - lon1_rad
            
            a = (math.sin(dlat / 2) ** 2 + 
                 math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
            c = 2 * math.asin(math.sqrt(a))
            
            # Radius of Earth in meters
            r = 6371000
            
            # Calculate the distance
            distance = r * c
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f"Error calculating distance: {e}")
            return float('inf')
    
    def get_current_distance(self):
        """Get current distance to target waypoint"""
        return self.last_distance
    
    def get_target_waypoint_info(self):
        """Get information about current target waypoint"""
        if self.target_waypoint and self.target_waypoint in self.waypoints:
            return {
                'name': self.target_waypoint,
                'coordinates': self.waypoints[self.target_waypoint],
                'distance': self.last_distance
            }
        return None
    
    def set_custom_waypoint(self, name, latitude, longitude, altitude):
        """Set a custom waypoint (for dynamic waypoints)"""
        self.waypoints[name] = {
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude
        }
        self.get_logger().info(f"Custom waypoint '{name}' set: ({latitude:.6f}, {longitude:.6f}, {altitude:.1f})")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        waypoint_controller = WaypointControllerNode()
        rclpy.spin(waypoint_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'waypoint_controller' in locals():
            waypoint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
