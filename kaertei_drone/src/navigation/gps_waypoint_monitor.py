#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Bool, Float32, Int32
from kaertei_drone.hardware_config import HardwareConfig


class GPSWaypointMonitorNode(Node):
    def __init__(self):
        super().__init__('gps_waypoint_monitor')
        
        # Load hardware config
        self.hw_config = HardwareConfig()
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_radius', 3.0),       # meters - radius for waypoint reached
                ('update_rate', 2.0),           # Hz
                ('min_gps_accuracy', 5.0),      # meters - required GPS accuracy
                ('waypoint_timeout', 60.0),     # seconds - timeout for reaching waypoint
                ('enable_rtk_mode', False),     # Enable RTK GPS if available
                ('outdoor_altitude_hold', 30.0), # meters - default outdoor altitude
            ]
        )
        
        # QoS profile for GPS data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers - Waypoint navigation status
        self.waypoint_status_pub = self.create_publisher(String, '/gps/waypoint_status', qos_profile)
        self.waypoint_reached_pub = self.create_publisher(Bool, '/gps/waypoint_reached', qos_profile)
        self.waypoint_distance_pub = self.create_publisher(Float32, '/gps/waypoint_distance', qos_profile)
        self.waypoint_bearing_pub = self.create_publisher(Float32, '/gps/waypoint_bearing', qos_profile)
        self.navigation_command_pub = self.create_publisher(String, '/gps/navigation_command', qos_profile)
        self.current_waypoint_pub = self.create_publisher(Int32, '/gps/current_waypoint', qos_profile)
        
        # Publishers - GPS quality monitoring
        self.gps_accuracy_pub = self.create_publisher(Float32, '/gps/accuracy', qos_profile)
        self.gps_fix_quality_pub = self.create_publisher(String, '/gps/fix_quality', qos_profile)
        self.satellites_count_pub = self.create_publisher(Int32, '/gps/satellites_count', qos_profile)
        
        # Subscribers - GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        self.gps_raw_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self.gps_raw_callback, qos_profile)
        
        # Subscribers - Mission commands
        self.waypoint_command_sub = self.create_subscription(
            Point, '/mission/goto_waypoint', self.waypoint_command_callback, qos_profile)
        self.waypoint_index_sub = self.create_subscription(
            Int32, '/mission/set_waypoint_index', self.waypoint_index_callback, qos_profile)
        
        # Load GPS waypoints from hardware config
        self.waypoints = self.load_waypoints_from_config()
        
        # GPS state
        self.current_gps_position = None
        self.gps_accuracy = 999.0  # Start with poor accuracy
        self.satellites_visible = 0
        self.gps_fix_type = 0
        self.last_gps_update = None
        
        # Waypoint navigation state
        self.target_waypoint = None
        self.current_waypoint_index = 0
        self.waypoint_start_time = None
        self.last_distance_to_waypoint = None
        self.waypoint_reached = False
        self.navigation_active = False
        
        # Position history for accuracy estimation
        self.position_history = []
        self.max_history_size = 10
        
        # Timer for monitoring
        update_rate = self.get_parameter('update_rate').value
        self.monitor_timer = self.create_timer(1.0 / update_rate, self.monitor_callback)
        
        self.get_logger().info("GPS Waypoint Monitor Node initialized")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from config")
        
        # Log waypoint coordinates
        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f"Waypoint {i}: {waypoint['lat']:.6f}, {waypoint['lon']:.6f}, {waypoint['alt']:.1f}m")
    
    def load_waypoints_from_config(self):
        """Load waypoints from hardware configuration"""
        waypoints = []
        
        try:
            # Get mission waypoints from hardware config
            waypoint_1 = self.hw_config.get_waypoint_1()
            waypoint_2 = self.hw_config.get_waypoint_2()
            waypoint_3 = self.hw_config.get_waypoint_3()
            home = self.hw_config.get_home_position()
            
            waypoints = [waypoint_1, waypoint_2, waypoint_3, home]
            
            # Add pickup/drop locations
            outdoor_pickup = self.hw_config.get_outdoor_pickup_coords()
            outdoor_drop = self.hw_config.get_outdoor_drop_coords()
            
            waypoints.extend([outdoor_pickup, outdoor_drop])
            
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints from config: {e}")
            # Fallback waypoints for testing
            waypoints = [
                {'lat': -6.365000, 'lon': 106.825000, 'alt': 30.0},
                {'lat': -6.364500, 'lon': 106.825500, 'alt': 30.0},
                {'lat': -6.364000, 'lon': 106.826000, 'alt': 30.0},
                {'lat': -6.365500, 'lon': 106.824500, 'alt': 0.0},  # Home
            ]
        
        return waypoints
    
    def gps_callback(self, msg):
        """Handle GPS position updates (filtered)"""
        self.process_gps_data(msg, is_raw=False)
    
    def gps_raw_callback(self, msg):
        """Handle raw GPS position updates"""
        self.process_gps_data(msg, is_raw=True)
    
    def process_gps_data(self, msg, is_raw=False):
        """Process GPS data from both filtered and raw sources"""
        try:
            # Use filtered GPS by default, raw as backup
            if not is_raw or self.current_gps_position is None:
                
                # Check GPS fix quality
                if msg.status.status < 0:
                    self.get_logger().warn("No GPS fix available")
                    return
                
                # Update current position
                self.current_gps_position = {
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'timestamp': time.time()
                }
                
                # Update GPS quality metrics
                self.gps_fix_type = msg.status.status
                if hasattr(msg.status, 'satellites_visible'):
                    self.satellites_visible = msg.status.satellites_visible
                
                # Estimate accuracy from covariance matrix
                if len(msg.position_covariance) >= 9:
                    # Position covariance diagonal elements
                    var_x = msg.position_covariance[0]
                    var_y = msg.position_covariance[4]
                    var_z = msg.position_covariance[8]
                    
                    # Horizontal accuracy estimate
                    self.gps_accuracy = math.sqrt(var_x + var_y)
                else:
                    # Fallback accuracy estimation based on fix type
                    if self.gps_fix_type >= 2:  # 3D fix
                        self.gps_accuracy = 3.0  # Assume 3m accuracy
                    else:
                        self.gps_accuracy = 10.0  # Poor accuracy
                
                # Add to position history for stability analysis
                self.position_history.append(self.current_gps_position.copy())
                if len(self.position_history) > self.max_history_size:
                    self.position_history.pop(0)
                
                self.last_gps_update = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Error processing GPS data: {e}")
    
    def waypoint_command_callback(self, msg):
        """Handle waypoint navigation commands"""
        try:
            # Set target waypoint from Point message
            self.target_waypoint = {
                'lat': msg.x,  # Using x as latitude
                'lon': msg.y,  # Using y as longitude  
                'alt': msg.z   # Using z as altitude
            }
            
            self.navigation_active = True
            self.waypoint_start_time = time.time()
            self.waypoint_reached = False
            
            self.get_logger().info(f"New waypoint target: {msg.x:.6f}, {msg.y:.6f}, {msg.z:.1f}m")
            
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint command: {e}")
    
    def waypoint_index_callback(self, msg):
        """Handle waypoint index selection"""
        try:
            index = msg.data
            if 0 <= index < len(self.waypoints):
                self.current_waypoint_index = index
                self.target_waypoint = self.waypoints[index].copy()
                self.navigation_active = True
                self.waypoint_start_time = time.time()
                self.waypoint_reached = False
                
                waypoint = self.waypoints[index]
                self.get_logger().info(f"Navigating to waypoint {index}: {waypoint['lat']:.6f}, {waypoint['lon']:.6f}, {waypoint['alt']:.1f}m")
            else:
                self.get_logger().warn(f"Invalid waypoint index: {index}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint index: {e}")
    
    def monitor_callback(self):
        """Main monitoring loop"""
        try:
            current_time = time.time()
            
            # Check GPS data freshness
            if (self.last_gps_update is None or 
                current_time - self.last_gps_update > 5.0):
                self.publish_gps_quality("NO_GPS_DATA")
                self.publish_waypoint_status("GPS_UNAVAILABLE")
                return
            
            # Publish GPS quality information
            self.publish_gps_quality_info()
            
            # Handle waypoint navigation if active
            if self.navigation_active and self.target_waypoint is not None:
                self.handle_waypoint_navigation(current_time)
            else:
                self.publish_waypoint_status("IDLE")
            
        except Exception as e:
            self.get_logger().error(f"Error in monitor callback: {e}")
    
    def handle_waypoint_navigation(self, current_time):
        """Handle active waypoint navigation"""
        try:
            if self.current_gps_position is None:
                self.publish_waypoint_status("WAITING_GPS")
                return
            
            # Check GPS accuracy requirement
            min_accuracy = self.get_parameter('min_gps_accuracy').value
            if self.gps_accuracy > min_accuracy:
                self.publish_waypoint_status("GPS_ACCURACY_POOR")
                self.get_logger().warn(f"GPS accuracy {self.gps_accuracy:.1f}m > required {min_accuracy:.1f}m")
                return
            
            # Calculate distance and bearing to target
            distance = self.calculate_distance_to_waypoint()
            bearing = self.calculate_bearing_to_waypoint()
            
            # Check if waypoint reached
            waypoint_radius = self.get_parameter('waypoint_radius').value
            if distance <= waypoint_radius:
                if not self.waypoint_reached:
                    self.waypoint_reached = True
                    self.get_logger().info(f"Waypoint reached! Distance: {distance:.1f}m")
                
                self.publish_waypoint_status("WAYPOINT_REACHED")
                self.publish_waypoint_reached(True)
            else:
                self.waypoint_reached = False
                self.publish_waypoint_reached(False)
                
                # Check for timeout
                waypoint_timeout = self.get_parameter('waypoint_timeout').value
                if (self.waypoint_start_time is not None and 
                    current_time - self.waypoint_start_time > waypoint_timeout):
                    self.publish_waypoint_status("WAYPOINT_TIMEOUT")
                    self.get_logger().warn(f"Waypoint navigation timeout after {waypoint_timeout}s")
                else:
                    self.publish_waypoint_status("NAVIGATING")
                    
                    # Generate navigation command
                    nav_command = self.generate_navigation_command(distance, bearing)
                    self.publish_navigation_command(nav_command)
            
            # Publish distance and bearing
            self.publish_waypoint_distance(distance)
            self.publish_waypoint_bearing(bearing)
            self.publish_current_waypoint_index()
            
            self.last_distance_to_waypoint = distance
            
        except Exception as e:
            self.get_logger().error(f"Error in waypoint navigation: {e}")
    
    def calculate_distance_to_waypoint(self):
        """Calculate distance to current target waypoint"""
        if self.current_gps_position is None or self.target_waypoint is None:
            return 999.0
        
        return self.calculate_distance(
            self.current_gps_position['latitude'],
            self.current_gps_position['longitude'], 
            self.target_waypoint['lat'],
            self.target_waypoint['lon']
        )
    
    def calculate_bearing_to_waypoint(self):
        """Calculate bearing to current target waypoint"""
        if self.current_gps_position is None or self.target_waypoint is None:
            return 0.0
        
        return self.calculate_bearing(
            self.current_gps_position['latitude'],
            self.current_gps_position['longitude'],
            self.target_waypoint['lat'], 
            self.target_waypoint['lon']
        )
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates using Haversine formula"""
        try:
            # Convert to radians
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
            
            # Earth radius in meters
            distance = 6371000 * c
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f"Error calculating distance: {e}")
            return 999.0
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two GPS coordinates"""
        try:
            # Convert to radians
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            # Calculate bearing
            dlon = lon2_rad - lon1_rad
            
            y = math.sin(dlon) * math.cos(lat2_rad)
            x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
                 math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
            
            bearing_rad = math.atan2(y, x)
            
            # Convert to degrees and normalize to 0-360
            bearing_deg = math.degrees(bearing_rad)
            bearing_deg = (bearing_deg + 360) % 360
            
            return bearing_deg
            
        except Exception as e:
            self.get_logger().error(f"Error calculating bearing: {e}")
            return 0.0
    
    def generate_navigation_command(self, distance, bearing):
        """Generate navigation command based on distance and bearing"""
        try:
            if distance < 5.0:
                command = f"FINAL_APPROACH,{bearing:.0f}"
            elif distance < 20.0:
                command = f"APPROACH,{bearing:.0f}"
            else:
                command = f"NAVIGATE,{bearing:.0f}"
            
            return command
            
        except Exception as e:
            self.get_logger().error(f"Error generating navigation command: {e}")
            return "ERROR"
    
    # ===========================================
    # PUBLISHERS
    # ===========================================
    
    def publish_gps_quality_info(self):
        """Publish GPS quality information"""
        self.gps_accuracy_pub.publish(Float32(data=self.gps_accuracy))
        self.satellites_count_pub.publish(Int32(data=self.satellites_visible))
        
        # GPS fix quality string
        if self.gps_fix_type >= 4:  # RTK fix
            quality = "RTK_FIXED"
        elif self.gps_fix_type >= 3:  # RTK float
            quality = "RTK_FLOAT" 
        elif self.gps_fix_type >= 2:  # 3D fix
            quality = "3D_FIX"
        elif self.gps_fix_type >= 1:  # 2D fix
            quality = "2D_FIX"
        else:
            quality = "NO_FIX"
        
        self.gps_fix_quality_pub.publish(String(data=quality))
    
    def publish_gps_quality(self, status):
        """Publish GPS quality status"""
        self.gps_fix_quality_pub.publish(String(data=status))
    
    def publish_waypoint_status(self, status):
        """Publish waypoint navigation status"""
        self.waypoint_status_pub.publish(String(data=status))
    
    def publish_waypoint_reached(self, reached):
        """Publish waypoint reached status"""
        self.waypoint_reached_pub.publish(Bool(data=reached))
    
    def publish_waypoint_distance(self, distance):
        """Publish distance to waypoint"""
        self.waypoint_distance_pub.publish(Float32(data=distance))
    
    def publish_waypoint_bearing(self, bearing):
        """Publish bearing to waypoint"""
        self.waypoint_bearing_pub.publish(Float32(data=bearing))
    
    def publish_navigation_command(self, command):
        """Publish navigation command"""
        self.navigation_command_pub.publish(String(data=command))
    
    def publish_current_waypoint_index(self):
        """Publish current waypoint index"""
        self.current_waypoint_pub.publish(Int32(data=self.current_waypoint_index))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        gps_waypoint_monitor = GPSWaypointMonitorNode()
        rclpy.spin(gps_waypoint_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gps_waypoint_monitor' in locals():
            gps_waypoint_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
