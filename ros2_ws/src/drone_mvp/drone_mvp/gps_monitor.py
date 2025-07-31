#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, TwistStamped
from std_msgs.msg import Bool, String, Float32
from mavros_msgs.msg import State


class GPSMonitorNode(Node):
    def __init__(self):
        super().__init__('gps_monitor')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('movement_threshold', 2.0),  # meters
                ('stuck_timeout', 10.0),      # seconds
                ('update_rate', 1.0),         # Hz
                ('min_satellites', 6),
                ('min_hdop', 2.0)
            ]
        )
        
        # QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.moving_status_pub = self.create_publisher(Bool, '/gps/moving_status', qos_profile)
        self.gps_quality_pub = self.create_publisher(String, '/gps/quality_status', qos_profile)
        self.distance_traveled_pub = self.create_publisher(Float32, '/gps/distance_traveled', qos_profile)
        self.speed_pub = self.create_publisher(Float32, '/gps/ground_speed', qos_profile)
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        self.mavros_state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, qos_profile)
        
        # Alternative GPS topics
        self.gps_raw_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self.gps_callback, qos_profile)
        
        # GPS monitoring state
        self.current_gps_position = None
        self.previous_gps_position = None
        self.last_movement_time = time.time()
        self.total_distance_traveled = 0.0
        self.last_position_update_time = None
        self.current_speed = 0.0
        
        # GPS quality parameters
        self.satellites_visible = 0
        self.hdop = 99.0  # High value = poor accuracy
        self.gps_fix_type = 0
        self.mavros_connected = False
        self.flight_mode = "UNKNOWN"
        
        # Movement detection
        self.is_moving = False
        self.position_history = []  # Store recent positions for movement analysis
        self.max_history_size = 5
        
        # Timer for monitoring
        update_rate = self.get_parameter('update_rate').value
        self.monitor_timer = self.create_timer(1.0 / update_rate, self.monitor_callback)
        
        self.get_logger().info("GPS Monitor Node initialized")
    
    def gps_callback(self, msg):
        """Handle GPS position updates"""
        try:
            # Check if GPS fix is valid
            if msg.status.status < 0:  # No GPS fix
                self.get_logger().warn("No GPS fix available")
                return
            
            # Update current position
            self.previous_gps_position = self.current_gps_position
            self.current_gps_position = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'timestamp': time.time()
            }
            
            # Update GPS quality info
            self.satellites_visible = getattr(msg.status, 'satellites_visible', 0)
            
            # Calculate movement if we have previous position
            if self.previous_gps_position is not None:
                distance = self.calculate_distance(
                    self.previous_gps_position['latitude'],
                    self.previous_gps_position['longitude'],
                    self.current_gps_position['latitude'],
                    self.current_gps_position['longitude']
                )
                
                # Update total distance
                self.total_distance_traveled += distance
                
                # Calculate speed
                time_diff = (self.current_gps_position['timestamp'] - 
                           self.previous_gps_position['timestamp'])
                if time_diff > 0:
                    self.current_speed = distance / time_diff
                
                # Check for movement
                movement_threshold = self.get_parameter('movement_threshold').value
                if distance > movement_threshold:
                    self.is_moving = True
                    self.last_movement_time = time.time()
                    self.get_logger().debug(f"Movement detected: {distance:.2f}m")
            
            # Add to position history
            self.position_history.append(self.current_gps_position.copy())
            if len(self.position_history) > self.max_history_size:
                self.position_history.pop(0)
            
            self.last_position_update_time = time.time()
            
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
    
    def monitor_callback(self):
        """Main monitoring loop"""
        try:
            current_time = time.time()
            
            # Check if GPS data is recent
            if (self.last_position_update_time is None or 
                current_time - self.last_position_update_time > 5.0):
                self.get_logger().warn("No recent GPS data")
                self.publish_movement_status(False)
                self.publish_gps_quality("NO_GPS_DATA")
                return
            
            # Check for stuck condition
            stuck_timeout = self.get_parameter('stuck_timeout').value
            time_since_movement = current_time - self.last_movement_time
            
            if time_since_movement > stuck_timeout:
                self.is_moving = False
                self.get_logger().warn(f"GPS stuck - no movement for {time_since_movement:.1f}s")
            
            # Analyze position history for additional movement detection
            movement_detected = self.analyze_position_history()
            if movement_detected:
                self.is_moving = True
                self.last_movement_time = current_time
            
            # Publish status
            self.publish_movement_status(self.is_moving)
            self.publish_gps_quality(self.get_gps_quality_string())
            self.publish_distance_and_speed()
            
        except Exception as e:
            self.get_logger().error(f"Error in monitor callback: {e}")
    
    def analyze_position_history(self):
        """Analyze position history to detect subtle movements"""
        if len(self.position_history) < 3:
            return False
        
        try:
            # Calculate total displacement over history
            first_pos = self.position_history[0]
            last_pos = self.position_history[-1]
            
            total_displacement = self.calculate_distance(
                first_pos['latitude'], first_pos['longitude'],
                last_pos['latitude'], last_pos['longitude']
            )
            
            # Check if total displacement indicates movement
            if total_displacement > self.get_parameter('movement_threshold').value * 0.5:
                return True
            
            # Check for consistent movement trend
            movements = []
            for i in range(1, len(self.position_history)):
                prev_pos = self.position_history[i-1]
                curr_pos = self.position_history[i]
                
                distance = self.calculate_distance(
                    prev_pos['latitude'], prev_pos['longitude'],
                    curr_pos['latitude'], curr_pos['longitude']
                )
                movements.append(distance)
            
            # If multiple recent movements, consider it as moving
            recent_movements = [m for m in movements[-3:] if m > 1.0]  # 1 meter threshold
            if len(recent_movements) >= 2:
                return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Error analyzing position history: {e}")
            return False
    
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
            return 0.0
    
    def get_gps_quality_string(self):
        """Get GPS quality status string"""
        if not self.mavros_connected:
            return "MAVROS_DISCONNECTED"
        elif self.satellites_visible < self.get_parameter('min_satellites').value:
            return "POOR_SATELLITES"
        elif self.hdop > self.get_parameter('min_hdop').value:
            return "POOR_ACCURACY"
        elif self.current_gps_position is None:
            return "NO_FIX"
        else:
            return "GOOD"
    
    def publish_movement_status(self, is_moving):
        """Publish GPS movement status"""
        msg = Bool()
        msg.data = is_moving
        self.moving_status_pub.publish(msg)
    
    def publish_gps_quality(self, quality_status):
        """Publish GPS quality status"""
        msg = String()
        msg.data = quality_status
        self.gps_quality_pub.publish(msg)
    
    def publish_distance_and_speed(self):
        """Publish distance traveled and current speed"""
        # Publish total distance
        distance_msg = Float32()
        distance_msg.data = self.total_distance_traveled
        self.distance_traveled_pub.publish(distance_msg)
        
        # Publish current speed
        speed_msg = Float32()
        speed_msg.data = self.current_speed
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        gps_monitor = GPSMonitorNode()
        rclpy.spin(gps_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gps_monitor' in locals():
            gps_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
