#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Waypoint Navigator
Mengganti AUTO mode dengan waypoint navigation custom yang lebih fleksibel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix
import math
import time
from enum import Enum

from kaertei_drone.hardware.hardware_config import HardwareConfig

class WaypointStatus(Enum):
    """Status waypoint navigation"""
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING" 
    REACHED = "REACHED"
    TIMEOUT = "TIMEOUT"
    FAILED = "FAILED"

class WaypointNavigator(Node):
    """
    Waypoint Navigator untuk navigasi custom setelah exit gate
    Menggantikan AUTO mode dengan kontrol waypoint manual yang lebih fleksibel
    """
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Load hardware configuration
        self.config = HardwareConfig()
        
        # Current state
        self.current_position = None
        self.current_altitude = 0.0
        self.mavros_state = State()
        self.waypoint_status = WaypointStatus.IDLE
        self.current_waypoint_index = 0
        self.navigation_start_time = None
        
        # Navigation parameters
        self.waypoint_reached_threshold = 3.0  # meters
        self.waypoint_timeout = 120  # seconds per waypoint
        self.navigation_altitude = 3.0  # outdoor cruise altitude
        
        # Custom waypoint sequences
        self.waypoint_sequences = {
            # Sequence dari exit gate ke pickup location
            'exit_to_pickup': [
                {'name': 'exit_transition', 'lat': -6.365200, 'lon': 106.824800, 'alt': 30.0},
                {'name': 'approach_pickup', 'lat': -6.365000, 'lon': 106.825000, 'alt': 30.0},
                {'name': 'pickup_position', 'lat': -6.364900, 'lon': 106.825100, 'alt': 20.0}
            ],
            # Sequence dari pickup ke dropzone
            'pickup_to_drop': [
                {'name': 'departure_pickup', 'lat': -6.364900, 'lon': 106.825100, 'alt': 30.0},
                {'name': 'transit_to_drop', 'lat': -6.364700, 'lon': 106.825300, 'alt': 30.0},
                {'name': 'approach_dropzone', 'lat': -6.364500, 'lon': 106.825500, 'alt': 30.0},
                {'name': 'drop_position', 'lat': -6.364400, 'lon': 106.825600, 'alt': 20.0}
            ],
            # Sequence dari dropzone ke finish/landing
            'drop_to_finish': [
                {'name': 'departure_drop', 'lat': -6.364400, 'lon': 106.825600, 'alt': 30.0},
                {'name': 'transit_home', 'lat': -6.365000, 'lon': 106.825000, 'alt': 30.0},
                {'name': 'approach_home', 'lat': -6.365300, 'lon': 106.824700, 'alt': 30.0},
                {'name': 'landing_position', 'lat': -6.365500, 'lon': 106.824500, 'alt': 10.0}
            ]
        }
        
        # MAVROS subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.position_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.position_callback, 10)
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget, '/mavros/setpoint_position/global', 10)
        self.status_pub = self.create_publisher(
            String, '/waypoint_navigator/status', 10)
        self.waypoint_reached_pub = self.create_publisher(
            Bool, '/waypoint_navigator/waypoint_reached', 10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Command subscriber for waypoint sequences
        self.command_sub = self.create_subscription(
            String, '/waypoint_navigator/command', self.command_callback, 10)
        
        self.get_logger().info("🛰️ Waypoint Navigator initialized")
        self.get_logger().info("Available sequences: exit_to_pickup, pickup_to_drop, drop_to_finish")
        
        # Timer untuk waypoint navigation
        self.navigation_timer = self.create_timer(0.5, self.navigation_update)
    
    def state_callback(self, msg):
        """Update MAVROS state"""
        self.mavros_state = msg
    
    def position_callback(self, msg):
        """Update current position"""
        if msg.status.status >= 0:  # Valid GPS fix
            self.current_position = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            }
            self.current_altitude = msg.altitude
    
    def command_callback(self, msg):
        """Handle waypoint navigation commands"""
        command = msg.data.strip().lower()
        
        if command in self.waypoint_sequences:
            self.start_waypoint_sequence(command)
        elif command == 'stop':
            self.stop_navigation()
        elif command == 'status':
            self.publish_status()
        elif command.startswith('goto_waypoint_'):
            # Format: goto_waypoint_<sequence>_<index>
            # Example: goto_waypoint_exit_to_pickup_1
            parts = command.split('_')
            if len(parts) >= 4:
                sequence_name = '_'.join(parts[2:-1])
                try:
                    waypoint_index = int(parts[-1])
                    self.goto_specific_waypoint(sequence_name, waypoint_index)
                except ValueError:
                    self.get_logger().error(f"Invalid waypoint index: {parts[-1]}")
        else:
            self.get_logger().error(f"Unknown command: {command}")
    
    def start_waypoint_sequence(self, sequence_name):
        """Start navigasi waypoint sequence"""
        if sequence_name not in self.waypoint_sequences:
            self.get_logger().error(f"Unknown waypoint sequence: {sequence_name}")
            return
        
        self.current_sequence = sequence_name
        self.current_waypoint_index = 0
        self.waypoint_status = WaypointStatus.NAVIGATING
        self.navigation_start_time = time.time()
        
        waypoints = self.waypoint_sequences[sequence_name]
        self.get_logger().info(f"🛰️ Starting waypoint sequence: {sequence_name}")
        self.get_logger().info(f"Total waypoints: {len(waypoints)}")
        
        # Start navigating to first waypoint
        self.navigate_to_current_waypoint()
    
    def goto_specific_waypoint(self, sequence_name, waypoint_index):
        """Navigate directly to specific waypoint dalam sequence"""
        if sequence_name not in self.waypoint_sequences:
            self.get_logger().error(f"Unknown waypoint sequence: {sequence_name}")
            return
        
        waypoints = self.waypoint_sequences[sequence_name]
        if waypoint_index >= len(waypoints):
            self.get_logger().error(f"Waypoint index {waypoint_index} out of range for {sequence_name}")
            return
        
        self.current_sequence = sequence_name
        self.current_waypoint_index = waypoint_index
        self.waypoint_status = WaypointStatus.NAVIGATING
        self.navigation_start_time = time.time()
        
        self.get_logger().info(f"🎯 Going directly to waypoint {waypoint_index} in {sequence_name}")
        self.navigate_to_current_waypoint()
    
    def navigate_to_current_waypoint(self):
        """Navigate ke waypoint saat ini"""
        if not hasattr(self, 'current_sequence') or self.current_sequence not in self.waypoint_sequences:
            return
        
        waypoints = self.waypoint_sequences[self.current_sequence]
        if self.current_waypoint_index >= len(waypoints):
            self.waypoint_sequence_completed()
            return
        
        current_waypoint = waypoints[self.current_waypoint_index]
        
        # Set flight mode to GUIDED for manual waypoint control
        if self.mavros_state.mode != "GUIDED":
            self.set_flight_mode("GUIDED")
        
        # Create and publish setpoint
        setpoint = GlobalPositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = "map"
        
        # Coordinate frame: use lat/lon/alt
        setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        
        # Set position
        setpoint.latitude = current_waypoint['lat']
        setpoint.longitude = current_waypoint['lon']
        setpoint.altitude = current_waypoint['alt']
        
        # Enable position control only (no velocity/acceleration)
        setpoint.type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ
        )
        
        self.setpoint_pub.publish(setpoint)
        
        self.get_logger().info(f"🛰️ Navigating to waypoint {self.current_waypoint_index}: {current_waypoint['name']}")
        self.get_logger().info(f"📍 Target: {current_waypoint['lat']:.6f}, {current_waypoint['lon']:.6f}, {current_waypoint['alt']:.1f}m")
    
    def navigation_update(self):
        """Update navigation status dan check waypoint reached"""
        if self.waypoint_status != WaypointStatus.NAVIGATING:
            return
        
        if not self.current_position:
            return
        
        # Check timeout
        if self.navigation_start_time and (time.time() - self.navigation_start_time) > self.waypoint_timeout:
            self.waypoint_timeout_handler()
            return
        
        # Check if waypoint reached
        if self.is_waypoint_reached():
            self.waypoint_reached_handler()
    
    def is_waypoint_reached(self):
        """Check apakah waypoint sudah reached"""
        if not self.current_position or not hasattr(self, 'current_sequence'):
            return False
        
        waypoints = self.waypoint_sequences[self.current_sequence]
        if self.current_waypoint_index >= len(waypoints):
            return False
        
        current_waypoint = waypoints[self.current_waypoint_index]
        distance = self.calculate_distance(
            self.current_position['lat'], self.current_position['lon'],
            current_waypoint['lat'], current_waypoint['lon']
        )
        
        altitude_diff = abs(self.current_position['alt'] - current_waypoint['alt'])
        
        # Waypoint reached if within horizontal threshold and altitude threshold
        horizontal_reached = distance <= self.waypoint_reached_threshold
        altitude_reached = altitude_diff <= 5.0  # 5 meter altitude tolerance
        
        return horizontal_reached and altitude_reached
    
    def waypoint_reached_handler(self):
        """Handle when waypoint is reached"""
        waypoints = self.waypoint_sequences[self.current_sequence]
        current_waypoint = waypoints[self.current_waypoint_index]
        
        self.get_logger().info(f"✅ Waypoint {self.current_waypoint_index} reached: {current_waypoint['name']}")
        
        # Publish waypoint reached
        reached_msg = Bool()
        reached_msg.data = True
        self.waypoint_reached_pub.publish(reached_msg)
        
        # Move to next waypoint
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(waypoints):
            # Sequence completed
            self.waypoint_sequence_completed()
        else:
            # Navigate to next waypoint
            self.navigation_start_time = time.time()  # Reset timer
            self.navigate_to_current_waypoint()
    
    def waypoint_sequence_completed(self):
        """Handle when entire waypoint sequence is completed"""
        self.waypoint_status = WaypointStatus.REACHED
        self.get_logger().info(f"🎉 Waypoint sequence '{self.current_sequence}' completed!")
        
        # Publish completion status
        status_msg = String()
        status_msg.data = f"sequence_completed:{self.current_sequence}"
        self.status_pub.publish(status_msg)
    
    def waypoint_timeout_handler(self):
        """Handle waypoint timeout"""
        self.waypoint_status = WaypointStatus.TIMEOUT
        waypoints = self.waypoint_sequences[self.current_sequence]
        current_waypoint = waypoints[self.current_waypoint_index]
        
        self.get_logger().warn(f"⏰ Waypoint {self.current_waypoint_index} timeout: {current_waypoint['name']}")
        
        # Publish timeout status
        status_msg = String()
        status_msg.data = f"waypoint_timeout:{self.current_waypoint_index}"
        self.status_pub.publish(status_msg)
    
    def stop_navigation(self):
        """Stop current navigation"""
        self.waypoint_status = WaypointStatus.IDLE
        self.get_logger().info("🛑 Navigation stopped")
        
        # Publish stop status
        status_msg = String()
        status_msg.data = "navigation_stopped"
        self.status_pub.publish(status_msg)
    
    def set_flight_mode(self, mode):
        """Set flight mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)
            
            if future.result().mode_sent:
                self.get_logger().info(f"✈️ Flight mode set to: {mode}")
                return True
            else:
                self.get_logger().error(f"Failed to set flight mode to: {mode}")
                return False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate Haversine distance between two GPS coordinates"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        distance = R * c
        return distance
    
    def publish_status(self):
        """Publish current navigation status"""
        status_info = {
            'waypoint_status': self.waypoint_status.value,
            'current_sequence': getattr(self, 'current_sequence', 'none'),
            'waypoint_index': self.current_waypoint_index,
            'position': self.current_position,
            'altitude': self.current_altitude
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f"📊 Navigation Status: {self.waypoint_status.value}")
        if hasattr(self, 'current_sequence'):
            waypoints = self.waypoint_sequences[self.current_sequence]
            self.get_logger().info(f"Sequence: {self.current_sequence} ({self.current_waypoint_index}/{len(waypoints)})")
    
    def update_waypoint_coordinates(self, sequence_name, waypoint_index, lat, lon, alt):
        """Update waypoint coordinates untuk customization"""
        if sequence_name not in self.waypoint_sequences:
            self.get_logger().error(f"Unknown sequence: {sequence_name}")
            return False
        
        waypoints = self.waypoint_sequences[sequence_name]
        if waypoint_index >= len(waypoints):
            self.get_logger().error(f"Waypoint index out of range: {waypoint_index}")
            return False
        
        old_coords = waypoints[waypoint_index]
        waypoints[waypoint_index]['lat'] = lat
        waypoints[waypoint_index]['lon'] = lon
        waypoints[waypoint_index]['alt'] = alt
        
        self.get_logger().info(f"📍 Updated waypoint {waypoint_index} in {sequence_name}")
        self.get_logger().info(f"Old: {old_coords['lat']:.6f}, {old_coords['lon']:.6f}, {old_coords['alt']:.1f}")
        self.get_logger().info(f"New: {lat:.6f}, {lon:.6f}, {alt:.1f}")
        
        return True

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
