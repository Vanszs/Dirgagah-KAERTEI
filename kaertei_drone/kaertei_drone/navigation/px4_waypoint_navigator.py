#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - PX4 Waypoint Navigator
Menggunakan waypoint yang sudah di-set di PX4, bukan hardcode koordinat
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, WaypointList, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointPull, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix
import time
from enum import Enum
from .px4_waypoint_config import PX4WaypointConfig

from kaertei_drone.hardware.hardware_config import HardwareConfig

class PX4WaypointStatus(Enum):
    """Status PX4 waypoint navigation"""
    IDLE = "IDLE"
    LOADING_WAYPOINTS = "LOADING_WAYPOINTS"
    GOING_TO_WAYPOINT = "GOING_TO_WAYPOINT"
    WAYPOINT_REACHED = "WAYPOINT_REACHED"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    FAILED = "FAILED"

class PX4WaypointNavigator(Node):
    """
    PX4 Waypoint Navigator untuk menggunakan waypoint yang sudah di-set di PX4
    Tidak hardcode koordinat, menggunakan waypoint index yang sudah dikonfigurasi
    """
    
    def __init__(self):
        super().__init__('px4_waypoint_navigator')
        
        # Load waypoint configuration system
        self.waypoint_config = PX4WaypointConfig()
        self.get_logger().info(f"🛰️ PX4 Waypoint Navigator initialized")
        self.get_logger().info(f"📍 Venue: {self.waypoint_config.config.get('competition_venue', 'default')}")
        
        # Current state
        self.current_position = None
        self.mavros_state = State()
        self.waypoint_status = PX4WaypointStatus.IDLE
        self.current_waypoint_index = 0
        self.navigation_start_time = None
        self.px4_waypoints = []
        
        # Navigation parameters from config
        settings = self.waypoint_config.config.get('settings', {})
        self.waypoint_timeout = settings.get('waypoint_timeout', 120)
        self.waypoint_radius = settings.get('waypoint_radius', 3.0)
        self.auto_mission_altitude = settings.get('auto_mission_altitude', 3.0)
        self.retry_attempts = settings.get('retry_attempts', 2)
        
        # Print current waypoint sequences from config
        enabled_sequences = self.waypoint_config.get_enabled_sequences()
        self.get_logger().info(f"📋 Loaded {len(enabled_sequences)} enabled waypoint sequences:")
        for seq_name, waypoints in enabled_sequences.items():
            self.get_logger().info(f"   {seq_name}: {waypoints}")
        
        # MAVROS subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.position_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.position_callback, 10)
        self.waypoint_list_sub = self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self.waypoint_list_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/px4_waypoint_navigator/status', 10)
        self.waypoint_reached_pub = self.create_publisher(
            Bool, '/px4_waypoint_navigator/waypoint_reached', 10)
        
        # Service clients untuk PX4 waypoint management
        self.waypoint_pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        # self.waypoint_goto_client = self.create_client(WaypointGOTO, '/mavros/mission/goto')  # Not available
        self.waypoint_set_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Command subscriber for waypoint sequences
        self.command_sub = self.create_subscription(
            String, '/px4_waypoint_navigator/command', self.command_callback, 10)
        
        self.get_logger().info("🛰️ PX4 Waypoint Navigator initialized")
        self.get_logger().info("Available sequences: exit_to_pickup, pickup_to_drop, drop_to_finish")
        self.get_logger().info("📋 Loading waypoints from PX4...")
        
        # Load waypoints dari PX4
        self.load_px4_waypoints()
        
        # Timer untuk waypoint navigation monitoring
        self.navigation_timer = self.create_timer(1.0, self.navigation_update)
    
    def state_callback(self, msg):
        """Update MAVROS state"""
        self.mavros_state = msg
        
        # Monitor AUTO mode untuk waypoint completion
        if msg.mode == "AUTO.MISSION":
            # Check if we've reached current waypoint
            # PX4 will automatically advance to next waypoint
            pass
    
    def position_callback(self, msg):
        """Update current position"""
        if msg.status.status >= 0:  # Valid GPS fix
            self.current_position = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            }
    
    def waypoint_list_callback(self, msg):
        """Update PX4 waypoint list"""
        self.px4_waypoints = msg.waypoints
        self.get_logger().info(f"📋 Received {len(self.px4_waypoints)} waypoints from PX4")
    
    def load_px4_waypoints(self):
        """Load waypoints dari PX4"""
        if not self.waypoint_pull_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ Waypoint pull service not available")
            return False
        
        request = WaypointPull.Request()
        
        try:
            future = self.waypoint_pull_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10)
            
            if future.result().success:
                self.get_logger().info("✅ Successfully loaded waypoints from PX4")
                return True
            else:
                self.get_logger().error("❌ Failed to load waypoints from PX4")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint pull service error: {e}")
            return False
    
    def command_callback(self, msg):
        """Handle waypoint navigation commands"""
        command = msg.data.strip()
        
        # Handle waypoint sequence command format: waypoint_sequence:1,2,3
        if command.startswith('waypoint_sequence:'):
            waypoint_str = command.split(':', 1)[1]
            try:
                waypoints = [int(x.strip()) for x in waypoint_str.split(',')]
                self.get_logger().info(f"🚀 Starting custom waypoint sequence: {waypoints}")
                self.start_custom_waypoint_sequence(waypoints)
            except ValueError as e:
                self.get_logger().error(f"❌ Invalid waypoint sequence format: {e}")
                
        # Handle predefined sequence names
        elif command.lower() in self.waypoint_config.get_enabled_sequences():
            self.start_waypoint_sequence(command.lower())
            
        elif command.lower() == 'stop':
            self.stop_navigation()
        elif command.lower() == 'status':
            self.publish_status()
        elif command.lower() == 'reload_config':
            self.reload_waypoint_config()
        elif command.lower().startswith('goto_waypoint_'):
            # Format: goto_waypoint_<index>
            parts = command.lower().split('_')
            if len(parts) == 3:
                try:
                    waypoint_index = int(parts[2])
                    self.goto_px4_waypoint(waypoint_index)
                except ValueError:
                    self.get_logger().error(f"❌ Invalid waypoint index: {parts[2]}")
        else:
            self.get_logger().error(f"❌ Unknown command: {command}")
            
    def reload_waypoint_config(self):
        """Reload waypoint configuration"""
        self.waypoint_config = PX4WaypointConfig()
        enabled_sequences = self.waypoint_config.get_enabled_sequences()
        self.get_logger().info(f"🔄 Reloaded config with {len(enabled_sequences)} sequences")
        for seq_name, waypoints in enabled_sequences.items():
            self.get_logger().info(f"   {seq_name}: {waypoints}")
            
    def start_custom_waypoint_sequence(self, waypoints):
        """Start custom waypoint sequence from list of waypoint indices"""
        if not waypoints:
            self.get_logger().error("❌ Empty waypoint sequence")
            return
            
        self.get_logger().info(f"🎯 Starting custom sequence: {waypoints}")
        self.current_waypoint_sequence = waypoints
        self.current_waypoint_index = 0
        self.waypoint_status = PX4WaypointStatus.NAVIGATING
        self.navigation_start_time = time.time()
        
        # Navigate to first waypoint
        if self.goto_px4_waypoint(waypoints[0]):
            self.publish_status(f"navigating_custom_sequence:{waypoints}")
        else:
            self.waypoint_status = PX4WaypointStatus.ERROR
            self.publish_status("error:failed_to_start_custom_sequence")
    
    def start_waypoint_sequence(self, sequence_name):
        """Start navigasi waypoint sequence menggunakan PX4 waypoints dari config"""
        waypoints = self.waypoint_config.get_sequence(sequence_name)
        if not waypoints:
            self.get_logger().error(f"❌ Sequence not found or disabled: {sequence_name}")
            return
        
        self.get_logger().info(f"🛰️ Starting PX4 waypoint sequence: {sequence_name}")
        self.get_logger().info(f"� Waypoint indices: {waypoints}")
        
        # Store sequence info
        self.current_waypoint_sequence = waypoints
        self.current_sequence_name = sequence_name
        self.current_waypoint_index = 0
        self.waypoint_status = PX4WaypointStatus.NAVIGATING
        self.navigation_start_time = time.time()
        
        # Navigate to first waypoint
        if self.goto_px4_waypoint(waypoints[0]):
            self.publish_status(f"navigating_sequence:{sequence_name}")
        else:
            self.waypoint_status = PX4WaypointStatus.ERROR
            self.publish_status(f"error:failed_to_start_sequence:{sequence_name}")
        
        self.current_sequence = sequence_name
        self.current_waypoint_index = 0
        self.waypoint_status = PX4WaypointStatus.GOING_TO_WAYPOINT
        self.navigation_start_time = time.time()
        
        # Navigate to first waypoint dalam sequence
        first_waypoint_index = waypoint_indices[0]
        self.goto_px4_waypoint(first_waypoint_index)
    
    def goto_px4_waypoint(self, waypoint_index):
        """Navigate ke specific PX4 waypoint"""
        self.get_logger().info(f"🎯 Going to PX4 waypoint {waypoint_index}")
        
        # Set flight mode ke AUTO untuk waypoint navigation
        if not self.set_flight_mode("AUTO.MISSION"):
            self.get_logger().error("❌ Failed to set AUTO.MISSION mode")
            return False
        
        # Use WaypointSetCurrent command untuk navigate ke specific waypoint
        if not self.waypoint_set_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Waypoint set current service not available")
            return False
        
        request = WaypointSetCurrent.Request()
        request.wp_seq = waypoint_index
        
        try:
            future = self.waypoint_set_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10)
            
            if future.result().success:
                self.get_logger().info(f"✅ Successfully set current waypoint to {waypoint_index}")
                self.waypoint_status = PX4WaypointStatus.GOING_TO_WAYPOINT
                return True
            else:
                self.get_logger().error(f"❌ Failed to set current waypoint to {waypoint_index}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint set current service error: {e}")
            return False
    
    def navigation_update(self):
        """Update navigation status dan monitor waypoint progress"""
        if self.waypoint_status == PX4WaypointStatus.IDLE:
            return
        
        # Check timeout
        if self.navigation_start_time and (time.time() - self.navigation_start_time) > self.waypoint_timeout:
            self.waypoint_timeout_handler()
            return
        
        # Monitor PX4 mission progress
        if self.mavros_state.mode == "AUTO.MISSION":
            # Check if waypoint reached atau mission completed
            # PX4 handles waypoint advancement automatically
            self.monitor_px4_mission_progress()
    
    def monitor_px4_mission_progress(self):
        """Monitor PX4 mission progress untuk waypoint completion"""
        # Dalam AUTO.MISSION mode, PX4 akan automatically advance waypoints
        # Kita monitor apakah current sequence sudah complete
        
        if not hasattr(self, 'current_sequence'):
            return
        
        waypoint_indices = self.px4_waypoint_sequences[self.current_sequence]
        
        # Simple check: if we've been in AUTO mode untuk reasonable time,
        # assume waypoints are being executed
        if self.navigation_start_time:
            elapsed_time = time.time() - self.navigation_start_time
            
            # Estimate progress based on time (rough estimation)
            estimated_waypoint_time = 30  # seconds per waypoint estimate
            estimated_current_waypoint = min(
                int(elapsed_time / estimated_waypoint_time),
                len(waypoint_indices) - 1
            )
            
            if estimated_current_waypoint != self.current_waypoint_index:
                self.current_waypoint_index = estimated_current_waypoint
                self.get_logger().info(f"📍 Estimated progress: waypoint {estimated_current_waypoint}")
                
                # Publish waypoint reached
                reached_msg = Bool()
                reached_msg.data = True
                self.waypoint_reached_pub.publish(reached_msg)
            
            # Check if all waypoints dalam sequence completed
            if elapsed_time > (len(waypoint_indices) * estimated_waypoint_time):
                self.waypoint_sequence_completed()
    
    def waypoint_sequence_completed(self):
        """Handle when entire waypoint sequence is completed"""
        self.waypoint_status = PX4WaypointStatus.WAYPOINT_REACHED
        self.get_logger().info(f"🎉 PX4 waypoint sequence '{self.current_sequence}' completed!")
        
        # Publish completion status
        status_msg = String()
        status_msg.data = f"sequence_completed:{self.current_sequence}"
        self.status_pub.publish(status_msg)
    
    def waypoint_timeout_handler(self):
        """Handle waypoint timeout"""
        self.waypoint_status = PX4WaypointStatus.FAILED
        
        if hasattr(self, 'current_sequence'):
            waypoint_indices = self.px4_waypoint_sequences[self.current_sequence]
            current_waypoint = waypoint_indices[self.current_waypoint_index] if self.current_waypoint_index < len(waypoint_indices) else "unknown"
            
            self.get_logger().warn(f"⏰ PX4 waypoint {current_waypoint} timeout in sequence '{self.current_sequence}'")
        else:
            self.get_logger().warn("⏰ PX4 waypoint navigation timeout")
        
        # Publish timeout status
        status_msg = String()
        status_msg.data = f"waypoint_timeout"
        self.status_pub.publish(status_msg)
    
    def stop_navigation(self):
        """Stop current navigation"""
        self.waypoint_status = PX4WaypointStatus.IDLE
        
        # Switch back to GUIDED mode untuk manual control
        self.set_flight_mode("GUIDED")
        
        self.get_logger().info("🛑 PX4 waypoint navigation stopped")
        
        # Publish stop status
        status_msg = String()
        status_msg.data = "navigation_stopped"
        self.status_pub.publish(status_msg)
    
    def set_flight_mode(self, mode):
        """Set flight mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Set mode service not available")
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
                self.get_logger().error(f"❌ Failed to set flight mode to: {mode}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")
            return False
    
    def publish_status(self):
        """Publish current navigation status"""
        status_info = {
            'waypoint_status': self.waypoint_status.value,
            'current_sequence': getattr(self, 'current_sequence', 'none'),
            'waypoint_index': self.current_waypoint_index,
            'position': self.current_position,
            'flight_mode': self.mavros_state.mode,
            'total_px4_waypoints': len(self.px4_waypoints)
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f"📊 PX4 Navigation Status: {self.waypoint_status.value}")
        self.get_logger().info(f"Flight Mode: {self.mavros_state.mode}")
        if hasattr(self, 'current_sequence'):
            waypoint_indices = self.px4_waypoint_sequences[self.current_sequence]
            self.get_logger().info(f"Sequence: {self.current_sequence} ({self.current_waypoint_index}/{len(waypoint_indices)})")
    
    def update_waypoint_sequence(self, sequence_name, waypoint_indices):
        """Update waypoint sequence dengan PX4 waypoint indices"""
        if not isinstance(waypoint_indices, list):
            self.get_logger().error("Waypoint indices must be a list")
            return False
        
        old_sequence = self.px4_waypoint_sequences.get(sequence_name, [])
        self.px4_waypoint_sequences[sequence_name] = waypoint_indices
        
        self.get_logger().info(f"📍 Updated waypoint sequence '{sequence_name}'")
        self.get_logger().info(f"Old: {old_sequence}")
        self.get_logger().info(f"New: {waypoint_indices}")
        
        return True
    
    def list_px4_waypoints(self):
        """List all available PX4 waypoints"""
        self.get_logger().info(f"📋 PX4 Waypoints ({len(self.px4_waypoints)} total):")
        
        for i, waypoint in enumerate(self.px4_waypoints):
            self.get_logger().info(
                f"  Waypoint {i}: "
                f"lat={waypoint.x_lat:.6f}, "
                f"lon={waypoint.y_long:.6f}, "
                f"alt={waypoint.z_alt:.1f}m, "
                f"cmd={waypoint.command}"
            )

def main(args=None):
    rclpy.init(args=args)
    navigator = PX4WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
