#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Simplified Mission Control
Flow sederhana 3 waypoint sesuai permintaan user:

1. Exit gate -> naik 3m -> WP1  
2. WP1 -> search object sambil maju -> pickup -> WP2
3. WP2 -> drop logic -> WP3
4. WP3 -> selesai (no additional logic)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from .unified_vision_system import VisionSystem
from .simple_3wp_config import Simple3WaypointConfig
import time

class SimplifiedMissionControl(Node):
    def __init__(self):
        super().__init__('simplified_mission_control')
        
        # Load config
        self.config = Simple3WaypointConfig()
        self.waypoint_sequence = self.config.get_waypoint_sequence()
        
        # Mission state
        self.current_state = "INIT"
        self.current_waypoint = None
        self.mission_start_time = None
        self.object_detected = False
        self.object_picked = False
        self.mission_completed = False
        
        # Publishers
        self.px4_command_pub = self.create_publisher(String, '/px4_waypoint_navigator/command', 10)
        self.vision_mode_pub = self.create_publisher(String, '/vision/mode', 10)
        self.flight_command_pub = self.create_publisher(String, '/flight/command', 10)
        self.magnet_command_pub = self.create_publisher(String, '/magnet/command', 10)
        
        # Subscribers
        self.vision_object_sub = self.create_subscription(String, '/vision/object_detected', self.object_callback, 10)
        self.vision_aligned_sub = self.create_subscription(Bool, '/vision/aligned', self.alignment_callback, 10)
        self.vision_exit_sub = self.create_subscription(Bool, '/vision/exit_detected', self.exit_callback, 10)
        self.px4_status_sub = self.create_subscription(String, '/px4_waypoint_navigator/status', self.px4_status_callback, 10)
        self.user_input_sub = self.create_subscription(String, '/mission/user_input', self.user_input_callback, 10)
        
        # State variables
        self.exit_detected = False
        self.object_aligned = False
        self.waypoint_reached = False
        self.waiting_for_next = False
        
        # Timer for state machine
        self.timer = self.create_timer(1.0, self.state_machine)
        
        self.get_logger().info("🎯 Simplified Mission Control initialized")
        self.config.print_config()
        
    def state_machine(self):
        """Simple state machine untuk 3 waypoint mission"""
        
        if self.current_state == "INIT":
            self.init_mission()
            
        elif self.current_state == "WAIT_EXIT":
            self.wait_for_exit_gate()
            
        elif self.current_state == "ASCEND_3M":
            self.ascend_to_3m()
            
        elif self.current_state == "GO_WP1":
            self.navigate_to_wp1()
            
        elif self.current_state == "SEARCH_PICKUP":
            self.search_and_pickup_at_wp1()
            
        elif self.current_state == "GO_WP2":
            self.navigate_to_wp2()
            
        elif self.current_state == "DROP_WP2":
            self.drop_at_wp2()
            
        elif self.current_state == "GO_WP3":
            self.navigate_to_wp3()
            
        elif self.current_state == "MISSION_COMPLETE":
            self.mission_complete()
            
    def init_mission(self):
        """Initialize mission"""
        self.get_logger().info("🚀 Starting Simplified 3 Waypoint Mission")
        
        # Set vision to exit gate detection
        vision_msg = String()
        vision_msg.data = "exit_gate"
        self.vision_mode_pub.publish(vision_msg)
        
        self.current_state = "WAIT_EXIT"
        self.mission_start_time = time.time()
        
    def wait_for_exit_gate(self):
        """Wait for exit gate detection"""
        if self.exit_detected:
            self.get_logger().info("✅ Exit gate detected! Ready to proceed")
            self.current_state = "ASCEND_3M"
            self.waiting_for_next = True
        else:
            # Still waiting for exit detection
            pass
            
    def ascend_to_3m(self):
        """Ascend to 3m altitude after exit"""
        if self.waiting_for_next:
            return
            
        self.get_logger().info("⬆️ Ascending to 3m altitude...")
        
        # Send ascend command
        flight_msg = String()
        flight_msg.data = "ascend:3.0"
        self.flight_command_pub.publish(flight_msg)
        
        # Wait for user confirmation (in real implementation, wait for altitude confirmation)
        self.waiting_for_next = True
        self.get_logger().info("📢 Press 'next' to continue to WP1")
        
    def navigate_to_wp1(self):
        """Navigate to WP1 (pickup area)"""
        if self.waiting_for_next:
            return
            
        wp1_index = self.waypoint_sequence[0]
        self.get_logger().info(f"🛰️ Navigating to WP1 (PX4 waypoint {wp1_index})")
        
        # Send waypoint command
        px4_msg = String()
        px4_msg.data = f"goto_waypoint_{wp1_index}"
        self.px4_command_pub.publish(px4_msg)
        
        self.current_waypoint = wp1_index
        self.current_state = "SEARCH_PICKUP"
        
    def search_and_pickup_at_wp1(self):
        """Search object sambil maju dan pickup when found"""
        if not hasattr(self, 'search_started'):
            self.get_logger().info("🔍 Starting object search at WP1...")
            
            # Set vision to object detection mode
            vision_msg = String()
            vision_msg.data = "object_detection"
            self.vision_mode_pub.publish(vision_msg)
            
            # Start forward movement search pattern
            flight_msg = String()
            flight_msg.data = "search_forward:slow"
            self.flight_command_pub.publish(flight_msg)
            
            self.search_started = True
            self.search_start_time = time.time()
            
        # Check if object detected and aligned
        if self.object_detected and self.object_aligned:
            self.get_logger().info("🎯 Object detected and aligned! Executing pickup...")
            
            # Stop movement
            flight_msg = String()
            flight_msg.data = "hold_position"
            self.flight_command_pub.publish(flight_msg)
            
            # Descend for pickup
            flight_msg = String()
            flight_msg.data = "descend:0.5"  # pickup altitude
            self.flight_command_pub.publish(flight_msg)
            
            # Activate magnet
            magnet_msg = String()
            magnet_msg.data = "front_on"
            self.magnet_command_pub.publish(magnet_msg)
            
            # Wait for pickup confirmation
            self.waiting_for_next = True
            self.object_picked = True
            self.get_logger().info("📢 Object pickup complete! Press 'next' to go to WP2")
            
        # Check timeout
        elif hasattr(self, 'search_start_time'):
            search_timeout = self.config.config["mission_settings"]["search_timeout"]
            if time.time() - self.search_start_time > search_timeout:
                self.get_logger().warn(f"⏰ Search timeout ({search_timeout}s). Proceeding to WP2 anyway...")
                self.waiting_for_next = True
                
    def navigate_to_wp2(self):
        """Navigate to WP2 (dropzone)"""
        if self.waiting_for_next:
            return
            
        wp2_index = self.waypoint_sequence[1]
        self.get_logger().info(f"🛰️ Navigating to WP2 (PX4 waypoint {wp2_index})")
        
        # Ascend back to cruising altitude
        flight_msg = String()
        flight_msg.data = "ascend:3.0"
        self.flight_command_pub.publish(flight_msg)
        
        # Navigate to WP2
        px4_msg = String()
        px4_msg.data = f"goto_waypoint_{wp2_index}"
        self.px4_command_pub.publish(px4_msg)
        
        self.current_waypoint = wp2_index
        self.current_state = "DROP_WP2"
        
    def drop_at_wp2(self):
        """Drop object at WP2"""
        if not hasattr(self, 'drop_started'):
            self.get_logger().info("📦 Executing drop at WP2...")
            
            # Set vision to dropzone detection (optional)
            vision_msg = String()
            vision_msg.data = "dropzone"
            self.vision_mode_pub.publish(vision_msg)
            
            # Descend for drop
            flight_msg = String()
            flight_msg.data = "descend:0.8"  # drop altitude
            self.flight_command_pub.publish(flight_msg)
            
            # Deactivate magnet
            magnet_msg = String()
            magnet_msg.data = "front_off"
            self.magnet_command_pub.publish(magnet_msg)
            
            self.drop_started = True
            self.waiting_for_next = True
            self.get_logger().info("📢 Drop complete! Press 'next' to go to WP3")
            
    def navigate_to_wp3(self):
        """Navigate to WP3 (final waypoint)"""
        if self.waiting_for_next:
            return
            
        wp3_index = self.waypoint_sequence[2]  
        self.get_logger().info(f"🛰️ Navigating to WP3 (PX4 waypoint {wp3_index}) - Final waypoint")
        
        # Ascend back to cruising altitude
        flight_msg = String()
        flight_msg.data = "ascend:3.0"
        self.flight_command_pub.publish(flight_msg)
        
        # Navigate to WP3
        px4_msg = String()
        px4_msg.data = f"goto_waypoint_{wp3_index}"
        self.px4_command_pub.publish(px4_msg)
        
        self.current_waypoint = wp3_index
        self.current_state = "MISSION_COMPLETE"
        
    def mission_complete(self):
        """Mission completed at WP3"""
        if not self.mission_completed:
            self.get_logger().info("🎉 Mission completed at WP3!")
            self.get_logger().info("✅ No additional logic required - mission successful!")
            
            # Set mission complete flag
            self.mission_completed = True
            
            # Calculate mission time
            if self.mission_start_time:
                mission_time = time.time() - self.mission_start_time
                self.get_logger().info(f"⏱️ Total mission time: {mission_time:.1f} seconds")
                
            # Optional: Land at WP3 or hold position
            flight_msg = String()
            flight_msg.data = "hold_position"
            self.flight_command_pub.publish(flight_msg)
            
    # Callback functions
    def object_callback(self, msg):
        """Handle object detection"""
        if msg.data != "NONE":
            self.object_detected = True
            self.get_logger().info(f"👀 Object detected: {msg.data}")
        else:
            self.object_detected = False
            
    def alignment_callback(self, msg):
        """Handle alignment status"""
        self.object_aligned = msg.data
        if msg.data:
            self.get_logger().info("🎯 Object aligned!")
            
    def exit_callback(self, msg):
        """Handle exit gate detection"""
        self.exit_detected = msg.data
        if msg.data:
            self.get_logger().info("🚪 Exit gate detected!")
            
    def px4_status_callback(self, msg):
        """Handle PX4 waypoint navigator status"""
        self.get_logger().info(f"🛰️ PX4 Status: {msg.data}")
        
        if "waypoint_reached" in msg.data.lower():
            self.waypoint_reached = True
            
            # Auto-advance states based on current waypoint
            if self.current_state == "SEARCH_PICKUP" and self.object_picked:
                self.current_state = "GO_WP2"
                self.waiting_for_next = False
            elif self.current_state == "DROP_WP2":
                self.current_state = "GO_WP3"  
                self.waiting_for_next = False
                
    def user_input_callback(self, msg):
        """Handle user input for manual progression"""
        command = msg.data.strip().lower()
        
        if command == 'next' and self.waiting_for_next:
            self.get_logger().info("▶️ User input: Proceeding to next state")
            self.waiting_for_next = False
            
            # Advance states
            if self.current_state == "ASCEND_3M":
                self.current_state = "GO_WP1"
            elif self.current_state == "SEARCH_PICKUP":
                self.current_state = "GO_WP2"
            elif self.current_state == "DROP_WP2":
                self.current_state = "GO_WP3"
                
        elif command == 'status':
            self.print_status()
            
    def print_status(self):
        """Print current mission status"""
        self.get_logger().info("📊 MISSION STATUS:")
        self.get_logger().info(f"   State: {self.current_state}")
        self.get_logger().info(f"   Current WP: {self.current_waypoint}")
        self.get_logger().info(f"   Object detected: {self.object_detected}")
        self.get_logger().info(f"   Object picked: {self.object_picked}")
        self.get_logger().info(f"   Exit detected: {self.exit_detected}")
        self.get_logger().info(f"   Waiting for next: {self.waiting_for_next}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        mission_control = SimplifiedMissionControl()
        rclpy.spin(mission_control)
    except KeyboardInterrupt:
        pass
    finally:
        if 'mission_control' in locals():
            mission_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
