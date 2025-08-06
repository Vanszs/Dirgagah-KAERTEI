#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Simplified 3-Waypoint Mission System
Mission flow: Exit Gate → WP1 (search+pickup) → WP2 (drop) → WP3 (finish)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point, Twist
import time
import json
from enum import Enum

# Import vision modules
from .vision.vision_manager import VisionManager

class Simple3WaypointCheckpoint(Enum):
    """Simplified 3-waypoint mission checkpoints"""
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    EXIT_GATE_SEARCH = "EXIT_GATE_SEARCH"
    EXIT_GATE_ALIGN = "EXIT_GATE_ALIGN"
    EXIT_GATE_PASS = "EXIT_GATE_PASS"
    ASCEND_TO_3M = "ASCEND_TO_3M"
    
    # WP1: Search and pickup
    NAVIGATE_TO_WP1 = "NAVIGATE_TO_WP1"
    SEARCH_FORWARD_WP1 = "SEARCH_FORWARD_WP1"
    ITEM_ALIGN_WP1 = "ITEM_ALIGN_WP1"
    PICKUP_WP1 = "PICKUP_WP1"
    
    # WP2: Drop
    NAVIGATE_TO_WP2 = "NAVIGATE_TO_WP2"
    DROPZONE_SEARCH_WP2 = "DROPZONE_SEARCH_WP2"
    DROPZONE_ALIGN_WP2 = "DROPZONE_ALIGN_WP2"
    DROP_WP2 = "DROP_WP2"
    
    # WP3: Finish (no additional logic)
    NAVIGATE_TO_WP3 = "NAVIGATE_TO_WP3"
    LAND_WP3 = "LAND_WP3"
    
    COMPLETED = "COMPLETED"


class Simple3WaypointMission(Node):
    """Simplified 3-waypoint mission system with vision integration"""
    
    def __init__(self):
        super().__init__('simple_3waypoint_mission')
        
        # Mission state
        self.current_checkpoint = Simple3WaypointCheckpoint.INIT
        self.mission_active = False
        self.vision_enabled = False
        
        # Timing
        self.checkpoint_start_time = None
        self.search_timeout = 30.0  # seconds for search operations
        self.alignment_timeout = 10.0  # seconds for alignment
        
        self.setup_publishers_subscribers()
        self.get_logger().info("🚁 Simple 3-Waypoint Mission System initialized")
        
    def setup_publishers_subscribers(self):
        """Setup ROS communication"""
        # Mission control publishers
        self.checkpoint_pub = self.create_publisher(String, '/mission/checkpoint', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)
        self.px4_command_pub = self.create_publisher(String, '/px4/waypoint/command', 10)
        
        # Vision control publishers
        self.vision_mode_pub = self.create_publisher(String, '/vision/detection_mode', 10)
        self.exit_gate_enable_pub = self.create_publisher(Bool, '/vision/exit_gate/enable', 10)
        self.item_enable_pub = self.create_publisher(Bool, '/vision/item/enable', 10)
        self.dropzone_enable_pub = self.create_publisher(Bool, '/vision/dropzone/enable', 10)
        
        # Hardware control publishers
        self.magnet_pub = self.create_publisher(Bool, '/hardware/magnet/enable', 10)
        self.movement_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Mission control subscribers
        self.mission_start_sub = self.create_subscription(
            Bool, '/mission/start', self.mission_start_callback, 10)
        self.checkpoint_complete_sub = self.create_subscription(
            String, '/mission/checkpoint_complete', self.checkpoint_complete_callback, 10)
        
        # Vision result subscribers
        self.exit_gate_detected_sub = self.create_subscription(
            Bool, '/vision/exit_gate/detected', self.exit_gate_detected_callback, 10)
        self.exit_gate_aligned_sub = self.create_subscription(
            Bool, '/vision/exit_gate/aligned', self.exit_gate_aligned_callback, 10)
        self.item_detected_sub = self.create_subscription(
            Bool, '/vision/item/detected', self.item_detected_callback, 10)
        self.item_aligned_sub = self.create_subscription(
            Bool, '/vision/item/aligned', self.item_aligned_callback, 10)
        self.dropzone_detected_sub = self.create_subscription(
            Bool, '/vision/dropzone/detected', self.dropzone_detected_callback, 10)
        self.dropzone_aligned_sub = self.create_subscription(
            Bool, '/vision/dropzone/aligned', self.dropzone_aligned_callback, 10)
        
        # PX4 waypoint result subscribers
        self.waypoint_reached_sub = self.create_subscription(
            String, '/px4/waypoint/status', self.waypoint_status_callback, 10)
        
        # State tracking
        self.exit_gate_detected = False
        self.exit_gate_aligned = False
        self.item_detected = False
        self.item_aligned = False
        self.dropzone_detected = False
        self.dropzone_aligned = False
        self.waypoint_reached = False
        
        # Mission timer
        self.mission_timer = self.create_timer(1.0, self.mission_update_callback)
        
    def mission_start_callback(self, msg):
        """Start the simplified 3-waypoint mission"""
        if msg.data and not self.mission_active:
            self.mission_active = True
            self.current_checkpoint = Simple3WaypointCheckpoint.INIT
            self.get_logger().info("🚀 Starting Simplified 3-Waypoint Mission")
            self.execute_checkpoint(self.current_checkpoint)
            
    def mission_update_callback(self):
        """Main mission update loop"""
        if not self.mission_active:
            return
            
        # Check for timeouts
        if self.checkpoint_start_time:
            elapsed = time.time() - self.checkpoint_start_time
            
            # Handle search timeouts
            if self.current_checkpoint in [Simple3WaypointCheckpoint.SEARCH_FORWARD_WP1] and elapsed > self.search_timeout:
                self.get_logger().warning(f"⚠️ Search timeout at {self.current_checkpoint.value}")
                self.advance_to_next_checkpoint()
                
            # Handle alignment timeouts
            elif self.current_checkpoint in [
                Simple3WaypointCheckpoint.EXIT_GATE_ALIGN,
                Simple3WaypointCheckpoint.ITEM_ALIGN_WP1,
                Simple3WaypointCheckpoint.DROPZONE_ALIGN_WP2
            ] and elapsed > self.alignment_timeout:
                self.get_logger().warning(f"⚠️ Alignment timeout at {self.current_checkpoint.value}")
                self.advance_to_next_checkpoint()
        
    def execute_checkpoint(self, checkpoint: Simple3WaypointCheckpoint):
        """Execute specific mission checkpoint"""
        self.current_checkpoint = checkpoint
        self.checkpoint_start_time = time.time()
        
        # Publish checkpoint status
        checkpoint_msg = String()
        checkpoint_msg.data = checkpoint.value
        self.checkpoint_pub.publish(checkpoint_msg)
        
        self.get_logger().info(f"📍 Executing checkpoint: {checkpoint.value}")
        
        if checkpoint == Simple3WaypointCheckpoint.INIT:
            self.execute_init()
            
        elif checkpoint == Simple3WaypointCheckpoint.TAKEOFF:
            self.execute_takeoff()
            
        elif checkpoint == Simple3WaypointCheckpoint.EXIT_GATE_SEARCH:
            self.execute_exit_gate_search()
            
        elif checkpoint == Simple3WaypointCheckpoint.EXIT_GATE_ALIGN:
            self.execute_exit_gate_align()
            
        elif checkpoint == Simple3WaypointCheckpoint.EXIT_GATE_PASS:
            self.execute_exit_gate_pass()
            
        elif checkpoint == Simple3WaypointCheckpoint.ASCEND_TO_3M:
            self.execute_ascend_to_3m()
            
        # WP1 Checkpoints
        elif checkpoint == Simple3WaypointCheckpoint.NAVIGATE_TO_WP1:
            self.execute_navigate_to_wp1()
            
        elif checkpoint == Simple3WaypointCheckpoint.SEARCH_FORWARD_WP1:
            self.execute_search_forward_wp1()
            
        elif checkpoint == Simple3WaypointCheckpoint.ITEM_ALIGN_WP1:
            self.execute_item_align_wp1()
            
        elif checkpoint == Simple3WaypointCheckpoint.PICKUP_WP1:
            self.execute_pickup_wp1()
            
        # WP2 Checkpoints
        elif checkpoint == Simple3WaypointCheckpoint.NAVIGATE_TO_WP2:
            self.execute_navigate_to_wp2()
            
        elif checkpoint == Simple3WaypointCheckpoint.DROPZONE_SEARCH_WP2:
            self.execute_dropzone_search_wp2()
            
        elif checkpoint == Simple3WaypointCheckpoint.DROPZONE_ALIGN_WP2:
            self.execute_dropzone_align_wp2()
            
        elif checkpoint == Simple3WaypointCheckpoint.DROP_WP2:
            self.execute_drop_wp2()
            
        # WP3 Checkpoints
        elif checkpoint == Simple3WaypointCheckpoint.NAVIGATE_TO_WP3:
            self.execute_navigate_to_wp3()
            
        elif checkpoint == Simple3WaypointCheckpoint.LAND_WP3:
            self.execute_land_wp3()
            
        elif checkpoint == Simple3WaypointCheckpoint.COMPLETED:
            self.execute_completed()
    
    # Checkpoint execution methods
    def execute_init(self):
        """Initialize mission"""
        self.get_logger().info("🔧 Initializing systems...")
        
        # Enable vision systems
        self.set_vision_mode("init")
        
        # Initialize PX4 waypoint system with simple_3wp preset
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "set_preset",
            "preset": "simple_3wp"
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(2)
        self.advance_to_next_checkpoint()
        
    def execute_takeoff(self):
        """Execute takeoff"""
        self.get_logger().info("🛫 Taking off...")
        
        # Command takeoff via PX4
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "takeoff",
            "altitude": 2.0
        })
        self.px4_command_pub.publish(px4_cmd)
        
        # Advance after takeoff delay
        time.sleep(5)
        self.advance_to_next_checkpoint()
        
    def execute_exit_gate_search(self):
        """Search for exit gate"""
        self.get_logger().info("🔍 Searching for exit gate...")
        
        # Enable exit gate detection
        self.set_vision_mode("exit_gate")
        enable_msg = Bool()
        enable_msg.data = True
        self.exit_gate_enable_pub.publish(enable_msg)
        
    def execute_exit_gate_align(self):
        """Align with exit gate center"""
        self.get_logger().info("🎯 Aligning with exit gate center...")
        
        # Vision system will handle alignment via /vision/exit_gate/alignment_error
        # Checkpoint will advance when aligned
        
    def execute_exit_gate_pass(self):
        """Pass through the exit gate"""
        self.get_logger().info("🚪 Passing through exit gate...")
        
        # Move forward through gate
        move_cmd = Twist()
        move_cmd.linear.x = 1.0  # Forward movement
        self.movement_pub.publish(move_cmd)
        
        time.sleep(3)
        self.advance_to_next_checkpoint()
        
    def execute_ascend_to_3m(self):
        """Ascend to 3 meters"""
        self.get_logger().info("⬆️ Ascending to 3 meters...")
        
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "set_altitude",
            "altitude": 3.0
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(3)
        self.advance_to_next_checkpoint()
        
    def execute_navigate_to_wp1(self):
        """Navigate to WP1 using PX4 waypoint system"""
        self.get_logger().info("🎯 Navigating to WP1...")
        
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "goto_waypoint",
            "waypoint_index": 0  # WP1
        })
        self.px4_command_pub.publish(px4_cmd)
        
    def execute_search_forward_wp1(self):
        """Search for item while moving forward at WP1"""
        self.get_logger().info("🔍 Searching for item at WP1...")
        
        # Enable item detection
        self.set_vision_mode("item")
        enable_msg = Bool()
        enable_msg.data = True
        self.item_enable_pub.publish(enable_msg)
        
        # The item detector will handle forward movement search
        
    def execute_item_align_wp1(self):
        """Align with item for pickup at WP1"""
        self.get_logger().info("🎯 Aligning with item at WP1...")
        
        # Vision system handles alignment
        
    def execute_pickup_wp1(self):
        """Pickup item at WP1"""
        self.get_logger().info("🧲 Picking up item at WP1...")
        
        # Descend and enable magnet
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "descend",
            "distance": 1.0
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(2)
        
        # Enable magnet
        magnet_msg = Bool()
        magnet_msg.data = True
        self.magnet_pub.publish(magnet_msg)
        
        time.sleep(1)
        
        # Ascend back
        px4_cmd.data = json.dumps({
            "command": "ascend",
            "distance": 1.0
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(2)
        self.advance_to_next_checkpoint()
        
    def execute_navigate_to_wp2(self):
        """Navigate to WP2 for drop logic"""
        self.get_logger().info("🎯 Navigating to WP2...")
        
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "goto_waypoint",
            "waypoint_index": 1  # WP2
        })
        self.px4_command_pub.publish(px4_cmd)
        
    def execute_dropzone_search_wp2(self):
        """Search for dropzone at WP2"""
        self.get_logger().info("🔍 Searching for dropzone at WP2...")
        
        # Enable dropzone detection
        self.set_vision_mode("dropzone")
        enable_msg = Bool()
        enable_msg.data = True
        self.dropzone_enable_pub.publish(enable_msg)
        
    def execute_dropzone_align_wp2(self):
        """Align with dropzone at WP2"""
        self.get_logger().info("🎯 Aligning with dropzone at WP2...")
        
        # Vision system handles alignment
        
    def execute_drop_wp2(self):
        """Drop item at WP2"""
        self.get_logger().info("📦 Dropping item at WP2...")
        
        # Descend and disable magnet
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "descend",
            "distance": 0.5
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(2)
        
        # Disable magnet to drop item
        magnet_msg = Bool()
        magnet_msg.data = False
        self.magnet_pub.publish(magnet_msg)
        
        time.sleep(1)
        
        # Ascend back
        px4_cmd.data = json.dumps({
            "command": "ascend",
            "distance": 0.5
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(2)
        self.advance_to_next_checkpoint()
        
    def execute_navigate_to_wp3(self):
        """Navigate to WP3 (final, no additional logic)"""
        self.get_logger().info("🏁 Navigating to WP3 (Final)...")
        
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "goto_waypoint",
            "waypoint_index": 2  # WP3
        })
        self.px4_command_pub.publish(px4_cmd)
        
    def execute_land_wp3(self):
        """Land at WP3"""
        self.get_logger().info("🛬 Landing at WP3...")
        
        px4_cmd = String()
        px4_cmd.data = json.dumps({
            "command": "land"
        })
        self.px4_command_pub.publish(px4_cmd)
        
        time.sleep(5)
        self.advance_to_next_checkpoint()
        
    def execute_completed(self):
        """Mission completed"""
        self.get_logger().info("✅ Mission completed!")
        self.mission_active = False
        
        # Disable all vision systems
        self.set_vision_mode("disabled")
        
        status_msg = String()
        status_msg.data = "MISSION_COMPLETED"
        self.mission_status_pub.publish(status_msg)
    
    # Helper methods
    def set_vision_mode(self, mode: str):
        """Set vision detection mode"""
        mode_msg = String()
        mode_msg.data = mode
        self.vision_mode_pub.publish(mode_msg)
        
    def advance_to_next_checkpoint(self):
        """Advance to the next mission checkpoint"""
        checkpoints = list(Simple3WaypointCheckpoint)
        current_index = checkpoints.index(self.current_checkpoint)
        
        if current_index < len(checkpoints) - 1:
            next_checkpoint = checkpoints[current_index + 1]
            self.execute_checkpoint(next_checkpoint)
        else:
            self.execute_checkpoint(Simple3WaypointCheckpoint.COMPLETED)
    
    # Vision callback handlers
    def exit_gate_detected_callback(self, msg):
        """Handle exit gate detection"""
        self.exit_gate_detected = msg.data
        if msg.data and self.current_checkpoint == Simple3WaypointCheckpoint.EXIT_GATE_SEARCH:
            self.advance_to_next_checkpoint()
            
    def exit_gate_aligned_callback(self, msg):
        """Handle exit gate alignment"""
        self.exit_gate_aligned = msg.data
        if msg.data and self.current_checkpoint == Simple3WaypointCheckpoint.EXIT_GATE_ALIGN:
            self.advance_to_next_checkpoint()
            
    def item_detected_callback(self, msg):
        """Handle item detection"""
        self.item_detected = msg.data
        if msg.data and self.current_checkpoint == Simple3WaypointCheckpoint.SEARCH_FORWARD_WP1:
            self.advance_to_next_checkpoint()
            
    def item_aligned_callback(self, msg):
        """Handle item alignment"""
        self.item_aligned = msg.data
        if msg.data and self.current_checkpoint == Simple3WaypointCheckpoint.ITEM_ALIGN_WP1:
            self.advance_to_next_checkpoint()
            
    def dropzone_detected_callback(self, msg):
        """Handle dropzone detection"""
        self.dropzone_detected = msg.data
        if msg.data and self.current_checkpoint == Simple3WaypointCheckpoint.DROPZONE_SEARCH_WP2:
            self.advance_to_next_checkpoint()
            
    def dropzone_aligned_callback(self, msg):
        """Handle dropzone alignment"""
        self.dropzone_aligned = msg.data
        if msg.data and self.current_checkpoint == Simple3WaypointCheckpoint.DROPZONE_ALIGN_WP2:
            self.advance_to_next_checkpoint()
            
    def waypoint_status_callback(self, msg):
        """Handle PX4 waypoint status"""
        try:
            status_data = json.loads(msg.data)
            if status_data.get('status') == 'reached':
                waypoint_index = status_data.get('waypoint_index', -1)
                
                if waypoint_index == 0 and self.current_checkpoint == Simple3WaypointCheckpoint.NAVIGATE_TO_WP1:
                    self.advance_to_next_checkpoint()
                elif waypoint_index == 1 and self.current_checkpoint == Simple3WaypointCheckpoint.NAVIGATE_TO_WP2:
                    self.advance_to_next_checkpoint()
                elif waypoint_index == 2 and self.current_checkpoint == Simple3WaypointCheckpoint.NAVIGATE_TO_WP3:
                    self.advance_to_next_checkpoint()
                    
        except Exception as e:
            self.get_logger().error(f"Error parsing waypoint status: {e}")
    
    def checkpoint_complete_callback(self, msg):
        """Handle manual checkpoint completion"""
        if msg.data == "advance":
            self.advance_to_next_checkpoint()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        mission = Simple3WaypointMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        if 'mission' in locals():
            mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
