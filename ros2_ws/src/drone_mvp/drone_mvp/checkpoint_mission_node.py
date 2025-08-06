#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import Image
import time
import threading
from enum import Enum
import json

# Import MAVLink for direct PX4 communication
try:
    from pymavlink import mavutil
except ImportError:
    print("❌ pymavlink not installed. Run: pip3 install pymavlink")
    mavutil = None

# Import hardware configuration
from .hardware_config import HardwareConfig

class MissionCheckpoint(Enum):
    """Mission checkpoints for debugging"""
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    SEARCH_ITEM_1_FRONT = "SEARCH_ITEM_1_FRONT"
    ALIGN_ITEM_1 = "ALIGN_ITEM_1"
    PICKUP_ITEM_1 = "PICKUP_ITEM_1"
    SEARCH_ITEM_2_BACK = "SEARCH_ITEM_2_BACK"
    ALIGN_ITEM_2 = "ALIGN_ITEM_2"
    PICKUP_ITEM_2 = "PICKUP_ITEM_2"
    NAVIGATE_TURN_DIRECTION = "NAVIGATE_TURN_DIRECTION"
    SEARCH_DROPZONE = "SEARCH_DROPZONE"
    DROP_ITEM_1_FRONT = "DROP_ITEM_1_FRONT"
    ASCEND_AFTER_DROP_1 = "ASCEND_AFTER_DROP_1"
    ALIGN_DROP_2_BACK = "ALIGN_DROP_2_BACK"
    DROP_ITEM_2_BACK = "DROP_ITEM_2_BACK"
    FIND_EXIT = "FIND_EXIT"
    ASCEND_TO_OUTDOOR = "ASCEND_TO_OUTDOOR"
    AUTO_WAYPOINT_1 = "AUTO_WAYPOINT_1"
    MANUAL_SEARCH_OUTDOOR = "MANUAL_SEARCH_OUTDOOR"
    PICKUP_OUTDOOR = "PICKUP_OUTDOOR"
    ASCEND_TO_WAYPOINT_2 = "ASCEND_TO_WAYPOINT_2"
    AUTO_WAYPOINT_2 = "AUTO_WAYPOINT_2"
    MANUAL_SEARCH_DROP_OUTDOOR = "MANUAL_SEARCH_DROP_OUTDOOR"
    DROP_OUTDOOR = "DROP_OUTDOOR"
    ASCEND_TO_WAYPOINT_3 = "ASCEND_TO_WAYPOINT_3"
    AUTO_WAYPOINT_3_LANDING = "AUTO_WAYPOINT_3_LANDING"
    COMPLETED = "COMPLETED"

class CheckpointMissionNode(Node):
    def __init__(self):
        super().__init__('checkpoint_mission_node')
        
        # Load hardware configuration
        self.hw_config = HardwareConfig()
        self.hw_config.print_summary()
        
        # Validate configuration
        issues = self.hw_config.validate_config()
        if issues:
            self.get_logger().warn("Configuration issues found:")
            for issue in issues:
                self.get_logger().warn(f"  {issue}")
        
        # Debugging mode - can be set via parameter or config
        self.declare_parameter('debug_mode', self.hw_config.get_mission_debug_mode())
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info(f"🐛 Debug Mode: {'ENABLED' if self.debug_mode else 'DISABLED'}")
        if self.debug_mode:
            self.get_logger().info("   → Manual 'next' input required for each checkpoint")
        else:
            self.get_logger().info("   → Autonomous execution mode")
        
        # Current checkpoint
        self.current_checkpoint = MissionCheckpoint.INIT
        self.waiting_for_next = self.debug_mode  # Only wait if debug mode
        self.checkpoint_completed = False
        
        # Mission state
        self.item1_collected = False
        self.item2_collected = False
        self.indoor_items_dropped = False
        self.outdoor_item_collected = False
        self.mission_completed = False
        
        # Detection states
        self.item_detected = False
        self.item_position = Point()
        self.item_aligned = False
        self.dropzone_detected = False
        self.exit_detected = False
        
        # MAVLink connection to PX4
        self.mavlink_connection = None
        self.connect_to_px4()
        
        # Flight state
        self.armed = False
        self.mode = "MANUAL"
        self.altitude = 0.0
        self.position = Point()
        
        # Publishers
        self.checkpoint_status_pub = self.create_publisher(String, '/mission/checkpoint', 10)
        self.flight_command_pub = self.create_publisher(String, '/flight/command', 10)
        self.camera_enable_pub = self.create_publisher(String, '/camera/enable', 10)
        self.magnet_command_pub = self.create_publisher(String, '/magnet/command', 10)
        self.velocity_pub = self.create_publisher(Twist, '/drone/velocity', 10)
        
        # Subscribers
        self.user_input_sub = self.create_subscription(String, '/mission/user_input', self.user_input_callback, 10)
        self.vision_detection_sub = self.create_subscription(Point, '/vision/detection', self.vision_detection_callback, 10)
        self.alignment_status_sub = self.create_subscription(Bool, '/vision/aligned', self.alignment_callback, 10)
        
        # Timer for main mission loop and MAVLink heartbeat
        self.mission_timer = self.create_timer(0.5, self.mission_loop)
        self.mavlink_timer = self.create_timer(1.0, self.mavlink_heartbeat)
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("🚁 Checkpoint Mission Control Node Started")
        self.print_checkpoint_info()
    
    def connect_to_px4(self):
        """Connect to PX4 via MAVLink"""
        if mavutil is None:
            self.get_logger().error("❌ pymavlink not available")
            return
            
        try:
            # Get connection string from config
            connection_string = self.hw_config.get_px4_connection_string()
            self.get_logger().info(f"Connecting to PX4: {connection_string}")
            
            # Connect to PX4
            self.mavlink_connection = mavutil.mavlink_connection(connection_string)
            
            # Wait for heartbeat with timeout
            timeout = self.hw_config.get_connection_timeout()
            self.get_logger().info(f"Waiting for PX4 heartbeat (timeout: {timeout}s)...")
            
            self.mavlink_connection.wait_heartbeat(timeout=timeout)
            self.get_logger().info("✅ Connected to PX4!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to PX4: {e}")
            self.get_logger().error("Check:")
            self.get_logger().error(f"  1. PX4 connected to {self.hw_config.get_px4_port()}")
            self.get_logger().error("  2. Port permissions: sudo chmod 666 /dev/ttyUSB0")
            self.get_logger().error("  3. User in dialout group")
            self.mavlink_connection = None
    
    def mavlink_heartbeat(self):
        """Send heartbeat and process MAVLink messages"""
        if not self.mavlink_connection:
            return
            
        try:
            # Send heartbeat
            self.mavlink_connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            
            # Process incoming messages
            msg = self.mavlink_connection.recv_match(blocking=False)
            if msg:
                self.process_mavlink_message(msg)
                
        except Exception as e:
            self.get_logger().error(f"MAVLink error: {e}")
    
    def process_mavlink_message(self, msg):
        """Process incoming MAVLink messages"""
        if msg.get_type() == 'HEARTBEAT':
            self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            self.altitude = msg.relative_alt / 1000.0  # Convert mm to m
            self.position.x = msg.lat / 1e7
            self.position.y = msg.lon / 1e7
            self.position.z = self.altitude
    
    def send_mavlink_command(self, command, params=None):
        """Send command to PX4 via MAVLink"""
        if not self.mavlink_connection:
            self.get_logger().error("❌ No MAVLink connection")
            return False
            
        try:
            if command == "ARM":
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                
            elif command == "DISARM":
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                
            elif command == "TAKEOFF":
                altitude = params.get('altitude', 1.5) if params else 1.5
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude
                )
                
            elif command == "LAND":
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                
            elif command == "SET_ALTITUDE":
                altitude = params.get('altitude', 3.0) if params else 3.0
                self.mavlink_connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111111000,  # type_mask (only altitude)
                    0, 0, -altitude,  # position (NED: negative Z is up)
                    0, 0, 0,  # velocity
                    0, 0, 0,  # acceleration
                    0, 0  # yaw, yaw_rate
                )
                
            elif command == "SET_WAYPOINT":
                if params:
                    lat = params.get('lat')
                    lon = params.get('lon')
                    alt = params.get('alt')
                    
                    self.mavlink_connection.mav.set_position_target_global_int_send(
                        0,  # time_boot_ms
                        self.mavlink_connection.target_system,
                        self.mavlink_connection.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        0b0000111111111000,  # type_mask (only position)
                        int(lat * 1e7),  # lat in 1E7 degrees
                        int(lon * 1e7),  # lon in 1E7 degrees
                        alt,  # altitude
                        0, 0, 0,  # velocity
                        0, 0, 0,  # acceleration
                        0, 0  # yaw, yaw_rate
                    )
                
            elif command == "SET_MODE":
                mode = params.get('mode', 'STABILIZED') if params else 'STABILIZED'
                mode_mapping = {
                    'MANUAL': 1,
                    'STABILIZED': 2,
                    'ALTITUDE': 3,
                    'POSITION': 4,
                    'OFFBOARD': 6,
                    'LAND': 9
                }
                
                if mode in mode_mapping:
                    self.mavlink_connection.mav.set_mode_send(
                        self.mavlink_connection.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_mapping[mode]
                    )
                    
            return True
            
        except Exception as e:
            self.get_logger().error(f"MAVLink command error: {e}")
            return False
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity command to drone"""
        if not self.mavlink_connection:
            return
            
        self.mavlink_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask (only velocity)
            0, 0, 0,  # position
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration
            0, yaw_rate  # yaw, yaw_rate
        )
    
    def input_handler(self):
        """Handle user input for checkpoint progression"""
        while rclpy.ok():
            try:
                print("\n" + "="*50)
                print(f"🎯 Current Checkpoint: {self.current_checkpoint.value}")
                print(f"🐛 Debug Mode: {'ON' if self.debug_mode else 'OFF'}")
                print("Commands:")
                print("  'next' - Continue to next checkpoint")
                print("  'status' - Show current status")
                print("  'debug' - Toggle debug mode")
                print("  'help' - Show help")
                print("  'abort' - Abort mission")
                print("="*50)
                
                user_input = input("Enter command: ").strip().lower()
                
                if user_input == 'next' and self.waiting_for_next:
                    self.waiting_for_next = False
                    self.checkpoint_completed = True
                    self.get_logger().info("✅ User confirmed - proceeding to next checkpoint")
                    
                elif user_input == 'status':
                    self.print_status()
                    
                elif user_input == 'debug':
                    self.toggle_debug_mode()
                    
                elif user_input == 'help':
                    self.print_help()
                    
                elif user_input == 'abort':
                    self.abort_mission()
                    break
                    
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("Input handler terminated")
                break
            except Exception as e:
                self.get_logger().error(f"Input handler error: {e}")
    
    def print_checkpoint_info(self):
        """Print current checkpoint information"""
        print("\n" + "🚁"*20)
        print(f"CHECKPOINT: {self.current_checkpoint.value}")
        print("🚁"*20)
        
        checkpoint_descriptions = {
            MissionCheckpoint.INIT: "Initialize systems and arm drone",
            MissionCheckpoint.TAKEOFF: "Takeoff to 1.0m altitude", 
            MissionCheckpoint.SEARCH_ITEM_1_FRONT: "Move forward, activate front camera, search for item 1",
            MissionCheckpoint.ALIGN_ITEM_1: "Align drone to center item 1 in camera view",
            MissionCheckpoint.PICKUP_ITEM_1: "Descend and pickup item 1 with front magnet",
            MissionCheckpoint.SEARCH_ITEM_2_BACK: "Move forward, activate back camera, search for item 2",
            MissionCheckpoint.ALIGN_ITEM_2: "Align drone to center item 2 in camera view",
            MissionCheckpoint.PICKUP_ITEM_2: "Descend and pickup item 2 with back magnet",
            MissionCheckpoint.NAVIGATE_TURN_DIRECTION: "Navigate turn (left/right based on config)",
            MissionCheckpoint.SEARCH_DROPZONE: "Search for dropzone baskets",
            MissionCheckpoint.DROP_ITEM_1_FRONT: "Drop item 1 with front magnet",
            MissionCheckpoint.ASCEND_AFTER_DROP_1: "Ascend after dropping item 1",
            MissionCheckpoint.ALIGN_DROP_2_BACK: "Switch to back camera and align for item 2 drop",
            MissionCheckpoint.DROP_ITEM_2_BACK: "Drop item 2 with back magnet",
            MissionCheckpoint.FIND_EXIT: "Find exit gate with top camera",
            MissionCheckpoint.ASCEND_TO_OUTDOOR: "Ascend to 3m for outdoor mission",
            MissionCheckpoint.AUTO_WAYPOINT_1: "AUTO mode - Fly to waypoint 1",
            MissionCheckpoint.MANUAL_SEARCH_OUTDOOR: "MANUAL mode - Search for outdoor item",
            MissionCheckpoint.PICKUP_OUTDOOR: "Pickup outdoor item with front magnet only",
            MissionCheckpoint.ASCEND_TO_WAYPOINT_2: "Ascend to 3m for waypoint 2",
            MissionCheckpoint.AUTO_WAYPOINT_2: "AUTO mode - Fly to waypoint 2",
            MissionCheckpoint.MANUAL_SEARCH_DROP_OUTDOOR: "MANUAL mode - Search for outdoor dropzone",
            MissionCheckpoint.DROP_OUTDOOR: "Drop outdoor item",
            MissionCheckpoint.ASCEND_TO_WAYPOINT_3: "Ascend for final waypoint",
            MissionCheckpoint.AUTO_WAYPOINT_3_LANDING: "AUTO mode - Fly to waypoint 3 and land",
            MissionCheckpoint.COMPLETED: "Mission completed successfully"
        }
        
        description = checkpoint_descriptions.get(self.current_checkpoint, "Unknown checkpoint")
        print(f"Description: {description}")
        
        if self.waiting_for_next:
            print("⏳ Waiting for 'next' command to proceed...")
        else:
            print("🔄 Executing checkpoint...")
        print()
    
    def print_status(self):
        """Print current system status"""
        print("\n" + "📊"*20)
        print("SYSTEM STATUS")
        print("📊"*20)
        print(f"Armed: {self.armed}")
        print(f"Mode: {self.mode}")
        print(f"Altitude: {self.altitude:.2f}m")
        print(f"Position: ({self.position.x:.2f}, {self.position.y:.2f})")
        print(f"Item 1 Collected: {self.item1_collected}")
        print(f"Item 2 Collected: {self.item2_collected}")
        print(f"Indoor Items Dropped: {self.indoor_items_dropped}")
        print(f"Outdoor Item Collected: {self.outdoor_item_collected}")
        print(f"Item Detected: {self.item_detected}")
        print(f"Item Aligned: {self.item_aligned}")
        print()
    
    def print_help(self):
        """Print help information"""
        print("\n" + "❓"*20)
        print("HELP - DEBUGGING COMMANDS")
        print("❓"*20)
        print("next    - Proceed to next checkpoint")
        print("status  - Show system status")
        print("debug   - Toggle debug mode ON/OFF")
        print("help    - Show this help")
        print("abort   - Abort mission and land")
        print()
    
    def toggle_debug_mode(self):
        """Toggle debug mode on/off"""
        self.debug_mode = not self.debug_mode
        self.get_logger().info(f"🐛 Debug Mode: {'ENABLED' if self.debug_mode else 'DISABLED'}")
        
        if self.debug_mode:
            self.get_logger().info("   → Manual 'next' input required for each checkpoint")
            # If we just enabled debug mode and not waiting, start waiting
            if not self.waiting_for_next:
                self.waiting_for_next = True
        else:
            self.get_logger().info("   → Autonomous execution mode")
            # If we just disabled debug mode and waiting, continue
            if self.waiting_for_next:
                self.waiting_for_next = False
                self.checkpoint_completed = True
        
        print(f"\n🔄 Debug mode {'ENABLED' if self.debug_mode else 'DISABLED'}")
        if self.debug_mode:
            print("Mission will now wait for 'next' command at each checkpoint")
        else:
            print("Mission will now run autonomously")
        
    def abort_mission(self):
        """Abort mission and initiate emergency landing"""
        self.get_logger().warn("🚨 MISSION ABORTED - EMERGENCY LANDING")
        self.send_mavlink_command("LAND")
        self.current_checkpoint = MissionCheckpoint.LANDING
        
    def vision_detection_callback(self, msg):
        """Handle vision detection results"""
        if msg.x > 0 and msg.y > 0:  # Valid detection
            self.item_detected = True
            self.item_position = msg
            self.get_logger().info(f"🎯 Item detected at: ({msg.x:.2f}, {msg.y:.2f})")
    
    def alignment_callback(self, msg):
        """Handle alignment status"""
        self.item_aligned = msg.data
        if self.item_aligned:
            self.get_logger().info("✅ Item aligned for pickup")
    
    def user_input_callback(self, msg):
        """Handle user input from ROS topic (for external control)"""
        if msg.data == 'next' and self.waiting_for_next:
            self.waiting_for_next = False
            self.checkpoint_completed = True
    
    def mission_loop(self):
        """Main mission control loop"""
        if not self.checkpoint_completed or self.waiting_for_next:
            return
            
        # Reset checkpoint completion flag
        self.checkpoint_completed = False
        
        # Execute current checkpoint
        self.execute_checkpoint()
        
        # Advance to next checkpoint
        self.advance_checkpoint()
    
    def execute_checkpoint(self):
        """Execute the current checkpoint"""
        checkpoint = self.current_checkpoint
        
        self.get_logger().info(f"🔄 Executing checkpoint: {checkpoint.value}")
        
        if checkpoint == MissionCheckpoint.INIT:
            self.get_logger().info("🚁 Initializing mission systems...")
            self.send_mavlink_command("SET_MODE", {"mode": "STABILIZED"})
            time.sleep(1)
            self.send_mavlink_command("ARM")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.TAKEOFF:
            self.get_logger().info("🚀 Executing takeoff to 1.0m...")
            self.send_mavlink_command("SET_MODE", {"mode": "POSITION"})
            time.sleep(1)
            self.send_mavlink_command("TAKEOFF", {"altitude": self.hw_config.get_indoor_altitude()})
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.SEARCH_ITEM_1_FRONT:
            self.get_logger().info("🔍 Moving forward, activating front camera...")
            self.publish_camera_command("enable:front")
            self.send_velocity_command(0.5, 0, 0)  # Move forward
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ALIGN_ITEM_1:
            self.get_logger().info("🎯 Aligning with item 1...")
            if self.item_detected:
                # Calculate alignment movement based on item position
                center_x = 320  # Assuming 640px camera width
                center_y = 240  # Assuming 480px camera height
                
                error_x = self.item_position.x - center_x
                error_y = self.item_position.y - center_y
                
                # Convert pixel error to velocity commands
                vx = -error_y * 0.001  # Forward/backward
                vy = -error_x * 0.001  # Left/right
                
                self.send_velocity_command(vx, vy, 0)
                
                if abs(error_x) < 20 and abs(error_y) < 20:  # Close enough
                    self.item_aligned = True
                    self.send_velocity_command(0, 0, 0)  # Stop
                    
            self.wait_for_user_confirmation()
                
        elif checkpoint == MissionCheckpoint.PICKUP_ITEM_1:
            self.get_logger().info("📦 Picking up item 1 with front magnet...")
            self.send_velocity_command(0, 0, 0.3)  # Descend
            time.sleep(2)
            self.publish_magnet_command("front:on")
            time.sleep(1)
            self.send_velocity_command(0, 0, -0.3)  # Ascend
            time.sleep(2)
            self.send_velocity_command(0, 0, 0)  # Stop
            self.item1_collected = True
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.SEARCH_ITEM_2_BACK:
            self.get_logger().info("🔍 Moving forward, activating back camera...")
            self.publish_camera_command("disable:front")
            self.publish_camera_command("enable:back")
            self.send_velocity_command(0.5, 0, 0)  # Move forward
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ALIGN_ITEM_2:
            self.get_logger().info("🎯 Aligning with item 2...")
            if self.item_detected:
                center_x = 320
                center_y = 240
                error_x = self.item_position.x - center_x
                error_y = self.item_position.y - center_y
                vx = -error_y * 0.001
                vy = -error_x * 0.001
                self.send_velocity_command(vx, vy, 0)
                
                if abs(error_x) < 20 and abs(error_y) < 20:
                    self.item_aligned = True
                    self.send_velocity_command(0, 0, 0)
                    
            self.wait_for_user_confirmation()
                
        elif checkpoint == MissionCheckpoint.PICKUP_ITEM_2:
            self.get_logger().info("📦 Picking up item 2 with back magnet...")
            self.send_velocity_command(0, 0, 0.3)  # Descend
            time.sleep(2)
            self.publish_magnet_command("back:on")
            time.sleep(1)
            self.send_velocity_command(0, 0, -0.3)  # Ascend
            time.sleep(2)
            self.send_velocity_command(0, 0, 0)  # Stop
            self.item2_collected = True
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.NAVIGATE_TURN_DIRECTION:
            self.get_logger().info(f"🔄 Navigating turn ({self.hw_config.get_turn_direction()})...")
            self.publish_camera_command("disable:back")
            
            # Get turn direction from config
            turn_direction = self.hw_config.get_turn_direction()
            
            if turn_direction.lower() == "left":
                # Turn left maneuver
                self.send_velocity_command(0, 0.5, 0)  # Move left
                time.sleep(3)
                self.send_velocity_command(0.5, 0, 0)  # Move forward
                time.sleep(2)
            else:  # Default right turn
                # Turn right maneuver
                self.send_velocity_command(0, -0.5, 0)  # Move right
                time.sleep(3)
                self.send_velocity_command(0.5, 0, 0)  # Move forward
                time.sleep(2)
                
            self.send_velocity_command(0, 0, 0)  # Stop
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.SEARCH_DROPZONE:
            self.get_logger().info("🎪 Searching for dropzone baskets...")
            self.publish_camera_command("enable:front")
            # Slow search pattern
            self.send_velocity_command(0.2, 0, 0)
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.DROP_ITEM_1_FRONT:
            self.get_logger().info("📦 Dropping item 1 (front magnet)...")
            self.send_velocity_command(0, 0, 0)  # Stop and hover
            self.publish_magnet_command("front:off")  # Drop item 1
            time.sleep(2)  # Wait for drop
            self.get_logger().info("✅ Item 1 dropped")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_AFTER_DROP_1:
            self.get_logger().info("⬆️ Ascending after dropping item 1...")
            self.send_velocity_command(0, 0, -0.3)  # Ascend
            time.sleep(2)
            self.send_velocity_command(0, 0, 0)  # Stop
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ALIGN_DROP_2_BACK:
            self.get_logger().info("🎯 Switching to back camera and aligning for item 2 drop...")
            self.publish_camera_command("disable:front")
            self.publish_camera_command("enable:back")
            
            # Move to align for back camera drop
            if self.dropzone_detected:
                # Similar alignment logic as before
                center_x = 320
                center_y = 240
                error_x = self.item_position.x - center_x
                error_y = self.item_position.y - center_y
                vx = -error_y * 0.001
                vy = -error_x * 0.001
                self.send_velocity_command(vx, vy, 0)
                
                if abs(error_x) < 20 and abs(error_y) < 20:
                    self.item_aligned = True
                    self.send_velocity_command(0, 0, 0)
            
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.DROP_ITEM_2_BACK:
            self.get_logger().info("📦 Dropping item 2 (back magnet)...")
            self.send_velocity_command(0, 0, 0)  # Stop and hover
            self.publish_magnet_command("back:off")  # Drop item 2
            time.sleep(2)  # Wait for drop
            self.indoor_items_dropped = True
            self.get_logger().info("✅ Item 2 dropped - Indoor mission complete!")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.FIND_EXIT:
            self.get_logger().info("🚪 Searching for exit gate...")
            self.publish_camera_command("disable:back")
            self.publish_camera_command("enable:top")
            self.send_velocity_command(0.3, 0, 0)  # Move toward exit
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_TO_OUTDOOR:
            self.get_logger().info("⬆️ Ascending to outdoor altitude (3m)...")
            self.publish_camera_command("disable:all")
            outdoor_alt = self.hw_config.get_outdoor_altitude()
            self.send_mavlink_command("SET_ALTITUDE", {"altitude": outdoor_alt})
            self.get_logger().info(f"🛫 Climbing to {outdoor_alt}m for outdoor mission")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.AUTO_WAYPOINT_1:
            self.get_logger().info("🛰️ AUTO mode - Flying to Waypoint 1...")
            wp1 = self.hw_config.get_waypoint_1()
            self.send_mavlink_command("SET_MODE", {"mode": "AUTO"})
            self.send_mavlink_command("SET_WAYPOINT", {
                "lat": wp1['lat'], 
                "lon": wp1['lon'], 
                "alt": wp1['alt']
            })
            self.get_logger().info(f"📍 Target: ({wp1['lat']:.6f}, {wp1['lon']:.6f}, {wp1['alt']}m)")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.MANUAL_SEARCH_OUTDOOR:
            self.get_logger().info("🔍 Switching to MANUAL - Searching for outdoor item...")
            self.send_mavlink_command("SET_MODE", {"mode": "POSITION"})
            self.publish_camera_command("enable:front")  # Only front camera for outdoor
            self.send_velocity_command(0.2, 0, 0)  # Slow forward search
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.PICKUP_OUTDOOR:
            self.get_logger().info("📦 Picking up outdoor item (front magnet only)...")
            # Only use front magnet and front camera for outdoor pickup
            if self.item_detected:
                center_x = 320
                center_y = 240
                error_x = self.item_position.x - center_x
                error_y = self.item_position.y - center_y
                vx = -error_y * 0.001
                vy = -error_x * 0.001
                self.send_velocity_command(vx, vy, 0)
                
                if abs(error_x) < 20 and abs(error_y) < 20:
                    self.send_velocity_command(0, 0, 0.3)  # Descend
                    time.sleep(2)
                    self.publish_magnet_command("front:on")
                    time.sleep(1)
                    self.send_velocity_command(0, 0, -0.3)  # Ascend
                    time.sleep(2)
                    self.send_velocity_command(0, 0, 0)  # Stop
                    self.outdoor_item_collected = True
                    
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_TO_WAYPOINT_2:
            self.get_logger().info("⬆️ Ascending to 3m for Waypoint 2...")
            outdoor_alt = self.hw_config.get_outdoor_altitude()
            self.send_mavlink_command("SET_ALTITUDE", {"altitude": outdoor_alt})
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.AUTO_WAYPOINT_2:
            self.get_logger().info("🛰️ AUTO mode - Flying to Waypoint 2...")
            wp2 = self.hw_config.get_waypoint_2()
            self.send_mavlink_command("SET_MODE", {"mode": "AUTO"})
            self.send_mavlink_command("SET_WAYPOINT", {
                "lat": wp2['lat'], 
                "lon": wp2['lon'], 
                "alt": wp2['alt']
            })
            self.get_logger().info(f"📍 Target: ({wp2['lat']:.6f}, {wp2['lon']:.6f}, {wp2['alt']}m)")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.MANUAL_SEARCH_DROP_OUTDOOR:
            self.get_logger().info("🎪 Switching to MANUAL - Searching for outdoor dropzone...")
            self.send_mavlink_command("SET_MODE", {"mode": "POSITION"})
            self.send_velocity_command(0.2, 0, 0)  # Slow search
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.DROP_OUTDOOR:
            self.get_logger().info("📦 Dropping outdoor item...")
            self.send_velocity_command(0, 0, 0)  # Stop and hover
            self.publish_magnet_command("front:off")  # Drop outdoor item
            time.sleep(2)
            self.get_logger().info("✅ Outdoor item dropped!")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_TO_WAYPOINT_3:
            self.get_logger().info("⬆️ Ascending for final waypoint...")
            outdoor_alt = self.hw_config.get_outdoor_altitude()
            self.send_mavlink_command("SET_ALTITUDE", {"altitude": outdoor_alt})
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.AUTO_WAYPOINT_3_LANDING:
            self.get_logger().info("🏁 AUTO mode - Flying to Waypoint 3 (Landing)...")
            wp3 = self.hw_config.get_waypoint_3()
            self.send_mavlink_command("SET_MODE", {"mode": "AUTO"})
            self.send_mavlink_command("SET_WAYPOINT", {
                "lat": wp3['lat'], 
                "lon": wp3['lon'], 
                "alt": wp3['alt']
            })
            self.get_logger().info(f"📍 Final target: ({wp3['lat']:.6f}, {wp3['lon']:.6f}, {wp3['alt']}m)")
            
            # After reaching waypoint 3, initiate landing
            time.sleep(5)  # Wait to reach waypoint
            self.send_mavlink_command("SET_MODE", {"mode": "LAND"})
            self.send_mavlink_command("LAND")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.COMPLETED:
            self.get_logger().info("🎉 Mission completed successfully!")
            self.mission_completed = True
    
    def wait_for_user_confirmation(self):
        """Set flag to wait for user confirmation (only in debug mode)"""
        if self.debug_mode:
            self.waiting_for_next = True
            self.print_checkpoint_info()
            self.get_logger().info("⏳ Waiting for 'next' command to continue...")
        else:
            # In autonomous mode, continue automatically with small delay
            time.sleep(1.0)  # Small delay for stability
    
    def advance_checkpoint(self):
        """Advance to the next checkpoint"""
        current_index = list(MissionCheckpoint).index(self.current_checkpoint)
        if current_index < len(MissionCheckpoint) - 1:
            self.current_checkpoint = list(MissionCheckpoint)[current_index + 1]
            self.item_detected = False
            self.item_aligned = False
            
            # Publish checkpoint status
            msg = String()
            msg.data = self.current_checkpoint.value
            self.checkpoint_status_pub.publish(msg)
        else:
            self.get_logger().info("🎉 All checkpoints completed!")
    
    def publish_camera_command(self, command):
        """Publish camera control commands"""
        msg = String()
        msg.data = command
        self.camera_enable_pub.publish(msg)
        self.get_logger().info(f"📷 Camera command: {command}")
    
    def publish_magnet_command(self, command):
        """Publish magnet control commands"""
        msg = String()
        msg.data = command
        self.magnet_command_pub.publish(msg)
        self.get_logger().info(f"🧲 Magnet command: {command}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CheckpointMissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
