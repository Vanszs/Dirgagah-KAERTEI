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

# MAVROS imports
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPointStamped

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
        
        # MAVROS state
        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.gps_position = NavSatFix()
        
        # Publishers
        self.checkpoint_status_pub = self.create_publisher(String, '/mission/checkpoint', 10)
        self.camera_enable_pub = self.create_publisher(String, '/camera/enable', 10)
        self.magnet_command_pub = self.create_publisher(String, '/magnet/command', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.global_position_pub = self.create_publisher(GeoPointStamped, '/mavros/setpoint_position/global', 10)
        
        # Subscribers
        self.user_input_sub = self.create_subscription(String, '/mission/user_input', self.user_input_callback, 10)
        self.vision_detection_sub = self.create_subscription(Point, '/vision/detection', self.vision_detection_callback, 10)
        self.alignment_status_sub = self.create_subscription(Bool, '/vision/aligned', self.alignment_callback, 10)
        
        # MAVROS subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        
        # MAVROS service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.landing_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for MAVROS connection
        self.wait_for_mavros_connection()
        
        # Timer for main mission loop
        self.mission_timer = self.create_timer(0.5, self.mission_loop)
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("🚁 Checkpoint Mission Control Node Started (MAVROS)")
        self.print_checkpoint_info()
    
    def wait_for_mavros_connection(self):
        """Wait for MAVROS to connect to flight controller"""
        self.get_logger().info("⏳ Waiting for MAVROS connection...")
        
        # Wait for services to become available
        services_to_wait = [
            self.arming_client,
            self.takeoff_client,
            self.set_mode_client
        ]
        
        for service in services_to_wait:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for service {service.srv_name}...")
        
        # Wait for MAVROS state messages
        timeout = 30.0  # 30 seconds timeout
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            if self.mavros_state.connected:
                self.get_logger().info("✅ MAVROS connected to flight controller!")
                return
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if not self.mavros_state.connected:
            self.get_logger().error("❌ Failed to connect to flight controller via MAVROS!")
            self.get_logger().error("Check:")
            self.get_logger().error("  1. Flight controller is connected and powered")
            self.get_logger().error("  2. MAVROS is running: ros2 launch mavros px4.launch")
            self.get_logger().error("  3. Correct port configuration in MAVROS launch file")
    
    def state_callback(self, msg):
        """Handle MAVROS state updates"""
        self.mavros_state = msg
    
    def pose_callback(self, msg):
        """Handle local position updates"""
        self.current_pose = msg
    
    def gps_callback(self, msg):
        """Handle GPS position updates"""
        self.gps_position = msg
    
    def mission_loop(self):
        """Main mission execution loop"""
        if not self.mavros_state.connected:
            return
        
        if self.waiting_for_next:
            return
        
        if not self.checkpoint_completed:
            self.execute_checkpoint()
            self.checkpoint_completed = True
        
        # Small delay before advancing
        time.sleep(0.5)
        
        # Advance to next checkpoint
        self.advance_checkpoint()
    
    def execute_checkpoint(self):
        """Execute the current checkpoint"""
        checkpoint = self.current_checkpoint
        
        self.get_logger().info(f"🔄 Executing checkpoint: {checkpoint.value}")
        
        if checkpoint == MissionCheckpoint.INIT:
            self.get_logger().info("🚁 Initializing mission systems...")
            self.set_flight_mode("STABILIZED")
            time.sleep(1)
            self.arm_drone()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.TAKEOFF:
            self.get_logger().info("🚀 Executing takeoff to 0.6m...")
            altitude = self.hw_config.get_indoor_altitude()
            self.takeoff_drone(altitude)
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.SEARCH_ITEM_1_FRONT:
            self.get_logger().info("🔍 Moving forward, activating front camera...")
            self.publish_camera_command("enable:front")
            self.send_velocity_command(0.5, 0, 0)  # Move forward
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ALIGN_ITEM_1:
            self.get_logger().info("🎯 Aligning with item 1...")
            if self.item_detected:
                self.align_with_item()
            self.wait_for_user_confirmation()
                
        elif checkpoint == MissionCheckpoint.PICKUP_ITEM_1:
            self.get_logger().info("📦 Picking up item 1 with front magnet...")
            self.pickup_sequence("front")
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
                self.align_with_item()
            self.wait_for_user_confirmation()
                
        elif checkpoint == MissionCheckpoint.PICKUP_ITEM_2:
            self.get_logger().info("📦 Picking up item 2 with back magnet...")
            self.pickup_sequence("back")
            self.item2_collected = True
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.NAVIGATE_TURN_DIRECTION:
            self.get_logger().info(f"🔄 Navigating turn ({self.hw_config.get_turn_direction()})...")
            self.execute_turn()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.SEARCH_DROPZONE:
            self.get_logger().info("🎪 Searching for dropzone baskets...")
            self.publish_camera_command("enable:front")
            self.send_velocity_command(0.2, 0, 0)  # Slow search
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.DROP_ITEM_1_FRONT:
            self.get_logger().info("📦 Dropping item 1 (front magnet)...")
            self.drop_item("front")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_AFTER_DROP_1:
            self.get_logger().info("⬆️ Ascending after dropping item 1...")
            self.send_velocity_command(0, 0, -0.3)  # Ascend
            time.sleep(2)
            self.send_velocity_command(0, 0, 0)  # Stop
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ALIGN_DROP_2_BACK:
            self.get_logger().info("🎯 Switching to back camera and aligning for item 2 drop...")
            self.switch_camera_and_align()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.DROP_ITEM_2_BACK:
            self.get_logger().info("📦 Dropping item 2 (back magnet)...")
            self.drop_item("back")
            self.indoor_items_dropped = True
            self.get_logger().info("✅ Indoor mission complete!")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.FIND_EXIT:
            self.get_logger().info("🚪 Searching for exit gate...")
            self.search_exit()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_TO_OUTDOOR:
            self.get_logger().info("⬆️ Ascending to outdoor altitude (3m)...")
            self.ascend_to_outdoor()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.AUTO_WAYPOINT_1:
            self.get_logger().info("🛰️ AUTO mode - Flying to Waypoint 1...")
            self.fly_to_waypoint(1)
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.MANUAL_SEARCH_OUTDOOR:
            self.get_logger().info("🔍 MANUAL mode - Searching for outdoor item...")
            self.manual_search_outdoor()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.PICKUP_OUTDOOR:
            self.get_logger().info("📦 Picking up outdoor item (front magnet only)...")
            self.pickup_outdoor()
            self.outdoor_item_collected = True
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_TO_WAYPOINT_2:
            self.get_logger().info("⬆️ Ascending to 3m for Waypoint 2...")
            self.ascend_to_outdoor()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.AUTO_WAYPOINT_2:
            self.get_logger().info("🛰️ AUTO mode - Flying to Waypoint 2...")
            self.fly_to_waypoint(2)
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.MANUAL_SEARCH_DROP_OUTDOOR:
            self.get_logger().info("🎪 MANUAL mode - Searching for outdoor dropzone...")
            self.manual_search_dropzone()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.DROP_OUTDOOR:
            self.get_logger().info("📦 Dropping outdoor item...")
            self.drop_item("front")
            self.get_logger().info("✅ Outdoor item dropped!")
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.ASCEND_TO_WAYPOINT_3:
            self.get_logger().info("⬆️ Ascending for final waypoint...")
            self.ascend_to_outdoor()
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.AUTO_WAYPOINT_3_LANDING:
            self.get_logger().info("🏁 AUTO mode - Flying to Waypoint 3 and landing...")
            self.fly_to_waypoint_and_land(3)
            self.wait_for_user_confirmation()
            
        elif checkpoint == MissionCheckpoint.COMPLETED:
            self.get_logger().info("🎉 Mission completed successfully!")
            self.mission_completed = True
    
    # MAVROS helper methods
    def set_flight_mode(self, mode):
        """Set flight mode via MAVROS"""
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"✅ Flight mode set to: {mode}")
            return True
        else:
            self.get_logger().error(f"❌ Failed to set flight mode to: {mode}")
            return False
    
    def arm_drone(self):
        """Arm the drone via MAVROS"""
        req = CommandBool.Request()
        req.value = True
        
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Drone armed successfully")
            return True
        else:
            self.get_logger().error("❌ Failed to arm drone")
            return False
    
    def takeoff_drone(self, altitude):
        """Takeoff to specified altitude via MAVROS"""
        req = CommandTOL.Request()
        req.altitude = altitude
        
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"✅ Takeoff to {altitude}m initiated")
            return True
        else:
            self.get_logger().error(f"❌ Failed to takeoff to {altitude}m")
            return False
    
    def land_drone(self):
        """Land the drone via MAVROS"""
        req = CommandTOL.Request()
        
        future = self.landing_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Landing initiated")
            return True
        else:
            self.get_logger().error("❌ Failed to initiate landing")
            return False
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity command via MAVROS"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        
        self.velocity_pub.publish(msg)
    
    def send_position_command(self, x, y, z, yaw=0.0):
        """Send position command via MAVROS"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Convert yaw to quaternion (simplified)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.z = yaw
        
        self.position_pub.publish(msg)
    
    def send_global_position_command(self, lat, lon, alt):
        """Send global position command via MAVROS"""
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.position.latitude = lat
        msg.position.longitude = lon
        msg.position.altitude = alt
        
        self.global_position_pub.publish(msg)
    
    # Mission execution helper methods
    def align_with_item(self):
        """Align drone with detected item"""
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
    
    def pickup_sequence(self, magnet):
        """Execute pickup sequence"""
        self.send_velocity_command(0, 0, 0.3)  # Descend
        time.sleep(2)
        self.publish_magnet_command(f"{magnet}:on")
        time.sleep(1)
        self.send_velocity_command(0, 0, -0.3)  # Ascend
        time.sleep(2)
        self.send_velocity_command(0, 0, 0)  # Stop
    
    def drop_item(self, magnet):
        """Drop item with specified magnet"""
        self.send_velocity_command(0, 0, 0)  # Stop and hover
        self.publish_magnet_command(f"{magnet}:off")
        time.sleep(2)  # Wait for drop
    
    def execute_turn(self):
        """Execute turn based on configuration"""
        self.publish_camera_command("disable:back")
        
        turn_direction = self.hw_config.get_turn_direction()
        
        if turn_direction.lower() == "left":
            self.send_velocity_command(0, 0.5, 0)  # Move left
            time.sleep(3)
            self.send_velocity_command(0.5, 0, 0)  # Move forward
            time.sleep(2)
        else:  # Default right turn
            self.send_velocity_command(0, -0.5, 0)  # Move right
            time.sleep(3)
            self.send_velocity_command(0.5, 0, 0)  # Move forward
            time.sleep(2)
            
        self.send_velocity_command(0, 0, 0)  # Stop
    
    def switch_camera_and_align(self):
        """Switch to back camera and align for drop"""
        self.publish_camera_command("disable:front")
        self.publish_camera_command("enable:back")
        
        if self.dropzone_detected:
            self.align_with_item()
    
    def search_exit(self):
        """Search for exit with top camera"""
        self.publish_camera_command("disable:back")
        self.publish_camera_command("enable:top")
        self.send_velocity_command(0.3, 0, 0)  # Move toward exit
    
    def ascend_to_outdoor(self):
        """Ascend to outdoor altitude"""
        outdoor_alt = self.hw_config.get_outdoor_altitude()
        current_z = self.current_pose.pose.position.z
        target_z = current_z + (outdoor_alt - self.hw_config.get_indoor_altitude())
        
        self.send_position_command(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            target_z
        )
        
        self.get_logger().info(f"🛫 Climbing to {outdoor_alt}m for outdoor mission")
    
    def fly_to_waypoint(self, waypoint_num):
        """Fly to specified GPS waypoint"""
        self.set_flight_mode("AUTO")
        
        if waypoint_num == 1:
            wp = self.hw_config.get_waypoint_1()
        elif waypoint_num == 2:
            wp = self.hw_config.get_waypoint_2()
        elif waypoint_num == 3:
            wp = self.hw_config.get_waypoint_3()
        else:
            self.get_logger().error(f"Invalid waypoint number: {waypoint_num}")
            return
        
        self.send_global_position_command(wp['lat'], wp['lon'], wp['alt'])
        self.get_logger().info(f"📍 Target: ({wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']}m)")
    
    def fly_to_waypoint_and_land(self, waypoint_num):
        """Fly to waypoint and then land"""
        self.fly_to_waypoint(waypoint_num)
        
        # Wait a bit to reach waypoint, then land
        time.sleep(10)
        self.set_flight_mode("LAND")
        self.land_drone()
    
    def manual_search_outdoor(self):
        """Manual search for outdoor item"""
        self.set_flight_mode("POSITION")
        self.publish_camera_command("enable:front")
        self.send_velocity_command(0.2, 0, 0)  # Slow forward search
    
    def manual_search_dropzone(self):
        """Manual search for outdoor dropzone"""
        self.set_flight_mode("POSITION")
        self.send_velocity_command(0.2, 0, 0)  # Slow search
    
    def pickup_outdoor(self):
        """Pickup outdoor item with front magnet only"""
        if self.item_detected:
            self.align_with_item()
            if self.item_aligned:
                self.pickup_sequence("front")
    
    # ... (rest of the methods remain the same as previous version)
    # Input handling, status printing, etc.
    
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
            self.checkpoint_completed = False
            
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
    
    def input_handler(self):
        """Handle user input for checkpoint progression"""
        while rclpy.ok():
            try:
                print("\n" + "="*60)
                print(f"🎯 Current Checkpoint: {self.current_checkpoint.value}")
                print(f"🐛 Debug Mode: {'ON' if self.debug_mode else 'OFF'}")
                print(f"📡 MAVROS Connected: {'YES' if self.mavros_state.connected else 'NO'}")
                print(f"🔋 Armed: {'YES' if self.mavros_state.armed else 'NO'}")
                print(f"✈️  Mode: {self.mavros_state.mode}")
                print("Commands:")
                print("  'next'   - Continue to next checkpoint")
                print("  'status' - Show current status")
                print("  'debug'  - Toggle debug mode ON/OFF")
                print("  'help'   - Show help")
                print("  'abort'  - Abort mission and land")
                print("="*60)
                
                user_input = input("Enter command: ").strip().lower()
                
                if user_input == 'next' and self.waiting_for_next:
                    self.waiting_for_next = False
                    self.checkpoint_completed = False
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
        print("\n" + "🚁"*25)
        print(f"CHECKPOINT: {self.current_checkpoint.value}")
        print("🚁"*25)
        
        checkpoint_descriptions = {
            MissionCheckpoint.INIT: "Initialize systems and arm drone",
            MissionCheckpoint.TAKEOFF: "Takeoff to 0.6m altitude", 
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
        print("\n" + "📊"*25)
        print("SYSTEM STATUS")
        print("📊"*25)
        print(f"MAVROS Connected: {self.mavros_state.connected}")
        print(f"Armed: {self.mavros_state.armed}")
        print(f"Mode: {self.mavros_state.mode}")
        print(f"Position: ({self.current_pose.pose.position.x:.2f}, {self.current_pose.pose.position.y:.2f}, {self.current_pose.pose.position.z:.2f})")
        print(f"GPS: ({self.gps_position.latitude:.6f}, {self.gps_position.longitude:.6f})")
        print(f"Item 1 Collected: {self.item1_collected}")
        print(f"Item 2 Collected: {self.item2_collected}")
        print(f"Indoor Items Dropped: {self.indoor_items_dropped}")
        print(f"Outdoor Item Collected: {self.outdoor_item_collected}")
        print(f"Item Detected: {self.item_detected}")
        print(f"Item Aligned: {self.item_aligned}")
        print()
    
    def print_help(self):
        """Print help information"""
        print("\n" + "❓"*25)
        print("HELP - DEBUGGING COMMANDS")
        print("❓"*25)
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
            if not self.waiting_for_next:
                self.waiting_for_next = True
        else:
            self.get_logger().info("   → Autonomous execution mode")
            if self.waiting_for_next:
                self.waiting_for_next = False
                self.checkpoint_completed = False
        
        print(f"\n🔄 Debug mode {'ENABLED' if self.debug_mode else 'DISABLED'}")
    
    def abort_mission(self):
        """Abort mission and initiate emergency landing"""
        self.get_logger().warn("🚨 MISSION ABORTED - EMERGENCY LANDING")
        self.set_flight_mode("LAND")
        self.land_drone()
        self.current_checkpoint = MissionCheckpoint.COMPLETED
    
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
        command = msg.data.strip().lower()
        if command == 'next' and self.waiting_for_next:
            self.waiting_for_next = False
            self.checkpoint_completed = False
            self.get_logger().info("✅ External 'next' command received")

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
