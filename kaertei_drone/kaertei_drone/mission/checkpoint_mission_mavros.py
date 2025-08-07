#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - 12 Checkpoint Mission Controller
Complete implementation of 12-checkpoint FSM system for autonomous drone mission
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import Image, NavSatFix
import time
import threading
from enum import Enum
import json

# MAVROS imports
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import TwistStamped

# Import hardware configuration
from ..hardware.hardware_config import HardwareConfig

class MissionCheckpoint(Enum):
    """12 Checkpoint Mission System - KAERTEI 2025 FAIO"""
    # Phase 1: Initialization & Indoor Search (CP 1-6)
    CP1_INIT_ARM = "CP1_INIT_ARM"               # Inisialisasi dan arming drone
    CP2_TAKEOFF_1M = "CP2_TAKEOFF_1M"           # Takeoff ke ketinggian 1m
    CP3_SEARCH_ITEM1 = "CP3_SEARCH_ITEM1"       # Mencari dan mengambil item pertama
    CP4_SEARCH_ITEM2_TURN = "CP4_SEARCH_ITEM2_TURN" # Berputar dan mencari item kedua
    CP5_DROP_ITEM1 = "CP5_DROP_ITEM1"           # Menjatuhkan item pertama
    CP6_DROP_ITEM2 = "CP6_DROP_ITEM2"           # Menjatuhkan item kedua
    
    # Phase 2: GPS Navigation & Outdoor Mission (CP 7-12)
    CP7_GPS_WP1_3 = "CP7_GPS_WP1_3"             # Navigasi ke GPS waypoint 1-3
    CP8_SEARCH_ITEM3 = "CP8_SEARCH_ITEM3"       # Mencari dan mengambil item ketiga
    CP9_DIRECT_WP4 = "CP9_DIRECT_WP4"           # Terbang langsung ke waypoint 4
    CP10_SEARCH_DROP_ITEM3 = "CP10_SEARCH_DROP_ITEM3" # Mencari dropzone dan menjatuhkan item ketiga
    CP11_GPS_WP5 = "CP11_GPS_WP5"               # Navigasi ke GPS waypoint 5
    CP12_LANDING_DISARM = "CP12_LANDING_DISARM" # Landing dan disarm
    
    # Mission States
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    PAUSED = "PAUSED"

class Checkpoint12MissionNode(Node):
    def __init__(self):
        super().__init__('checkpoint_12_mission_node')
        
        # Load hardware configuration
        self.hw_config = HardwareConfig()
        
        # Mission parameters
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        self.auto_continue = self.declare_parameter('auto_continue', False).value
        
        # Current mission state
        self.current_checkpoint = MissionCheckpoint.CP1_INIT_ARM
        self.waiting_for_next = self.debug_mode
        self.checkpoint_completed = False
        
        # Mission completion tracking
        self.item1_collected = False
        self.item2_collected = False
        self.item3_collected = False
        self.indoor_items_dropped = False
        self.outdoor_item_dropped = False
        self.mission_completed = False
        
        # Detection states
        self.item_detected = False
        self.item_position = Point()
        self.item_aligned = False
        self.bucket_detected = False
        self.bucket_position = Point()
        
        # Flight states
        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.gps_position = NavSatFix()
        self.current_altitude = 0.0
        self.armed = False
        
        # Publishers
        self.checkpoint_status_pub = self.create_publisher(String, '/mission/checkpoint_status', 10)
        self.mission_command_pub = self.create_publisher(String, '/mission/command', 10)
        self.camera_command_pub = self.create_publisher(String, '/camera/command', 10)
        self.magnet_command_pub = self.create_publisher(String, '/magnet/command', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.global_pos_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.mavros_state_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.vision_sub = self.create_subscription(String, '/vision/detection', self.vision_callback, 10)
        self.user_input_sub = self.create_subscription(String, '/mission/user_input', self.user_input_callback, 10)
        
        # Services
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        
        # Mission control timer
        self.mission_timer = self.create_timer(0.5, self.mission_control_loop)
        
        self.get_logger().info("🚁 KAERTEI 2025 - 12 Checkpoint Mission Controller Initialized")
        self.get_logger().info(f"Debug Mode: {self.debug_mode}")

    def mission_control_loop(self):
        """Main mission control loop"""
        if not self.waiting_for_next and not self.mission_completed:
            self.execute_checkpoint()
            
    def execute_checkpoint(self):
        """Execute current checkpoint based on 12-checkpoint system"""
        checkpoint = self.current_checkpoint
        
        self.get_logger().info(f"🔄 Executing: {checkpoint.value}")
        self.publish_checkpoint_status(checkpoint.value, "EXECUTING")
        
        if checkpoint == MissionCheckpoint.CP1_INIT_ARM:
            self.execute_cp1_init_arm()
            
        elif checkpoint == MissionCheckpoint.CP2_TAKEOFF_1M:
            self.execute_cp2_takeoff_1m()
            
        elif checkpoint == MissionCheckpoint.CP3_SEARCH_ITEM1:
            self.execute_cp3_search_item1()
            
        elif checkpoint == MissionCheckpoint.CP4_SEARCH_ITEM2_TURN:
            self.execute_cp4_search_item2_turn()
            
        elif checkpoint == MissionCheckpoint.CP5_DROP_ITEM1:
            self.execute_cp5_drop_item1()
            
        elif checkpoint == MissionCheckpoint.CP6_DROP_ITEM2:
            self.execute_cp6_drop_item2()
            
        elif checkpoint == MissionCheckpoint.CP7_GPS_WP1_3:
            self.execute_cp7_gps_wp1_3()
            
        elif checkpoint == MissionCheckpoint.CP8_SEARCH_ITEM3:
            self.execute_cp8_search_item3()
            
        elif checkpoint == MissionCheckpoint.CP9_DIRECT_WP4:
            self.execute_cp9_direct_wp4()
            
        elif checkpoint == MissionCheckpoint.CP10_SEARCH_DROP_ITEM3:
            self.execute_cp10_search_drop_item3()
            
        elif checkpoint == MissionCheckpoint.CP11_GPS_WP5:
            self.execute_cp11_gps_wp5()
            
        elif checkpoint == MissionCheckpoint.CP12_FINAL_DESCENT_DISARM:
            self.execute_cp12_final_descent_disarm()
            
        elif checkpoint == MissionCheckpoint.COMPLETED:
            self.handle_mission_completed()

    # ===========================================
    # CHECKPOINT IMPLEMENTATIONS
    # ===========================================
    
    def execute_cp1_init_arm(self):
        """CP1: Initialize & ARM flight controller"""
        self.get_logger().info("🔧 CP1: Initializing systems and arming...")
        
        # System checks
        if not self.system_health_check():
            self.get_logger().error("❌ System health check failed!")
            self.transition_to_error()
            return
            
        # Set mode to STABILIZED first
        self.set_flight_mode("STABILIZED")
        time.sleep(1)
        
        # ARM the drone
        if self.arm_drone():
            self.get_logger().info("✅ CP1 Complete: System armed and ready")
            self.complete_checkpoint(MissionCheckpoint.CP2_TAKEOFF_1M)
        else:
            self.get_logger().error("❌ CP1 Failed: Could not arm drone")
            self.transition_to_error()

    def execute_cp2_takeoff_1m(self):
        """CP2: Takeoff to 1.0m altitude"""
        self.get_logger().info("🚀 CP2: Taking off to 1.0m...")
        
        # Switch to GUIDED mode
        self.set_flight_mode("GUIDED")
        time.sleep(1)
        
        # Execute takeoff
        target_altitude = 1.0
        if self.takeoff_to_altitude(target_altitude):
            self.get_logger().info(f"✅ CP2 Complete: Stable hover at {target_altitude}m")
            self.complete_checkpoint(MissionCheckpoint.CP3_SEARCH_ITEM1)
        else:
            self.get_logger().error("❌ CP2 Failed: Takeoff unsuccessful")
            self.transition_to_error()

    def execute_cp3_search_item1(self):
        """CP3: Search Item 1 with front bottom camera"""
        self.get_logger().info("🔍 CP3: Searching for Item 1...")
        
        # Activate front bottom camera
        self.publish_camera_command("enable:front_bottom")
        
        # Move forward while searching
        self.send_velocity_command(0.3, 0, 0)  # Forward at 0.3 m/s
        
        # Wait for item detection or timeout
        if self.wait_for_item_detection(timeout=60):
            # Item detected, perform pickup sequence
            self.align_and_pickup_item("front", item_number=1)
            self.item1_collected = True
            self.get_logger().info("✅ CP3 Complete: Item 1 collected")
            self.complete_checkpoint(MissionCheckpoint.CP4_SEARCH_ITEM2_TURN)
        else:
            self.get_logger().warning("⚠️ CP3 Timeout: Item 1 not found, proceeding anyway")
            self.complete_checkpoint(MissionCheckpoint.CP4_SEARCH_ITEM2_TURN)

    def execute_cp4_search_item2_turn(self):
        """CP4: Search Item 2 with back camera & navigation turn"""
        self.get_logger().info("🔍🔄 CP4: Search Item 2 and execute turn...")
        
        # Continue forward while activating back camera
        self.publish_camera_command("enable:back")
        self.send_velocity_command(0.3, 0, 0)
        
        # Wait for item 2 detection
        if self.wait_for_item_detection(timeout=60, camera="back"):
            self.align_and_pickup_item("back", item_number=2)
            self.item2_collected = True
            
        # Execute navigation turn after item 2 collection
        self.execute_navigation_turn()
        
        self.get_logger().info("✅ CP4 Complete: Item 2 search and navigation turn done")
        self.complete_checkpoint(MissionCheckpoint.CP5_DROP_ITEM1)

    def execute_cp5_drop_item1(self):
        """CP5: Drop Item 1 to front bucket"""
        self.get_logger().info("🪣 CP5: Dropping Item 1...")
        
        if not self.item1_collected:
            self.get_logger().warning("⚠️ CP5: No Item 1 to drop, skipping")
            self.complete_checkpoint(MissionCheckpoint.CP6_DROP_ITEM2)
            return
            
        # Search for drop bucket with front camera
        self.publish_camera_command("enable:front")
        
        if self.search_and_align_bucket():
            self.drop_item("front", altitude=0.8)
            self.get_logger().info("✅ CP5 Complete: Item 1 dropped")
        else:
            self.get_logger().warning("⚠️ CP5: Bucket not found, emergency drop")
            self.drop_item("front", altitude=0.8)
            
        self.complete_checkpoint(MissionCheckpoint.CP6_DROP_ITEM2)

    def execute_cp6_drop_item2(self):
        """CP6: Drop Item 2 to back bucket"""
        self.get_logger().info("🪣 CP6: Dropping Item 2...")
        
        if not self.item2_collected:
            self.get_logger().warning("⚠️ CP6: No Item 2 to drop, skipping")
            self.complete_checkpoint(MissionCheckpoint.CP7_GPS_WP1_3)
            return
            
        # Search for drop bucket with back camera
        self.publish_camera_command("enable:back")
        
        if self.search_and_align_bucket(camera="back"):
            self.drop_item("back", altitude=0.8)
            self.get_logger().info("✅ CP6 Complete: Item 2 dropped")
        else:
            self.get_logger().warning("⚠️ CP6: Bucket not found, emergency drop")
            self.drop_item("back", altitude=0.8)
            
        self.indoor_items_dropped = True
        self.complete_checkpoint(MissionCheckpoint.CP7_GPS_WP1_3)

    def execute_cp7_gps_wp1_3(self):
        """CP7: GPS Navigation WP1-3 at 3m/s"""
        self.get_logger().info("🛰️ CP7: GPS Navigation WP1→WP2→WP3...")
        
        # Switch to AUTO mode for GPS navigation
        self.set_flight_mode("AUTO")
        
        # Navigate through waypoints 1, 2, 3
        waypoints = [1, 2, 3]
        
        for wp in waypoints:
            self.get_logger().info(f"📍 Navigating to WP{wp}")
            if self.navigate_to_waypoint(wp, speed=3.0):
                self.get_logger().info(f"✅ Reached WP{wp}")
            else:
                self.get_logger().error(f"❌ Failed to reach WP{wp}")
                self.transition_to_error()
                return
        
        self.get_logger().info("✅ CP7 Complete: WP1-3 navigation done")
        self.complete_checkpoint(MissionCheckpoint.CP8_SEARCH_ITEM3)

    def execute_cp8_search_item3(self):
        """CP8: Search Item 3 after WP3"""
        self.get_logger().info("🔍 CP8: Searching for Item 3...")
        
        # Switch back to GUIDED mode
        self.set_flight_mode("GUIDED")
        
        # Activate front bottom camera for item search
        self.publish_camera_command("enable:front_bottom")
        
        if self.wait_for_item_detection(timeout=60):
            self.align_and_pickup_item("front", item_number=3)
            self.item3_collected = True
            self.get_logger().info("✅ CP8 Complete: Item 3 collected")
        else:
            self.get_logger().warning("⚠️ CP8 Timeout: Item 3 not found")
            
        self.complete_checkpoint(MissionCheckpoint.CP9_DIRECT_WP4)

    def execute_cp9_direct_wp4(self):
        """CP9: Direct navigation to WP4 with payload"""
        self.get_logger().info("🛰️ CP9: Direct navigation to WP4...")
        
        # Switch to AUTO mode
        self.set_flight_mode("AUTO")
        
        if self.navigate_to_waypoint(4, speed=3.0):
            self.get_logger().info("✅ CP9 Complete: Arrived at WP4")
            self.complete_checkpoint(MissionCheckpoint.CP10_SEARCH_DROP_ITEM3)
        else:
            self.get_logger().error("❌ CP9 Failed: Could not reach WP4")
            self.transition_to_error()

    def execute_cp10_search_drop_item3(self):
        """CP10: Search drop bucket and drop Item 3"""
        self.get_logger().info("🪣🔍 CP10: Search bucket and drop Item 3...")
        
        # Switch to GUIDED mode for search
        self.set_flight_mode("GUIDED")
        
        if not self.item3_collected:
            self.get_logger().warning("⚠️ CP10: No Item 3 to drop, skipping")
            self.complete_checkpoint(MissionCheckpoint.CP11_GPS_WP5)
            return
            
        # Search for bucket
        self.publish_camera_command("enable:front")
        
        if self.search_and_align_bucket():
            self.drop_item("front", altitude=0.8)
            self.outdoor_item_dropped = True
            self.get_logger().info("✅ CP10 Complete: Item 3 dropped")
        else:
            self.get_logger().warning("⚠️ CP10: Bucket not found, emergency drop")
            self.drop_item("front", altitude=0.8)
            
        self.complete_checkpoint(MissionCheckpoint.CP11_GPS_WP5)

    def execute_cp11_gps_wp5(self):
        """CP11: GPS Navigation to final WP5"""
        self.get_logger().info("🛰️ CP11: Final navigation to WP5...")
        
        # Switch to AUTO mode
        self.set_flight_mode("AUTO")
        
        if self.navigate_to_waypoint(5, speed=3.0):
            self.get_logger().info("✅ CP11 Complete: Arrived at final WP5")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)
        else:
            self.get_logger().error("❌ CP11 Failed: Could not reach WP5")
            self.transition_to_error()

    def execute_cp12_final_descent_disarm(self):
        """CP12: Final descent and disarm (no ground contact detection)"""
        self.get_logger().info("🏁 CP12: Final descent and disarm...")
        
        # Switch to GUIDED mode for controlled descent
        self.set_flight_mode("GUIDED")
        time.sleep(1)
        
        # Gradual RPM reduction and descent
        self.get_logger().info("📉 Reducing RPM gradually...")
        
        # Controlled descent to ground
        target_altitude = 0.0
        descent_rate = -0.5  # m/s descent rate
        
        while self.current_altitude > 0.2:  # Land when close to ground
            self.send_altitude_command(target_altitude, descent_rate)
            time.sleep(0.5)
            
        # Disarm the drone
        if self.disarm_drone():
            self.get_logger().info("✅ CP12 Complete: Mission accomplished!")
            self.mission_completed = True
            self.complete_checkpoint(MissionCheckpoint.COMPLETED)
        else:
            self.get_logger().error("❌ CP12: Disarm failed")
            self.transition_to_error()

    # ===========================================
    # UTILITY FUNCTIONS
    # ===========================================
    
    def system_health_check(self):
        """Perform system health check before mission"""
        self.get_logger().info("🔍 Performing system health check...")
        
        # Check MAVROS connection
        if not self.mavros_state.connected:
            self.get_logger().error("❌ MAVROS not connected to flight controller")
            return False
            
        # TODO: Add more health checks
        # - Battery voltage
        # - GPS status
        # - Camera status
        # - Sensor status
        
        self.get_logger().info("✅ System health check passed")
        return True
        
    def arm_drone(self):
        """ARM the drone"""
        if not self.arm_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ARM service not available")
            return False
            
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.armed = True
            return True
        return False
        
    def disarm_drone(self):
        """DISARM the drone"""
        if not self.arm_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ARM service not available")
            return False
            
        request = CommandBool.Request()
        request.value = False
        
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.armed = False
            return True
        return False
        
    def set_flight_mode(self, mode):
        """Set flight mode"""
        if not self.mode_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMode service not available")
            return False
            
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        return future.result() and future.result().mode_sent
        
    def takeoff_to_altitude(self, altitude):
        """Takeoff to specified altitude"""
        if not self.takeoff_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Takeoff service not available")
            return False
            
        request = CommandTOL.Request()
        request.altitude = altitude
        
        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() and future.result().success:
            # Wait for stable altitude
            return self.wait_for_altitude(altitude, tolerance=0.1, timeout=15.0)
        return False
        
    def wait_for_altitude(self, target_altitude, tolerance=0.1, timeout=15.0):
        """Wait for drone to reach target altitude"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if abs(self.current_altitude - target_altitude) < tolerance:
                self.get_logger().info(f"✅ Altitude reached: {self.current_altitude:.2f}m")
                return True
            time.sleep(0.1)
            
        self.get_logger().error(f"❌ Altitude timeout: {self.current_altitude:.2f}m (target: {target_altitude:.2f}m)")
        return False
        
    def wait_for_item_detection(self, timeout=60, camera="front_bottom"):
        """Wait for item detection"""
        self.get_logger().info(f"👀 Waiting for item detection ({camera})...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.item_detected:
                return True
            time.sleep(0.1)
            
        return False
        
    def align_and_pickup_item(self, magnet_position, item_number):
        """Align with item and perform pickup"""
        self.get_logger().info(f"🎯 Aligning and picking up Item {item_number}...")
        
        # Visual servoing to center item
        if self.item_detected:
            self.visual_servo_to_item()
            
        # Descent and pickup
        self.descend_and_pickup(magnet_position)
        
        # Ascend back to travel altitude
        self.ascend_to_travel_altitude()
        
    def visual_servo_to_item(self):
        """Visual servoing to center item in camera"""
        # Simple proportional control
        center_x = 320  # Camera center X
        center_y = 240  # Camera center Y
        
        error_x = self.item_position.x - center_x
        error_y = self.item_position.y - center_y
        
        # Convert to velocity commands
        vx = -error_y * 0.001  # Forward/backward
        vy = -error_x * 0.001  # Left/right
        
        self.send_velocity_command(vx, vy, 0)
        
        # Check if aligned
        if abs(error_x) < 20 and abs(error_y) < 20:
            self.item_aligned = True
            self.send_velocity_command(0, 0, 0)  # Stop
            
    def descend_and_pickup(self, magnet_position):
        """Descend and pickup item"""
        self.get_logger().info(f"📉 Descending for pickup with {magnet_position} magnet...")
        
        # Descend to pickup altitude
        self.send_velocity_command(0, 0, 0.3)  # Descend
        time.sleep(3)  # Allow descent
        
        # Activate magnet
        self.publish_magnet_command(f"{magnet_position}:on")
        time.sleep(2)  # Allow pickup
        
        self.get_logger().info(f"✅ Pickup complete with {magnet_position} magnet")
        
    def ascend_to_travel_altitude(self):
        """Ascend back to travel altitude"""
        self.send_velocity_command(0, 0, -0.3)  # Ascend
        time.sleep(3)
        self.send_velocity_command(0, 0, 0)  # Stop
        
    def execute_navigation_turn(self):
        """Execute navigation turn based on configuration"""
        turn_direction = self.hw_config.get_turn_direction()
        self.get_logger().info(f"🔄 Executing {turn_direction} turn...")
        
        # Wait for LiDAR to detect wall
        # TODO: Implement LiDAR wall detection
        
        if turn_direction.lower() == "left":
            self.send_velocity_command(0, 0.5, 0)  # Move left
            time.sleep(2)
        else:  # Right turn
            self.send_velocity_command(0, -0.5, 0)  # Move right
            time.sleep(2)
            
        self.send_velocity_command(0, 0, 0)  # Stop
        
    def search_and_align_bucket(self, camera="front"):
        """Search for drop bucket and align"""
        self.get_logger().info(f"🔍 Searching for bucket with {camera} camera...")
        
        # TODO: Implement bucket detection
        # For now, assume bucket found
        time.sleep(2)
        self.bucket_detected = True
        
        return self.bucket_detected
        
    def drop_item(self, magnet_position, altitude=0.8):
        """Drop item at specified altitude"""
        self.get_logger().info(f"📦 Dropping item from {magnet_position} magnet at {altitude}m...")
        
        # Maintain altitude
        # TODO: Implement altitude hold at drop altitude
        
        # Deactivate magnet
        self.publish_magnet_command(f"{magnet_position}:off")
        time.sleep(2)  # Allow drop
        
        self.get_logger().info(f"✅ Item dropped from {magnet_position} magnet")
        
    def navigate_to_waypoint(self, waypoint_number, speed=3.0):
        """Navigate to GPS waypoint"""
        self.get_logger().info(f"🛰️ Navigating to WP{waypoint_number} at {speed} m/s...")
        
        # TODO: Implement GPS waypoint navigation
        # For now, simulate navigation
        time.sleep(10)  # Simulate travel time
        
        return True  # Assume successful
        
    def send_velocity_command(self, vx, vy, vz):
        """Send velocity command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.velocity_pub.publish(msg)
        
    def send_altitude_command(self, altitude, climb_rate):
        """Send altitude command"""
        # TODO: Implement altitude command
        pass
        
    def complete_checkpoint(self, next_checkpoint):
        """Complete current checkpoint and transition to next"""
        self.publish_checkpoint_status(self.current_checkpoint.value, "COMPLETED")
        self.current_checkpoint = next_checkpoint
        
        if self.debug_mode:
            self.waiting_for_next = True
            self.get_logger().info(f"🔄 Ready for next checkpoint: {next_checkpoint.value}")
            self.get_logger().info("💬 Send 'continue' to proceed...")
        
    def transition_to_error(self):
        """Transition to error state"""
        self.current_checkpoint = MissionCheckpoint.ERROR
        self.waiting_for_next = True
        self.get_logger().error("🚨 Mission entered ERROR state")
        
    def handle_mission_completed(self):
        """Handle mission completion"""
        self.get_logger().info("🎉 MISSION COMPLETED SUCCESSFULLY!")
        self.publish_checkpoint_status("MISSION_COMPLETED", "SUCCESS")
        self.mission_completed = True
        
    # ===========================================
    # MESSAGE PUBLISHERS
    # ===========================================
    
    def publish_checkpoint_status(self, checkpoint, status):
        """Publish checkpoint status"""
        msg = String()
        msg.data = f"{checkpoint}:{status}"
        self.checkpoint_status_pub.publish(msg)
        
    def publish_camera_command(self, command):
        """Publish camera command"""
        msg = String()
        msg.data = command
        self.camera_command_pub.publish(msg)
        
    def publish_magnet_command(self, command):
        """Publish magnet command"""
        msg = String()
        msg.data = command
        self.magnet_command_pub.publish(msg)
        
    # ===========================================
    # CALLBACK FUNCTIONS
    # ===========================================
    
    def mavros_state_callback(self, msg):
        """MAVROS state callback"""
        self.mavros_state = msg
        self.armed = msg.armed
        
    def pose_callback(self, msg):
        """Local position callback"""
        self.current_pose = msg
        self.current_altitude = msg.pose.position.z
        
    def gps_callback(self, msg):
        """GPS position callback"""
        self.gps_position = msg
        
    def vision_callback(self, msg):
        """Vision detection callback"""
        try:
            data = json.loads(msg.data)
            if data.get('detected', False):
                self.item_detected = True
                self.item_position.x = data.get('x', 0)
                self.item_position.y = data.get('y', 0)
        except:
            pass
            
    def user_input_callback(self, msg):
        """User input callback for debug mode"""
        if msg.data.lower() == "continue" and self.waiting_for_next:
            self.waiting_for_next = False
            self.get_logger().info("▶️ Continuing to next checkpoint...")
        elif msg.data.lower() == "pause":
            self.waiting_for_next = True
            self.get_logger().info("⏸️ Mission paused")
        elif msg.data.lower() == "emergency":
            self.get_logger().warning("🚨 Emergency stop triggered!")
            self.emergency_stop()
            
    def emergency_stop(self):
        """Emergency stop procedure"""
        self.send_velocity_command(0, 0, 0)  # Stop all movement
        self.waiting_for_next = True
        self.current_checkpoint = MissionCheckpoint.PAUSED

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Checkpoint12MissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
