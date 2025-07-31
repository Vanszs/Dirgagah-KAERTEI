#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum
import time

from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from drone_mvp.msg import SensorStatus
from drone_mvp.srv import MagnetControl


class MissionState(Enum):
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    INDOOR_SEARCH_ITEM1 = "INDOOR_SEARCH_ITEM1"
    INDOOR_PICKUP_ITEM1 = "INDOOR_PICKUP_ITEM1"
    INDOOR_SEARCH_ITEM2 = "INDOOR_SEARCH_ITEM2"
    INDOOR_PICKUP_ITEM2 = "INDOOR_PICKUP_ITEM2"
    INDOOR_NAVIGATE_TURN = "INDOOR_NAVIGATE_TURN"
    INDOOR_SEARCH_DROPZONE = "INDOOR_SEARCH_DROPZONE"
    INDOOR_DROP_ITEMS = "INDOOR_DROP_ITEMS"
    INDOOR_FIND_EXIT = "INDOOR_FIND_EXIT"
    OUTDOOR_TRANSITION = "OUTDOOR_TRANSITION"
    OUTDOOR_GPS_MISSION = "OUTDOOR_GPS_MISSION"
    OUTDOOR_MANUAL_FALLBACK = "OUTDOOR_MANUAL_FALLBACK"
    OUTDOOR_PICKUP = "OUTDOOR_PICKUP"
    OUTDOOR_RETURN = "OUTDOOR_RETURN"
    OUTDOOR_HOVER_SEARCH = "OUTDOOR_HOVER_SEARCH"
    OUTDOOR_DROP = "OUTDOOR_DROP"
    LANDING = "LANDING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('takeoff_altitude', 1.5),
                ('indoor_speed', 0.5),
                ('outdoor_speed', 2.0),
                ('hover_distance_threshold', 97.0),
                ('gps_stuck_timeout', 10.0),
                ('detection_timeout', 5.0)
            ]
        )
        
        # Mission state
        self.current_state = MissionState.INIT
        self.previous_state = None
        self.state_start_time = time.time()
        
        # Mission flags
        self.item1_collected = False
        self.item2_collected = False
        self.items_dropped = False
        self.exit_found = False
        self.outdoor_item_collected = False
        self.is_indoor_phase = True
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/mission/state', qos_profile)
        self.command_pub = self.create_publisher(String, '/mission/command', qos_profile)
        self.camera_enable_pub = self.create_publisher(String, '/vision/camera_enable', qos_profile)
        
        # Subscribers
        self.sensor_sub = self.create_subscription(
            SensorStatus, '/sensors/status', self.sensor_callback, qos_profile)
        self.vision_sub = self.create_subscription(
            String, '/vision/detection', self.vision_callback, qos_profile)
        self.gps_status_sub = self.create_subscription(
            Bool, '/gps/moving_status', self.gps_status_callback, qos_profile)
        self.waypoint_distance_sub = self.create_subscription(
            Float32, '/waypoint/distance', self.waypoint_distance_callback, qos_profile)
        self.exit_detection_sub = self.create_subscription(
            Bool, '/vision/exit_detected', self.exit_detection_callback, qos_profile)
        
        # Service clients
        self.magnet_client = self.create_client(MagnetControl, '/hardware/magnet_control')
        self.flight_mode_client = self.create_client(String, '/flight/mode_change')
        
        # Mission variables
        self.sensor_data = None
        self.last_detection = None
        self.gps_moving = True
        self.waypoint_distance = float('inf')
        self.detection_start_time = None
        
        # Timer for state machine
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.get_logger().info("Mission Node initialized - Ready for KAERTEI 2025 FAIO!")
    
    def sensor_callback(self, msg):
        """Handle sensor data updates"""
        self.sensor_data = msg
        
        # Auto-interrupt for indoor navigation if not centered
        if self.is_indoor_phase and not msg.is_centered:
            if self.current_state in [MissionState.INDOOR_SEARCH_ITEM1, 
                                    MissionState.INDOOR_SEARCH_ITEM2,
                                    MissionState.INDOOR_NAVIGATE_TURN]:
                self.get_logger().warn("Auto-correction triggered - drone not centered")
                self.publish_command("CALIBRATE_CENTER")
    
    def vision_callback(self, msg):
        """Handle vision detection results"""
        self.last_detection = msg.data
        if msg.data != "NONE":
            self.detection_start_time = time.time()
            self.get_logger().info(f"Vision detected: {msg.data}")
    
    def gps_status_callback(self, msg):
        """Handle GPS movement status"""
        self.gps_moving = msg.data
    
    def waypoint_distance_callback(self, msg):
        """Handle waypoint distance updates"""
        self.waypoint_distance = msg.data
    
    def exit_detection_callback(self, msg):
        """Handle exit gate detection"""
        if msg.data:
            self.exit_found = True
            self.get_logger().info("Exit gate detected!")
    
    def publish_state(self):
        """Publish current mission state"""
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)
    
    def publish_command(self, command):
        """Publish mission command"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Command sent: {command}")
    
    def enable_camera(self, camera_type):
        """Enable specific camera for vision detection"""
        msg = String()
        msg.data = camera_type
        self.camera_enable_pub.publish(msg)
        self.get_logger().info(f"Camera enabled: {camera_type}")
    
    def change_state(self, new_state):
        """Change mission state with logging"""
        if self.current_state != new_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
            self.get_logger().info(f"State changed: {self.previous_state.value} -> {new_state.value}")
            self.publish_state()
    
    def get_state_duration(self):
        """Get time spent in current state"""
        return time.time() - self.state_start_time
    
    def call_magnet_service(self, position, activate):
        """Call magnet control service"""
        if not self.magnet_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Magnet service not available")
            return False
        
        request = MagnetControl.Request()
        request.magnet_position = position
        request.activate = activate
        
        future = self.magnet_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"Magnet {position} {'ON' if activate else 'OFF'}: {result.message}")
            return result.success
        else:
            self.get_logger().error("Magnet service call failed")
            return False
    
    def state_machine_callback(self):
        """Main state machine logic"""
        try:
            if self.current_state == MissionState.INIT:
                self.handle_init_state()
            elif self.current_state == MissionState.TAKEOFF:
                self.handle_takeoff_state()
            elif self.current_state == MissionState.INDOOR_SEARCH_ITEM1:
                self.handle_indoor_search_item1_state()
            elif self.current_state == MissionState.INDOOR_PICKUP_ITEM1:
                self.handle_indoor_pickup_item1_state()
            elif self.current_state == MissionState.INDOOR_SEARCH_ITEM2:
                self.handle_indoor_search_item2_state()
            elif self.current_state == MissionState.INDOOR_PICKUP_ITEM2:
                self.handle_indoor_pickup_item2_state()
            elif self.current_state == MissionState.INDOOR_NAVIGATE_TURN:
                self.handle_indoor_navigate_turn_state()
            elif self.current_state == MissionState.INDOOR_SEARCH_DROPZONE:
                self.handle_indoor_search_dropzone_state()
            elif self.current_state == MissionState.INDOOR_DROP_ITEMS:
                self.handle_indoor_drop_items_state()
            elif self.current_state == MissionState.INDOOR_FIND_EXIT:
                self.handle_indoor_find_exit_state()
            elif self.current_state == MissionState.OUTDOOR_TRANSITION:
                self.handle_outdoor_transition_state()
            elif self.current_state == MissionState.OUTDOOR_GPS_MISSION:
                self.handle_outdoor_gps_mission_state()
            elif self.current_state == MissionState.OUTDOOR_MANUAL_FALLBACK:
                self.handle_outdoor_manual_fallback_state()
            elif self.current_state == MissionState.OUTDOOR_PICKUP:
                self.handle_outdoor_pickup_state()
            elif self.current_state == MissionState.OUTDOOR_RETURN:
                self.handle_outdoor_return_state()
            elif self.current_state == MissionState.OUTDOOR_HOVER_SEARCH:
                self.handle_outdoor_hover_search_state()
            elif self.current_state == MissionState.OUTDOOR_DROP:
                self.handle_outdoor_drop_state()
            elif self.current_state == MissionState.LANDING:
                self.handle_landing_state()
            elif self.current_state == MissionState.COMPLETED:
                self.handle_completed_state()
            elif self.current_state == MissionState.ERROR:
                self.handle_error_state()
                
        except Exception as e:
            self.get_logger().error(f"State machine error: {str(e)}")
            self.change_state(MissionState.ERROR)
    
    def handle_init_state(self):
        """Initialize mission"""
        self.get_logger().info("Initializing mission...")
        self.publish_command("ARM_DRONE")
        time.sleep(2)
        self.change_state(MissionState.TAKEOFF)
    
    def handle_takeoff_state(self):
        """Handle takeoff to indoor altitude"""
        if self.get_state_duration() < 2:
            self.publish_command("TAKEOFF")
        elif self.get_state_duration() > 5:  # Assume takeoff completed
            self.enable_camera("front")
            self.change_state(MissionState.INDOOR_SEARCH_ITEM1)
    
    def handle_indoor_search_item1_state(self):
        """Search for first item with front camera"""
        self.publish_command("MOVE_FORWARD_SLOW")
        
        if self.last_detection and "ITEM" in self.last_detection:
            self.change_state(MissionState.INDOOR_PICKUP_ITEM1)
        elif self.get_state_duration() > self.get_parameter('detection_timeout').value:
            self.get_logger().warn("Item 1 search timeout - continuing mission")
            self.change_state(MissionState.INDOOR_SEARCH_ITEM2)
    
    def handle_indoor_pickup_item1_state(self):
        """Pick up first item"""
        if not self.item1_collected:
            self.publish_command("ALIGN_TO_TARGET")
            time.sleep(2)
            if self.call_magnet_service("front", True):
                self.item1_collected = True
                self.get_logger().info("Item 1 collected!")
                self.enable_camera("back")
                self.change_state(MissionState.INDOOR_SEARCH_ITEM2)
    
    def handle_indoor_search_item2_state(self):
        """Search for second item with back camera"""
        self.publish_command("MOVE_FORWARD_SLOW")
        
        if self.last_detection and "ITEM" in self.last_detection:
            self.change_state(MissionState.INDOOR_PICKUP_ITEM2)
        elif self.get_state_duration() > self.get_parameter('detection_timeout').value:
            self.get_logger().warn("Item 2 search timeout - continuing to turn")
            self.change_state(MissionState.INDOOR_NAVIGATE_TURN)
    
    def handle_indoor_pickup_item2_state(self):
        """Pick up second item"""
        if not self.item2_collected:
            self.publish_command("ALIGN_TO_TARGET")
            time.sleep(2)
            if self.call_magnet_service("back", True):
                self.item2_collected = True
                self.get_logger().info("Item 2 collected!")
                self.change_state(MissionState.INDOOR_NAVIGATE_TURN)
    
    def handle_indoor_navigate_turn_state(self):
        """Navigate to turn point and turn"""
        if self.sensor_data:
            # Detect turn based on sensor loss (open space detected)
            if (self.sensor_data.distance_left > 3.0 or 
                self.sensor_data.distance_right > 3.0):
                if self.sensor_data.distance_left > self.sensor_data.distance_right:
                    self.publish_command("TURN_LEFT")
                else:
                    self.publish_command("TURN_RIGHT")
                self.enable_camera("front")
                self.change_state(MissionState.INDOOR_SEARCH_DROPZONE)
            else:
                self.publish_command("MOVE_FORWARD_SLOW")
    
    def handle_indoor_search_dropzone_state(self):
        """Search for dropzone baskets"""
        self.publish_command("MOVE_FORWARD_SLOW")
        
        if self.last_detection and "DROPZONE" in self.last_detection:
            self.change_state(MissionState.INDOOR_DROP_ITEMS)
        elif self.get_state_duration() > self.get_parameter('detection_timeout').value * 2:
            self.get_logger().warn("Dropzone search timeout - continuing to exit")
            self.change_state(MissionState.INDOOR_FIND_EXIT)
    
    def handle_indoor_drop_items_state(self):
        """Drop collected items in dropzone"""
        if not self.items_dropped:
            self.publish_command("ALIGN_TO_DROPZONE")
            time.sleep(2)
            
            # Drop items
            if self.item1_collected:
                self.call_magnet_service("front", False)
            if self.item2_collected:
                self.call_magnet_service("back", False)
            
            self.items_dropped = True
            self.get_logger().info("Items dropped in indoor dropzone!")
            self.enable_camera("top")
            self.change_state(MissionState.INDOOR_FIND_EXIT)
    
    def handle_indoor_find_exit_state(self):
        """Find exit gate with top camera"""
        self.publish_command("MOVE_FORWARD_SLOW")
        
        if self.exit_found:
            self.publish_command("ALIGN_TO_EXIT")
            time.sleep(2)
            self.is_indoor_phase = False
            self.change_state(MissionState.OUTDOOR_TRANSITION)
        elif self.get_state_duration() > self.get_parameter('detection_timeout').value * 3:
            self.get_logger().error("Exit not found - manual intervention required")
            self.change_state(MissionState.ERROR)
    
    def handle_outdoor_transition_state(self):
        """Transition to outdoor phase"""
        self.publish_command("EXIT_INDOOR")
        time.sleep(3)
        self.publish_command("SWITCH_TO_AUTO_MISSION")
        self.change_state(MissionState.OUTDOOR_GPS_MISSION)
    
    def handle_outdoor_gps_mission_state(self):
        """GPS mission to outdoor waypoint"""
        if not self.gps_moving and self.get_state_duration() > self.get_parameter('gps_stuck_timeout').value:
            self.get_logger().warn("GPS stuck - switching to manual fallback")
            self.change_state(MissionState.OUTDOOR_MANUAL_FALLBACK)
        elif self.waypoint_distance < 5.0:  # Reached pickup waypoint
            self.enable_camera("front")
            self.change_state(MissionState.OUTDOOR_PICKUP)
    
    def handle_outdoor_manual_fallback_state(self):
        """Manual fallback when GPS fails"""
        self.publish_command("SWITCH_TO_MANUAL")
        self.publish_command("MOVE_FORWARD_GPS_HEADING")
        
        if self.last_detection and "ITEM" in self.last_detection:
            self.change_state(MissionState.OUTDOOR_PICKUP)
        elif self.get_state_duration() > 30:  # Timeout for manual navigation
            self.get_logger().error("Manual fallback timeout")
            self.change_state(MissionState.ERROR)
    
    def handle_outdoor_pickup_state(self):
        """Pick up outdoor item"""
        if not self.outdoor_item_collected:
            self.publish_command("ALIGN_TO_TARGET")
            time.sleep(2)
            if self.call_magnet_service("front", True):
                self.outdoor_item_collected = True
                self.get_logger().info("Outdoor item collected!")
                self.publish_command("SWITCH_TO_AUTO_MISSION")
                self.change_state(MissionState.OUTDOOR_RETURN)
    
    def handle_outdoor_return_state(self):
        """Return to outdoor drop waypoint"""
        if self.waypoint_distance <= self.get_parameter('hover_distance_threshold').value:
            self.publish_command("HOVER")
            self.enable_camera("front")
            self.change_state(MissionState.OUTDOOR_HOVER_SEARCH)
    
    def handle_outdoor_hover_search_state(self):
        """Hover and search for outdoor dropzone"""
        if self.last_detection and "DROPZONE" in self.last_detection:
            self.change_state(MissionState.OUTDOOR_DROP)
        elif self.get_state_duration() > self.get_parameter('detection_timeout').value * 2:
            self.get_logger().warn("Outdoor dropzone not found - landing anyway")
            self.change_state(MissionState.LANDING)
    
    def handle_outdoor_drop_state(self):
        """Drop outdoor item"""
        self.publish_command("ALIGN_TO_DROPZONE")
        time.sleep(2)
        
        if self.outdoor_item_collected:
            self.call_magnet_service("front", False)
            self.get_logger().info("Outdoor item dropped!")
        
        self.publish_command("SWITCH_TO_AUTO_MISSION")
        self.change_state(MissionState.LANDING)
    
    def handle_landing_state(self):
        """Auto land at designated zone"""
        self.publish_command("AUTO_LAND")
        if self.get_state_duration() > 10:  # Assume landing completed
            self.change_state(MissionState.COMPLETED)
    
    def handle_completed_state(self):
        """Mission completed successfully"""
        if self.get_state_duration() < 5:
            self.get_logger().info("Mission completed successfully!")
            self.publish_command("DISARM_DRONE")
    
    def handle_error_state(self):
        """Handle error state"""
        self.get_logger().error("Mission in error state - manual intervention required")
        self.publish_command("EMERGENCY_LAND")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        mission_node = MissionNode()
        rclpy.spin(mission_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'mission_node' in locals():
            mission_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
