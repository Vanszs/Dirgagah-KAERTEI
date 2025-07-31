#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix


class FlightModeSwitcherNode(Node):
    def __init__(self):
        super().__init__('flight_mode_switcher')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('takeoff_altitude', 1.5),
                ('indoor_speed', 0.5),
                ('outdoor_speed', 2.0),
                ('hover_altitude', 1.5),
                ('land_speed', 0.5)
            ]
        )
        
        # QoS profile for MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers for MAVROS
        self.setpoint_position_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.setpoint_velocity_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        
        # Status publishers
        self.flight_mode_status_pub = self.create_publisher(String, '/flight/mode_status', qos_profile)
        self.flight_command_status_pub = self.create_publisher(String, '/flight/command_status', qos_profile)
        
        # Subscribers
        self.mission_command_sub = self.create_subscription(
            String, '/mission/command', self.mission_command_callback, qos_profile)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/drone/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.mavros_state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, qos_profile)
        
        # Service clients for MAVROS
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Flight state
        self.current_flight_mode = "UNKNOWN"
        self.is_armed = False
        self.is_guided = False
        self.mavros_connected = False
        self.current_altitude = 0.0
        
        # Command tracking
        self.last_command_time = time.time()
        self.command_sequence = 0
        
        # Setpoint publishing timer for GUIDED mode
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoints)  # 10 Hz
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Current setpoint
        self.current_setpoint = None
        self.current_velocity = None
        
        self.get_logger().info("Flight Mode Switcher Node initialized for ArduPilot with MAVROS")
    
    def mavros_state_callback(self, msg):
        """Handle MAVROS state updates"""
        try:
            self.mavros_connected = msg.connected
            self.is_armed = msg.armed
            self.current_flight_mode = msg.mode
            self.is_guided = (msg.mode == "GUIDED")
            
            if not msg.connected:
                self.get_logger().warn("MAVROS not connected to flight controller")
                
        except Exception as e:
            self.get_logger().error(f"Error processing MAVROS state: {e}")
    
    def mission_command_callback(self, msg):
        """Handle mission commands for flight control"""
        command = msg.data
        self.get_logger().info(f"Received flight command: {command}")
        
        try:
            if command == "ARM_DRONE":
                self.arm_drone()
            elif command == "DISARM_DRONE":
                self.disarm_drone()
            elif command == "TAKEOFF":
                self.takeoff()
            elif command == "SWITCH_TO_AUTO_MISSION":
                self.switch_to_auto_mission()
            elif command == "SWITCH_TO_MANUAL":
                self.switch_to_guided()
            elif command == "HOVER":
                self.hover()
            elif command == "AUTO_LAND":
                self.auto_land()
            elif command == "EMERGENCY_LAND":
                self.emergency_land()
            elif command in ["MOVE_FORWARD_SLOW", "MOVE_FORWARD_GPS_HEADING"]:
                self.move_forward()
            elif command == "TURN_LEFT":
                self.turn_left()
            elif command == "TURN_RIGHT":
                self.turn_right()
            elif command == "ALIGN_TO_TARGET":
                self.align_to_target()
            elif command == "ALIGN_TO_DROPZONE":
                self.align_to_dropzone()
            elif command == "ALIGN_TO_EXIT":
                self.align_to_exit()
            elif command == "EXIT_INDOOR":
                self.exit_indoor()
            else:
                self.get_logger().warn(f"Unknown flight command: {command}")
                
        except Exception as e:
            self.get_logger().error(f"Error executing flight command {command}: {e}")
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands for guided control"""
        if self.is_guided:
            self.current_velocity = msg
    
    def arm_drone(self):
        """Arm the drone using MAVROS"""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.publish_command_status("ARM_COMMAND_SUCCESS")
            self.get_logger().info("Drone armed successfully")
        else:
            self.publish_command_status("ARM_COMMAND_FAILED")
            self.get_logger().error("Failed to arm drone")
    
    def disarm_drone(self):
        """Disarm the drone using MAVROS"""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.publish_command_status("DISARM_COMMAND_SUCCESS")
            self.get_logger().info("Drone disarmed successfully")
        else:
            self.publish_command_status("DISARM_COMMAND_FAILED")
            self.get_logger().error("Failed to disarm drone")
    
    def takeoff(self):
        """Takeoff using GUIDED mode and setpoint"""
        # First switch to GUIDED mode
        self.switch_to_guided()
        time.sleep(1)
        
        # Set takeoff altitude setpoint
        altitude = self.get_parameter('takeoff_altitude').value
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = altitude
        pose.pose.orientation.w = 1.0
        
        self.current_setpoint = pose
        self.publish_command_status("TAKEOFF_COMMAND_SENT")
        self.get_logger().info(f"Takeoff command sent to {altitude}m")
    
    def switch_to_auto_mission(self):
        """Switch to AUTO mode for mission"""
        self.set_flight_mode("AUTO")
        self.is_guided = False
        self.publish_command_status("AUTO_MODE_SET")
        self.get_logger().info("Switched to AUTO mode")
    
    def switch_to_guided(self):
        """Switch to GUIDED mode for manual control"""
        self.set_flight_mode("GUIDED")
        self.is_guided = True
        self.publish_command_status("GUIDED_MODE_SET")
        self.get_logger().info("Switched to GUIDED mode")
    
    def hover(self):
        """Enter hover mode in GUIDED"""
        if not self.is_guided:
            self.switch_to_guided()
        
        # Stop velocity commands
        self.current_velocity = None
        self.publish_command_status("HOVER_MODE_ACTIVATED")
        self.get_logger().info("Hover mode activated")
    
    def auto_land(self):
        """Auto land using LAND mode"""
        self.set_flight_mode("LAND")
        self.is_guided = False
        self.publish_command_status("AUTO_LAND_COMMAND_SENT")
        self.get_logger().info("Auto land command sent")
    
    def emergency_land(self):
        """Emergency land using LAND mode"""
        self.set_flight_mode("LAND")
        self.is_guided = False
        self.publish_command_status("EMERGENCY_LAND_COMMAND_SENT")
        self.get_logger().warn("Emergency land command sent")
    
    def move_forward(self):
        """Move forward in GUIDED mode"""
        if not self.is_guided:
            self.switch_to_guided()
        
        # Create forward velocity command
        twist = Twist()
        twist.linear.x = self.get_parameter('indoor_speed').value
        self.current_velocity = twist
        self.publish_command_status("MOVE_FORWARD_ACTIVATED")
    
    def turn_left(self):
        """Turn left in GUIDED mode"""
        if not self.is_guided:
            self.switch_to_guided()
        
        twist = Twist()
        twist.angular.z = 0.5  # Positive yaw rate for left turn
        self.current_velocity = twist
        self.publish_command_status("TURN_LEFT_ACTIVATED")
    
    def turn_right(self):
        """Turn right in GUIDED mode"""
        if not self.is_guided:
            self.switch_to_guided()
        
        twist = Twist()
        twist.angular.z = -0.5  # Negative yaw rate for right turn
        self.current_velocity = twist
        self.publish_command_status("TURN_RIGHT_ACTIVATED")
    
    def align_to_target(self):
        """Align to detected target (placeholder)"""
        self.get_logger().info("Aligning to target...")
        self.publish_command_status("ALIGNING_TO_TARGET")
        # Alignment logic would be implemented based on vision feedback
    
    def align_to_dropzone(self):
        """Align to dropzone (placeholder)"""
        self.get_logger().info("Aligning to dropzone...")
        self.publish_command_status("ALIGNING_TO_DROPZONE")
    
    def align_to_exit(self):
        """Align to exit gate (placeholder)"""
        self.get_logger().info("Aligning to exit gate...")
        self.publish_command_status("ALIGNING_TO_EXIT")
    
    def exit_indoor(self):
        """Exit indoor area"""
        self.get_logger().info("Exiting indoor area...")
        self.move_forward()
        self.publish_command_status("EXITING_INDOOR")
    
    def set_flight_mode(self, mode):
        """Set flight mode using MAVROS"""
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"Flight mode set to {mode}")
            return True
        else:
            self.get_logger().error(f"Failed to set flight mode to {mode}")
            return False
    
    def publish_setpoints(self):
        """Publish setpoints for GUIDED mode"""
        if not self.is_guided:
            return
        
        # Publish position setpoint if available
        if self.current_setpoint is not None:
            self.current_setpoint.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_position_pub.publish(self.current_setpoint)
        
        # Publish velocity setpoint if available
        if self.current_velocity is not None:
            self.setpoint_velocity_pub.publish(self.current_velocity)
    
    def publish_command_status(self, status):
        """Publish command execution status"""
        msg = String()
        msg.data = status
        self.flight_command_status_pub.publish(msg)
    
    def publish_status(self):
        """Publish current flight status"""
        status_msg = String()
        status_msg.data = f"MODE_{self.current_flight_mode}_ARMED_{self.is_armed}_GUIDED_{self.is_guided}_MAVROS_{self.mavros_connected}"
        self.flight_mode_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        flight_switcher = FlightModeSwitcherNode()
        rclpy.spin(flight_switcher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'flight_switcher' in locals():
            flight_switcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
