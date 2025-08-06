#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Emergency Handler
Emergency procedures and safety systems for 12-checkpoint mission
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time

class EmergencyHandler(Node):
    def __init__(self):
        super().__init__('emergency_handler')
        
        # Emergency states
        self.emergency_active = False
        self.emergency_reason = ""
        self.battery_critical = False
        self.position_lost = False
        self.system_failure = False
        
        # Safety thresholds
        self.battery_critical_voltage = 14.4  # Volts
        self.position_timeout = 10.0  # Seconds
        self.max_altitude = 50.0  # Meters
        self.geofence_radius = 200.0  # Meters
        
        # System state
        self.current_battery = 0.0
        self.current_altitude = 0.0
        self.last_position_update = time.time()
        self.mavros_connected = False
        self.armed = False
        
        # Publishers
        self.emergency_status_pub = self.create_publisher(String, '/emergency/status', 10)
        self.emergency_action_pub = self.create_publisher(String, '/emergency/action', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.battery_sub = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.mavros_state_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.mission_sub = self.create_subscription(String, '/mission/checkpoint_status', self.mission_callback, 10)
        self.emergency_trigger_sub = self.create_subscription(String, '/emergency/trigger', self.emergency_trigger_callback, 10)
        
        # Services
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        
        # Safety monitoring timer
        self.safety_timer = self.create_timer(1.0, self.safety_monitoring_loop)
        
        self.get_logger().info("🚨 Emergency Handler initialized")
        
    def safety_monitoring_loop(self):
        """Main safety monitoring loop"""
        if self.emergency_active:
            return
            
        # Check various safety conditions
        self.check_battery_safety()
        self.check_position_safety()
        self.check_altitude_safety()
        self.check_system_health()
        
    def check_battery_safety(self):
        """Monitor battery voltage for critical levels"""
        if self.current_battery > 0 and self.current_battery < self.battery_critical_voltage:
            if not self.battery_critical:
                self.battery_critical = True
                self.trigger_emergency("BATTERY_CRITICAL", f"Battery: {self.current_battery:.1f}V")
                
    def check_position_safety(self):
        """Monitor position updates and GPS health"""
        time_since_update = time.time() - self.last_position_update
        
        if time_since_update > self.position_timeout:
            if not self.position_lost:
                self.position_lost = True
                self.trigger_emergency("POSITION_LOST", "GPS/Position timeout")
                
    def check_altitude_safety(self):
        """Monitor altitude limits"""
        if self.current_altitude > self.max_altitude:
            self.trigger_emergency("ALTITUDE_LIMIT", f"Altitude: {self.current_altitude:.1f}m")
            
    def check_system_health(self):
        """Monitor system health"""
        if not self.mavros_connected and self.armed:
            self.trigger_emergency("MAVROS_DISCONNECTED", "Flight controller communication lost")
            
    def trigger_emergency(self, reason, details):
        """Trigger emergency procedures"""
        if self.emergency_active:
            return  # Already in emergency
            
        self.emergency_active = True
        self.emergency_reason = reason
        
        self.get_logger().error(f"🚨 EMERGENCY TRIGGERED: {reason} - {details}")
        
        # Publish emergency status
        self.emergency_status_pub.publish(String(data=f"ACTIVE:{reason}:{details}"))
        
        # Execute emergency procedure based on type
        if reason == "BATTERY_CRITICAL":
            self.execute_emergency_land()
        elif reason == "POSITION_LOST":
            self.execute_emergency_hover()
        elif reason == "ALTITUDE_LIMIT":
            self.execute_emergency_descent()
        elif reason == "MAVROS_DISCONNECTED":
            self.execute_emergency_land()
        elif reason == "MANUAL_TRIGGER":
            self.execute_emergency_land()
        else:
            self.execute_emergency_hover()
            
    def execute_emergency_land(self):
        """Execute emergency landing procedure"""
        self.get_logger().warning("🚨 Executing EMERGENCY LAND")
        self.emergency_action_pub.publish(String(data="EMERGENCY_LAND"))
        
        # Stop all horizontal movement
        stop_msg = Twist()
        self.velocity_pub.publish(stop_msg)
        
        # Try to set LAND mode
        if self.set_flight_mode("LAND"):
            self.get_logger().info("✅ LAND mode activated")
        else:
            # Fallback: manual descent
            self.get_logger().warning("⚠️ LAND mode failed, manual descent")
            self.execute_manual_descent()
            
    def execute_emergency_hover(self):
        """Execute emergency hover procedure"""
        self.get_logger().warning("🚨 Executing EMERGENCY HOVER")
        self.emergency_action_pub.publish(String(data="EMERGENCY_HOVER"))
        
        # Stop all movement
        stop_msg = Twist()
        self.velocity_pub.publish(stop_msg)
        
        # Try to set HOLD mode
        if self.set_flight_mode("HOLD"):
            self.get_logger().info("✅ HOLD mode activated")
        else:
            self.get_logger().warning("⚠️ HOLD mode failed, maintaining position")
            
    def execute_emergency_descent(self):
        """Execute emergency descent to safe altitude"""
        self.get_logger().warning("🚨 Executing EMERGENCY DESCENT")
        self.emergency_action_pub.publish(String(data="EMERGENCY_DESCENT"))
        
        # Slow descent velocity
        descent_msg = Twist()
        descent_msg.linear.z = 0.5  # Descend at 0.5 m/s
        self.velocity_pub.publish(descent_msg)
        
    def execute_manual_descent(self):
        """Manual descent using velocity commands"""
        self.get_logger().warning("🚨 Executing MANUAL DESCENT")
        
        descent_msg = Twist()
        descent_msg.linear.z = 0.3  # Slow descent
        
        # Descend until close to ground
        descent_duration = max(5.0, self.current_altitude / 0.3)  # Estimate descent time
        
        for _ in range(int(descent_duration)):
            if not self.emergency_active:
                break
            self.velocity_pub.publish(descent_msg)
            time.sleep(1.0)
            
        # Stop and disarm
        stop_msg = Twist()
        self.velocity_pub.publish(stop_msg)
        self.disarm_drone()
        
    def execute_return_to_launch(self):
        """Execute return to launch"""
        self.get_logger().warning("🚨 Executing RETURN TO LAUNCH")
        self.emergency_action_pub.publish(String(data="RTL"))
        
        if self.set_flight_mode("RTL"):
            self.get_logger().info("✅ RTL mode activated")
        else:
            self.get_logger().error("❌ RTL mode failed, emergency land")
            self.execute_emergency_land()
            
    def clear_emergency(self):
        """Clear emergency state (manual override)"""
        self.get_logger().info("✅ Emergency cleared")
        self.emergency_active = False
        self.emergency_reason = ""
        self.battery_critical = False
        self.position_lost = False
        
        self.emergency_status_pub.publish(String(data="CLEARED"))
        
    def set_flight_mode(self, mode):
        """Set flight mode via MAVROS"""
        if not self.mode_service.wait_for_service(timeout_sec=2.0):
            return False
            
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        return future.result() and future.result().mode_sent
        
    def disarm_drone(self):
        """Disarm the drone"""
        if not self.arm_service.wait_for_service(timeout_sec=2.0):
            return False
            
        request = CommandBool.Request()
        request.value = False
        
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        result = future.result() and future.result().success
        if result:
            self.get_logger().info("✅ Drone disarmed")
        
        return result
        
    # ===========================================
    # CALLBACK FUNCTIONS
    # ===========================================
    
    def battery_callback(self, msg):
        """Battery status callback"""
        self.current_battery = msg.voltage
        
    def mavros_state_callback(self, msg):
        """MAVROS state callback"""
        self.mavros_connected = msg.connected
        self.armed = msg.armed
        
    def pose_callback(self, msg):
        """Position callback"""
        self.current_altitude = msg.pose.position.z
        self.last_position_update = time.time()
        
    def mission_callback(self, msg):
        """Mission status callback"""
        # Monitor mission for failures that require emergency action
        if "ERROR" in msg.data or "FAILED" in msg.data:
            self.get_logger().warning(f"Mission issue detected: {msg.data}")
            
    def emergency_trigger_callback(self, msg):
        """Manual emergency trigger callback"""
        command = msg.data.upper().strip()
        
        if command == "EMERGENCY_STOP":
            self.trigger_emergency("MANUAL_TRIGGER", "Manual emergency stop")
        elif command == "EMERGENCY_LAND":
            self.trigger_emergency("MANUAL_TRIGGER", "Manual emergency land")
        elif command == "RTL":
            self.execute_return_to_launch()
        elif command == "CLEAR":
            self.clear_emergency()
        elif command == "DISARM":
            self.disarm_drone()
        else:
            self.get_logger().warning(f"Unknown emergency command: {command}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        emergency_handler = EmergencyHandler()
        rclpy.spin(emergency_handler)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
