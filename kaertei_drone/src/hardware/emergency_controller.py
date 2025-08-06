#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import threading

from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from drone_mvp.hardware_config import HardwareConfig

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class EmergencyControllerNode(Node):
    def __init__(self):
        super().__init__('emergency_controller')
        
        # Load hardware config
        self.hw_config = HardwareConfig()
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 10.0),              # Hz
                ('emergency_timeout', 5.0),         # seconds before auto-recovery
                ('battery_critical_voltage', 3.0),  # per cell
                ('altitude_limit_max', 100.0),      # meters
                ('altitude_limit_min', -5.0),       # meters
                ('velocity_limit_max', 5.0),        # m/s
                ('enable_gpio_monitoring', True),   # Enable physical emergency button
                ('enable_auto_recovery', False),    # Enable automatic recovery
                ('buzzer_pattern_emergency', True), # Emergency buzzer pattern
            ]
        )
        
        # QoS profiles
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers - Emergency status and control
        self.emergency_status_pub = self.create_publisher(Bool, '/emergency/active', self.control_qos)
        self.emergency_reason_pub = self.create_publisher(String, '/emergency/reason', self.control_qos)
        self.emergency_action_pub = self.create_publisher(String, '/emergency/action', self.control_qos)
        self.emergency_override_pub = self.create_publisher(Bool, '/emergency/override_active', self.control_qos)
        
        # Publishers - MAVROS emergency commands
        self.mavros_rc_override_pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', self.control_qos)
        self.emergency_velocity_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self.control_qos)
        
        # Service clients for emergency actions
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_service = self.create_client(SetMode, '/mavros/set_mode')
        
        # Subscribers - System monitoring
        self.flight_status_sub = self.create_subscription(
            String, '/flight_controller/status', self.flight_status_callback, self.sensor_qos)
        self.battery_voltage_sub = self.create_subscription(
            Float32, '/flight_controller/battery_voltage', self.battery_callback, self.sensor_qos)
        self.altitude_sub = self.create_subscription(
            Float32, '/flight_controller/altitude', self.altitude_callback, self.sensor_qos)
        self.system_health_sub = self.create_subscription(
            String, '/flight_controller/health_status', self.system_health_callback, self.sensor_qos)
        
        # Subscribers - External emergency triggers
        self.emergency_trigger_sub = self.create_subscription(
            String, '/emergency/trigger', self.emergency_trigger_callback, self.control_qos)
        self.manual_emergency_sub = self.create_subscription(
            Bool, '/emergency/manual_trigger', self.manual_emergency_callback, self.control_qos)
        
        # Emergency state
        self.emergency_active = False
        self.emergency_reason = []
        self.emergency_start_time = None
        self.last_emergency_action = None
        self.override_active = False
        
        # System state monitoring
        self.current_flight_status = "UNKNOWN"
        self.current_battery_voltage = 0.0
        self.current_altitude = 0.0
        self.current_system_health = "UNKNOWN"
        self.armed = False
        
        # GPIO setup for physical emergency button and buzzer
        self.gpio_initialized = False
        if GPIO_AVAILABLE and self.get_parameter('enable_gpio_monitoring').value:
            self.setup_gpio()
        
        # Emergency button monitoring
        self.button_pressed = False
        self.button_last_state = False
        self.button_press_time = None
        
        # Recovery state
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        
        # Timer for monitoring
        update_rate = self.get_parameter('update_rate').value
        self.monitor_timer = self.create_timer(1.0 / update_rate, self.monitor_callback)
        
        self.get_logger().info("Emergency Controller Node initialized")
        if self.gpio_initialized:
            self.get_logger().info("GPIO emergency button and buzzer initialized")
        else:
            self.get_logger().warn("GPIO not available - using software emergency triggers only")
    
    def setup_gpio(self):
        """Setup GPIO pins for emergency button and buzzer"""
        try:
            if not GPIO_AVAILABLE:
                return
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Get GPIO pins from hardware config
            emergency_pin = self.hw_config.get_emergency_stop_pin()
            error_led_pin = self.hw_config.get_error_led_pin()
            
            # Setup emergency button (pull-up resistor, active LOW)
            GPIO.setup(emergency_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Setup error LED/buzzer (output, active HIGH)
            GPIO.setup(error_led_pin, GPIO.OUT)
            GPIO.output(error_led_pin, GPIO.LOW)
            
            # Add interrupt for emergency button
            GPIO.add_event_detect(emergency_pin, GPIO.FALLING, 
                                callback=self.gpio_emergency_callback, bouncetime=200)
            
            self.emergency_pin = emergency_pin
            self.buzzer_pin = error_led_pin
            self.gpio_initialized = True
            
            self.get_logger().info(f"GPIO initialized - Emergency pin: {emergency_pin}, Buzzer pin: {error_led_pin}")
            
        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            self.gpio_initialized = False
    
    def gpio_emergency_callback(self, channel):
        """Handle GPIO emergency button interrupt"""
        try:
            if not self.button_pressed:
                self.button_pressed = True
                self.button_press_time = time.time()
                self.get_logger().warn("PHYSICAL EMERGENCY BUTTON PRESSED!")
                
                # Trigger emergency
                self.trigger_emergency("PHYSICAL_BUTTON_PRESSED")
                
        except Exception as e:
            self.get_logger().error(f"GPIO emergency callback error: {e}")
    
    def flight_status_callback(self, msg):
        """Handle flight controller status updates"""
        self.current_flight_status = msg.data
        self.armed = "ARMED" in msg.data
        
        # Check for flight controller errors
        if "ERROR" in msg.data:
            self.trigger_emergency("FLIGHT_CONTROLLER_ERROR")
    
    def battery_callback(self, msg):
        """Handle battery voltage updates"""
        self.current_battery_voltage = msg.data
        
        # Check battery critical level
        critical_voltage = self.get_parameter('battery_critical_voltage').value
        # Assume 4S LiPo
        cell_voltage = self.current_battery_voltage / 4.0 if self.current_battery_voltage > 0 else 0.0
        
        if cell_voltage < critical_voltage and cell_voltage > 0:
            self.trigger_emergency("BATTERY_CRITICAL")
    
    def altitude_callback(self, msg):
        """Handle altitude updates"""
        self.current_altitude = msg.data
        
        # Check altitude limits
        max_altitude = self.get_parameter('altitude_limit_max').value
        min_altitude = self.get_parameter('altitude_limit_min').value
        
        if self.current_altitude > max_altitude:
            self.trigger_emergency("ALTITUDE_TOO_HIGH")
        elif self.current_altitude < min_altitude:
            self.trigger_emergency("ALTITUDE_TOO_LOW")
    
    def system_health_callback(self, msg):
        """Handle system health updates"""
        self.current_system_health = msg.data
        
        # Check for system errors
        if "ERROR" in msg.data and "BATTERY_CRITICAL" in msg.data:
            self.trigger_emergency("SYSTEM_HEALTH_ERROR")
    
    def emergency_trigger_callback(self, msg):
        """Handle external emergency triggers"""
        self.trigger_emergency(msg.data)
    
    def manual_emergency_callback(self, msg):
        """Handle manual emergency triggers"""
        if msg.data:
            self.trigger_emergency("MANUAL_TRIGGER")
        else:
            self.clear_emergency("MANUAL_CLEAR")
    
    def trigger_emergency(self, reason):
        """Trigger emergency state with given reason"""
        try:
            if reason not in self.emergency_reason:
                self.emergency_reason.append(reason)
                self.get_logger().error(f"EMERGENCY TRIGGERED: {reason}")
            
            if not self.emergency_active:
                self.emergency_active = True
                self.emergency_start_time = time.time()
                self.get_logger().error("EMERGENCY MODE ACTIVATED!")
                
                # Execute emergency action
                self.execute_emergency_action(reason)
                
                # Activate buzzer
                if self.gpio_initialized:
                    self.start_emergency_buzzer()
            
        except Exception as e:
            self.get_logger().error(f"Error triggering emergency: {e}")
    
    def clear_emergency(self, reason):
        """Clear emergency state"""
        try:
            if self.emergency_active:
                self.emergency_active = False
                self.emergency_reason.clear()
                self.override_active = False
                
                self.get_logger().info(f"Emergency cleared: {reason}")
                
                # Stop buzzer
                if self.gpio_initialized:
                    self.stop_emergency_buzzer()
                
                # Reset recovery attempts
                self.recovery_attempts = 0
                
        except Exception as e:
            self.get_logger().error(f"Error clearing emergency: {e}")
    
    def execute_emergency_action(self, reason):
        """Execute appropriate emergency action based on reason"""
        try:
            action = "UNKNOWN"
            
            if reason == "PHYSICAL_BUTTON_PRESSED":
                action = "IMMEDIATE_LAND"
                self.emergency_land()
                
            elif reason in ["BATTERY_CRITICAL", "SYSTEM_HEALTH_ERROR"]:
                action = "EMERGENCY_LAND"
                self.emergency_land()
                
            elif reason in ["ALTITUDE_TOO_HIGH", "ALTITUDE_TOO_LOW"]:
                action = "ALTITUDE_HOLD"
                self.emergency_altitude_hold()
                
            elif reason == "FLIGHT_CONTROLLER_ERROR":
                action = "RC_OVERRIDE"
                self.emergency_rc_override()
                
            else:
                action = "EMERGENCY_LAND"
                self.emergency_land()
            
            self.last_emergency_action = action
            self.publish_emergency_action(action)
            
        except Exception as e:
            self.get_logger().error(f"Error executing emergency action: {e}")
    
    def emergency_land(self):
        """Execute emergency landing procedure"""
        try:
            # Set to LAND mode
            if self.set_mode_service.wait_for_service(timeout_sec=1.0):
                request = SetMode.Request()
                request.custom_mode = "LAND"
                future = self.set_mode_service.call_async(request)
                self.get_logger().warn("Emergency LAND mode activated")
            
            # Also send direct velocity command for immediate descent
            emergency_velocity = Twist()
            emergency_velocity.linear.x = 0.0
            emergency_velocity.linear.y = 0.0
            emergency_velocity.linear.z = -1.0  # 1 m/s descent
            
            self.emergency_velocity_pub.publish(emergency_velocity)
            
        except Exception as e:
            self.get_logger().error(f"Error in emergency landing: {e}")
    
    def emergency_altitude_hold(self):
        """Execute emergency altitude hold"""
        try:
            # Stop all horizontal movement
            emergency_velocity = Twist()
            emergency_velocity.linear.x = 0.0
            emergency_velocity.linear.y = 0.0
            emergency_velocity.linear.z = 0.0
            
            self.emergency_velocity_pub.publish(emergency_velocity)
            
            # Set to LOITER mode for position hold
            if self.set_mode_service.wait_for_service(timeout_sec=1.0):
                request = SetMode.Request()
                request.custom_mode = "LOITER"
                future = self.set_mode_service.call_async(request)
                self.get_logger().warn("Emergency LOITER mode activated")
            
        except Exception as e:
            self.get_logger().error(f"Error in emergency altitude hold: {e}")
    
    def emergency_rc_override(self):
        """Execute RC override for manual control"""
        try:
            # Override RC channels to neutral/safe positions
            rc_override = OverrideRCIn()
            rc_override.channels = [1500] * 8  # Neutral position for all channels
            
            # Throttle to middle position for altitude hold
            rc_override.channels[2] = 1500  # Throttle channel
            
            self.mavros_rc_override_pub.publish(rc_override)
            self.override_active = True
            
            self.get_logger().warn("RC Override activated - manual control enabled")
            
        except Exception as e:
            self.get_logger().error(f"Error in RC override: {e}")
    
    def start_emergency_buzzer(self):
        """Start emergency buzzer pattern"""
        if self.gpio_initialized and GPIO_AVAILABLE:
            try:
                # Start buzzer in separate thread to avoid blocking
                buzzer_thread = threading.Thread(target=self.buzzer_pattern_thread)
                buzzer_thread.daemon = True
                buzzer_thread.start()
                
            except Exception as e:
                self.get_logger().error(f"Error starting emergency buzzer: {e}")
    
    def buzzer_pattern_thread(self):
        """Emergency buzzer pattern thread"""
        try:
            while self.emergency_active and self.gpio_initialized:
                # Emergency pattern: short beeps
                GPIO.output(self.buzzer_pin, GPIO.HIGH)
                time.sleep(0.2)
                GPIO.output(self.buzzer_pin, GPIO.LOW)
                time.sleep(0.2)
                
        except Exception as e:
            self.get_logger().error(f"Buzzer pattern thread error: {e}")
    
    def stop_emergency_buzzer(self):
        """Stop emergency buzzer"""
        if self.gpio_initialized and GPIO_AVAILABLE:
            try:
                GPIO.output(self.buzzer_pin, GPIO.LOW)
            except Exception as e:
                self.get_logger().error(f"Error stopping buzzer: {e}")
    
    def monitor_callback(self):
        """Main monitoring loop"""
        try:
            current_time = time.time()
            
            # Monitor GPIO button state if available
            if self.gpio_initialized:
                self.monitor_gpio_button()
            
            # Check for auto-recovery if enabled
            if (self.emergency_active and 
                self.get_parameter('enable_auto_recovery').value and
                self.emergency_start_time is not None):
                
                emergency_timeout = self.get_parameter('emergency_timeout').value
                if current_time - self.emergency_start_time > emergency_timeout:
                    self.attempt_recovery()
            
            # Publish emergency status
            self.publish_emergency_status()
            
        except Exception as e:
            self.get_logger().error(f"Error in monitor callback: {e}")
    
    def monitor_gpio_button(self):
        """Monitor physical emergency button state"""
        try:
            if GPIO_AVAILABLE:
                current_state = not GPIO.input(self.emergency_pin)  # Active LOW
                
                if current_state != self.button_last_state:
                    if current_state:  # Button pressed
                        self.button_pressed = True
                        self.button_press_time = time.time()
                    else:  # Button released
                        self.button_pressed = False
                    
                    self.button_last_state = current_state
                
        except Exception as e:
            self.get_logger().error(f"Error monitoring GPIO button: {e}")
    
    def attempt_recovery(self):
        """Attempt automatic recovery from emergency"""
        try:
            if self.recovery_attempts < self.max_recovery_attempts:
                self.recovery_attempts += 1
                
                self.get_logger().warn(f"Attempting recovery {self.recovery_attempts}/{self.max_recovery_attempts}")
                
                # Check if emergency conditions have cleared
                conditions_clear = True
                
                # Check battery
                if self.current_battery_voltage > 0:
                    cell_voltage = self.current_battery_voltage / 4.0
                    critical_voltage = self.get_parameter('battery_critical_voltage').value
                    if cell_voltage < critical_voltage:
                        conditions_clear = False
                
                # Check altitude limits
                max_altitude = self.get_parameter('altitude_limit_max').value
                min_altitude = self.get_parameter('altitude_limit_min').value
                if (self.current_altitude > max_altitude or 
                    self.current_altitude < min_altitude):
                    conditions_clear = False
                
                if conditions_clear:
                    self.clear_emergency("AUTO_RECOVERY")
                    self.get_logger().info("Automatic recovery successful")
                else:
                    # Reset emergency timer for next attempt
                    self.emergency_start_time = time.time()
                    self.get_logger().warn("Recovery conditions not met, continuing emergency mode")
            else:
                self.get_logger().error("Maximum recovery attempts reached - manual intervention required")
                
        except Exception as e:
            self.get_logger().error(f"Error in recovery attempt: {e}")
    
    def publish_emergency_status(self):
        """Publish emergency status information"""
        try:
            # Emergency active status
            self.emergency_status_pub.publish(Bool(data=self.emergency_active))
            
            # Emergency reason
            if self.emergency_reason:
                reason_str = ", ".join(self.emergency_reason)
            else:
                reason_str = "NONE"
            self.emergency_reason_pub.publish(String(data=reason_str))
            
            # Last emergency action
            if self.last_emergency_action:
                self.emergency_action_pub.publish(String(data=self.last_emergency_action))
            
            # Override status
            self.emergency_override_pub.publish(Bool(data=self.override_active))
            
        except Exception as e:
            self.get_logger().error(f"Error publishing emergency status: {e}")
    
    def publish_emergency_action(self, action):
        """Publish emergency action taken"""
        self.emergency_action_pub.publish(String(data=action))
    
    def __del__(self):
        """Cleanup GPIO on node destruction"""
        if self.gpio_initialized and GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        emergency_controller = EmergencyControllerNode()
        rclpy.spin(emergency_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'emergency_controller' in locals():
            emergency_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
