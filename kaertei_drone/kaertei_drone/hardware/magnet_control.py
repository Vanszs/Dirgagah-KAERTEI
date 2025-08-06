#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

# from kaertei_drone.srv import MagnetControl  # TODO: Create custom service
from std_srvs.srv import SetBool  # Temporary placeholder
from std_msgs.msg import String, Bool

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False


class MagnetControlNode(Node):
    def __init__(self):
        super().__init__('magnet_control')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('magnet_relay_pins.front', 18),
                ('magnet_relay_pins.back', 19),
                ('activation_duration', 2.0),  # seconds to keep magnet on for pickup
                ('gpio_mode', 'BCM')
            ]
        )
        
        # Get pin configurations
        # sesuaikan ini - pastikan pin tidak konflik dengan sistem lain di Pi 5
        self.front_magnet_pin = self.get_parameter('magnet_relay_pins.front').value
        self.back_magnet_pin = self.get_parameter('magnet_relay_pins.back').value
        
        # Initialize GPIO
        self.gpio_initialized = False
        if RPI_AVAILABLE:
            self.init_gpio()
        else:
            self.get_logger().warn("RPi.GPIO not available - using mock magnet control")
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Service server
        self.magnet_service = self.create_service(
            MagnetControl, '/hardware/magnet_control', self.magnet_control_callback)
        
        # Publishers for status
        self.front_magnet_status_pub = self.create_publisher(Bool, '/hardware/front_magnet_status', qos_profile)
        self.back_magnet_status_pub = self.create_publisher(Bool, '/hardware/back_magnet_status', qos_profile)
        self.magnet_status_pub = self.create_publisher(String, '/hardware/magnet_status', qos_profile)
        
        # Magnet states
        self.front_magnet_active = False
        self.back_magnet_active = False
        
        # Timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Magnet Control Node initialized")
    
    def init_gpio(self):
        """Initialize GPIO pins for relay control"""
        try:
            GPIO.setmode(GPIO.BCM)
            
            # Setup relay pins as outputs
            GPIO.setup(self.front_magnet_pin, GPIO.OUT)
            GPIO.setup(self.back_magnet_pin, GPIO.OUT)
            
            # Initialize relays to OFF (assuming active LOW)
            GPIO.output(self.front_magnet_pin, GPIO.HIGH)
            GPIO.output(self.back_magnet_pin, GPIO.HIGH)
            
            self.gpio_initialized = True
            self.get_logger().info(f"GPIO initialized - Front: Pin {self.front_magnet_pin}, Back: Pin {self.back_magnet_pin}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            self.gpio_initialized = False
    
    def magnet_control_callback(self, request, response):
        """Handle magnet control service requests"""
        try:
            magnet_position = request.magnet_position.lower()
            activate = request.activate
            
            self.get_logger().info(f"Magnet control request: {magnet_position} {'ON' if activate else 'OFF'}")
            
            if magnet_position == "front":
                success = self.control_front_magnet(activate)
            elif magnet_position == "back":
                success = self.control_back_magnet(activate)
            else:
                response.success = False
                response.message = f"Invalid magnet position: {magnet_position}. Use 'front' or 'back'"
                return response
            
            response.success = success
            if success:
                action = "activated" if activate else "deactivated"
                response.message = f"{magnet_position.capitalize()} magnet {action} successfully"
            else:
                response.message = f"Failed to control {magnet_position} magnet"
            
            # Publish status update
            self.publish_magnet_event(magnet_position, activate, success)
            
        except Exception as e:
            self.get_logger().error(f"Error in magnet control service: {e}")
            response.success = False
            response.message = f"Service error: {str(e)}"
        
        return response
    
    def control_front_magnet(self, activate):
        """Control front magnet"""
        try:
            if self.gpio_initialized:
                # Assuming active LOW relay (GPIO.LOW = relay ON, GPIO.HIGH = relay OFF)
                gpio_state = GPIO.LOW if activate else GPIO.HIGH
                GPIO.output(self.front_magnet_pin, gpio_state)
            else:
                self.get_logger().info(f"MOCK: Front magnet {'ON' if activate else 'OFF'}")
            
            self.front_magnet_active = activate
            self.get_logger().info(f"Front magnet {'activated' if activate else 'deactivated'}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error controlling front magnet: {e}")
            return False
    
    def control_back_magnet(self, activate):
        """Control back magnet"""
        try:
            if self.gpio_initialized:
                # Assuming active LOW relay (GPIO.LOW = relay ON, GPIO.HIGH = relay OFF)
                gpio_state = GPIO.LOW if activate else GPIO.HIGH
                GPIO.output(self.back_magnet_pin, gpio_state)
            else:
                self.get_logger().info(f"MOCK: Back magnet {'ON' if activate else 'OFF'}")
            
            self.back_magnet_active = activate
            self.get_logger().info(f"Back magnet {'activated' if activate else 'deactivated'}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error controlling back magnet: {e}")
            return False
    
    def publish_status(self):
        """Publish current magnet status"""
        # Publish individual magnet status
        front_msg = Bool()
        front_msg.data = self.front_magnet_active
        self.front_magnet_status_pub.publish(front_msg)
        
        back_msg = Bool()
        back_msg.data = self.back_magnet_active
        self.back_magnet_status_pub.publish(back_msg)
        
        # Publish combined status
        status_msg = String()
        if self.front_magnet_active and self.back_magnet_active:
            status_msg.data = "BOTH_ACTIVE"
        elif self.front_magnet_active:
            status_msg.data = "FRONT_ACTIVE"
        elif self.back_magnet_active:
            status_msg.data = "BACK_ACTIVE"
        else:
            status_msg.data = "BOTH_INACTIVE"
        
        self.magnet_status_pub.publish(status_msg)
    
    def publish_magnet_event(self, position, activated, success):
        """Publish magnet event for logging"""
        event_msg = String()
        action = "ACTIVATED" if activated else "DEACTIVATED"
        status = "SUCCESS" if success else "FAILED"
        event_msg.data = f"MAGNET_{position.upper()}_{action}_{status}"
        self.magnet_status_pub.publish(event_msg)
    
    def emergency_stop(self):
        """Emergency stop - turn off all magnets"""
        self.get_logger().warn("Emergency stop - deactivating all magnets")
        self.control_front_magnet(False)
        self.control_back_magnet(False)
    
    def destroy_node(self):
        """Clean up GPIO resources"""
        if self.gpio_initialized:
            try:
                # Turn off all relays before cleanup
                GPIO.output(self.front_magnet_pin, GPIO.HIGH)
                GPIO.output(self.back_magnet_pin, GPIO.HIGH)
                
                # Clean up GPIO
                GPIO.cleanup()
                self.get_logger().info("GPIO cleaned up")
                
            except Exception as e:
                self.get_logger().error(f"Error cleaning up GPIO: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        magnet_node = MagnetControlNode()
        rclpy.spin(magnet_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'magnet_node' in locals():
            magnet_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
