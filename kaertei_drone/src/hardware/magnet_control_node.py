#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

# Import GPIO with error handling
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("❌ RPi.GPIO not available. This node requires Raspberry Pi.")
    GPIO = None

# Import hardware configuration  
from .hardware_config import HardwareConfig

class MagnetControlNode(Node):
    def __init__(self):
        super().__init__('magnet_control_node')
        
        # Load hardware configuration
        self.hw_config = HardwareConfig()
        
        # Check if GPIO is available
        if GPIO is None:
            self.get_logger().error("❌ GPIO not available - not running on Raspberry Pi?")
            return
        
        # GPIO pins from configuration
        self.FRONT_MAGNET_PIN = self.hw_config.get_front_magnet_pin()
        self.BACK_MAGNET_PIN = self.hw_config.get_back_magnet_pin()
        self.relay_active_high = self.hw_config.is_relay_active_high()
        
        # Hardware delays from configuration
        self.delays = self.hw_config.get_hardware_delays()
        
        # Magnet states
        self.front_magnet_active = False
        self.back_magnet_active = False
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Publishers
        self.magnet_status_pub = self.create_publisher(Bool, '/magnet/status', 10)
        
        # Subscribers
        self.magnet_command_sub = self.create_subscription(
            String, '/magnet/command', self.magnet_command_callback, 10)
        
        self.get_logger().info("🧲 Magnet Control Node Started")
    
    def setup_gpio(self):
        """Setup GPIO pins for magnet control"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.FRONT_MAGNET_PIN, GPIO.OUT)
            GPIO.setup(self.BACK_MAGNET_PIN, GPIO.OUT)
            
            # Initialize with magnets off
            inactive_state = GPIO.LOW if self.relay_active_high else GPIO.HIGH
            GPIO.output(self.FRONT_MAGNET_PIN, inactive_state)
            GPIO.output(self.BACK_MAGNET_PIN, inactive_state)
            
            self.get_logger().info(f"✅ GPIO initialized:")
            self.get_logger().info(f"  Front magnet: GPIO {self.FRONT_MAGNET_PIN}")
            self.get_logger().info(f"  Back magnet: GPIO {self.BACK_MAGNET_PIN}")
            self.get_logger().info(f"  Relay logic: {'Active HIGH' if self.relay_active_high else 'Active LOW'}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize GPIO: {e}")
            self.get_logger().error("Check:")
            self.get_logger().error("  1. Running on Raspberry Pi")
            self.get_logger().error("  2. User in gpio group")
            self.get_logger().error("  3. GPIO pins not in use")
    
    def magnet_command_callback(self, msg):
        """Handle magnet control commands"""
        command = msg.data.lower()
        
        if command == "front:on":
            self.activate_front_magnet()
        elif command == "front:off":
            self.deactivate_front_magnet()
        elif command == "back:on":
            self.activate_back_magnet()
        elif command == "back:off":
            self.deactivate_back_magnet()
        elif command == "all:off":
            self.deactivate_all_magnets()
        else:
            self.get_logger().warning(f"Unknown magnet command: {command}")
    
    def activate_front_magnet(self):
        """Activate front electromagnet"""
        try:
            active_state = GPIO.HIGH if self.relay_active_high else GPIO.LOW
            GPIO.output(self.FRONT_MAGNET_PIN, active_state)
            self.front_magnet_active = True
            self.get_logger().info("🧲 Front magnet activated")
            
            # Wait for magnet activation delay
            time.sleep(self.delays['magnet_on'])
            
            # Publish status
            status = Bool()
            status.data = True
            self.magnet_status_pub.publish(status)
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to activate front magnet: {e}")
    
    def deactivate_front_magnet(self):
        """Deactivate front electromagnet"""
        try:
            inactive_state = GPIO.LOW if self.relay_active_high else GPIO.HIGH
            GPIO.output(self.FRONT_MAGNET_PIN, inactive_state)
            self.front_magnet_active = False
            self.get_logger().info("🔄 Front magnet deactivated")
            
            # Wait for magnet deactivation delay
            time.sleep(self.delays['magnet_off'])
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to deactivate front magnet: {e}")
    
    def activate_back_magnet(self):
        """Activate back electromagnet"""
        try:
            active_state = GPIO.HIGH if self.relay_active_high else GPIO.LOW
            GPIO.output(self.BACK_MAGNET_PIN, active_state)
            self.back_magnet_active = True
            self.get_logger().info("🧲 Back magnet activated")
            
            time.sleep(self.delays['magnet_on'])
            
            # Publish status
            status = Bool()
            status.data = True
            self.magnet_status_pub.publish(status)
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to activate back magnet: {e}")
    
    def deactivate_back_magnet(self):
        """Deactivate back electromagnet"""
        try:
            inactive_state = GPIO.LOW if self.relay_active_high else GPIO.HIGH
            GPIO.output(self.BACK_MAGNET_PIN, inactive_state)
            self.back_magnet_active = False
            self.get_logger().info("🔄 Back magnet deactivated")
            
            time.sleep(self.delays['magnet_off'])
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to deactivate back magnet: {e}")
    
    def deactivate_all_magnets(self):
        """Deactivate all electromagnets"""
        self.deactivate_front_magnet()
        self.deactivate_back_magnet()
        self.get_logger().info("🔄 All magnets deactivated")
    
    def __del__(self):
        """Cleanup GPIO on node destruction"""
        try:
            self.deactivate_all_magnets()
            GPIO.cleanup()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MagnetControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        try:
            GPIO.cleanup()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
