#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

# Import GPIO with error handling (Raspberry Pi 5)
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    print("⚠️  RPi.GPIO not available. Running in simulation mode.")
    HAS_GPIO = False
    # Create mock GPIO for development/testing
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        HIGH = True
        LOW = False
        
        @staticmethod
        def setmode(mode): pass
        @staticmethod
        def setup(pin, mode): pass
        @staticmethod
        def output(pin, state): pass
        @staticmethod
        def cleanup(): pass
    
    GPIO = MockGPIO()

# Import hardware configuration
from .hardware_config import HardwareConfig

class GpioControlNode(Node):
    """
    GPIO Control Node for Raspberry Pi 5
    Handles 2x relay control for electromagnets
    
    Features:
    - Front and back electromagnet relay control
    - Safety interlocks and timeout protection
    - Status monitoring and diagnostics
    - Emergency stop functionality
    """

    def __init__(self):
        super().__init__('gpio_control_node')
        self.hw_config = HardwareConfig()
        
        # GPIO state tracking
        self.magnet_states = {
            'front': False,
            'back': False
        }
        
        # Safety timers
        self.magnet_timers = {
            'front': None,
            'back': None
        }
        
        self.max_magnet_time = 30.0  # Maximum magnet on time (seconds)
        
        # Initialize GPIO
        self.initialize_gpio()
        
        # ROS Publishers
        self.status_pub = self.create_publisher(String, '/hardware/gpio/status', 10)
        self.magnet_feedback_pub = self.create_publisher(Bool, '/hardware/magnets/status', 10)
        
        # ROS Subscribers
        self.magnet_command_sub = self.create_subscription(
            String, '/hardware/magnets/command', self.magnet_command_callback, 10)
        self.gpio_command_sub = self.create_subscription(
            String, '/hardware/gpio/command', self.gpio_command_callback, 10)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("🔌 GPIO Control Node initialized")
        if HAS_GPIO:
            self.get_logger().info("✅ Raspberry Pi GPIO available")
        else:
            self.get_logger().warn("⚠️  Running in GPIO simulation mode")
            
        # Log relay configuration
        relay_pins = self.hw_config.get_relay_pins()
        self.get_logger().info(f"📍 Front Magnet Relay: GPIO {relay_pins['front_magnet']}")
        self.get_logger().info(f"📍 Back Magnet Relay: GPIO {relay_pins['back_magnet']}")

    def initialize_gpio(self):
        """Initialize GPIO pins for relay control"""
        if not HAS_GPIO:
            self.get_logger().warn("⚠️  GPIO simulation mode - no actual GPIO control")
            return
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup relay pins
            relay_pins = self.hw_config.get_relay_pins()
            for magnet, pin in relay_pins.items():
                GPIO.setup(pin, GPIO.OUT)
                # Start with relays OFF (active LOW by default)
                initial_state = GPIO.HIGH if not self.hw_config.is_relay_active_high() else GPIO.LOW
                GPIO.output(pin, initial_state)
                self.get_logger().info(f"✅ {magnet.title()} relay GPIO {pin} initialized (OFF)")
            
            # Setup status LED pins
            status_pin = self.hw_config.get_status_led_pin()
            error_pin = self.hw_config.get_error_led_pin()
            
            GPIO.setup(status_pin, GPIO.OUT)
            GPIO.setup(error_pin, GPIO.OUT)
            
            # Status LED ON, Error LED OFF
            GPIO.output(status_pin, GPIO.HIGH)
            GPIO.output(error_pin, GPIO.LOW)
            
            self.get_logger().info("✅ GPIO initialization complete")
            
        except Exception as e:
            self.get_logger().error(f"❌ GPIO initialization failed: {e}")

    def magnet_command_callback(self, msg):
        """Handle magnet control commands"""
        try:
            command = msg.data.upper()
            
            if ":" in command:
                # Format: "FRONT:ON" or "BACK:OFF"
                magnet, action = command.split(":", 1)
                magnet = magnet.lower()
                action = action.upper()
                
                if magnet in ['front', 'back']:
                    if action == "ON":
                        self.activate_magnet(magnet)
                    elif action == "OFF":
                        self.deactivate_magnet(magnet)
                    else:
                        self.get_logger().warn(f"❌ Unknown magnet action: {action}")
                else:
                    self.get_logger().warn(f"❌ Unknown magnet: {magnet}")
                    
            elif command == "ALL_OFF":
                self.deactivate_all_magnets()
                
            elif command == "STATUS":
                self.log_magnet_status()
                
            elif command == "EMERGENCY_STOP":
                self.emergency_stop()
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing magnet command: {e}")

    def gpio_command_callback(self, msg):
        """Handle general GPIO commands"""
        command = msg.data.upper()
        
        if command == "STATUS":
            self.log_gpio_status()
        elif command == "RESET":
            self.reset_gpio()
        elif command == "TEST":
            self.test_relays()

    def activate_magnet(self, magnet):
        """Activate electromagnet with safety checks"""
        if magnet not in ['front', 'back']:
            self.get_logger().warn(f"❌ Invalid magnet: {magnet}")
            return
        
        if self.magnet_states[magnet]:
            self.get_logger().info(f"⚠️  {magnet.title()} magnet already active")
            return
        
        try:
            if HAS_GPIO:
                relay_pins = self.hw_config.get_relay_pins()
                pin = relay_pins[f'{magnet}_magnet']
                
                # Activate relay (logic depends on active HIGH/LOW)
                active_state = GPIO.HIGH if self.hw_config.is_relay_active_high() else GPIO.LOW
                GPIO.output(pin, active_state)
            
            self.magnet_states[magnet] = True
            
            # Start safety timer
            self.start_magnet_timer(magnet)
            
            self.get_logger().info(f"🧲 {magnet.title()} magnet ACTIVATED")
            
            # Publish feedback
            self.publish_magnet_feedback()
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to activate {magnet} magnet: {e}")

    def deactivate_magnet(self, magnet):
        """Deactivate electromagnet"""
        if magnet not in ['front', 'back']:
            self.get_logger().warn(f"❌ Invalid magnet: {magnet}")
            return
        
        if not self.magnet_states[magnet]:
            self.get_logger().info(f"⚠️  {magnet.title()} magnet already inactive")
            return
        
        try:
            if HAS_GPIO:
                relay_pins = self.hw_config.get_relay_pins()
                pin = relay_pins[f'{magnet}_magnet']
                
                # Deactivate relay (opposite of active state)
                inactive_state = GPIO.LOW if self.hw_config.is_relay_active_high() else GPIO.HIGH
                GPIO.output(pin, inactive_state)
            
            self.magnet_states[magnet] = False
            
            # Cancel safety timer
            self.cancel_magnet_timer(magnet)
            
            self.get_logger().info(f"🧲 {magnet.title()} magnet DEACTIVATED")
            
            # Publish feedback
            self.publish_magnet_feedback()
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to deactivate {magnet} magnet: {e}")

    def deactivate_all_magnets(self):
        """Emergency deactivate all electromagnets"""
        self.get_logger().warn("🚨 EMERGENCY: Deactivating ALL magnets!")
        
        for magnet in ['front', 'back']:
            self.deactivate_magnet(magnet)

    def start_magnet_timer(self, magnet):
        """Start safety timer for magnet"""
        def timer_callback():
            self.get_logger().warn(f"⏰ Safety timeout! Deactivating {magnet} magnet")
            self.deactivate_magnet(magnet)
        
        self.magnet_timers[magnet] = self.create_timer(
            self.max_magnet_time, 
            timer_callback
        )

    def cancel_magnet_timer(self, magnet):
        """Cancel safety timer for magnet"""
        if self.magnet_timers[magnet]:
            self.magnet_timers[magnet].cancel()
            self.magnet_timers[magnet] = None

    def emergency_stop(self):
        """Emergency stop - deactivate everything"""
        self.get_logger().error("🚨 EMERGENCY STOP ACTIVATED!")
        
        # Deactivate all magnets
        self.deactivate_all_magnets()
        
        # Turn on error LED
        if HAS_GPIO:
            try:
                error_pin = self.hw_config.get_error_led_pin()
                GPIO.output(error_pin, GPIO.HIGH)
            except:
                pass

    def test_relays(self):
        """Test relay functionality"""
        self.get_logger().info("🔧 Testing relay functionality...")
        
        for magnet in ['front', 'back']:
            self.get_logger().info(f"Testing {magnet} magnet...")
            self.activate_magnet(magnet)
            time.sleep(2.0)
            self.deactivate_magnet(magnet)
            time.sleep(1.0)
        
        self.get_logger().info("✅ Relay test complete")

    def publish_status(self):
        """Publish GPIO status"""
        status_msg = String()
        front_state = "ON" if self.magnet_states['front'] else "OFF"
        back_state = "ON" if self.magnet_states['back'] else "OFF"
        
        status_msg.data = f"GPIO_STATUS:FRONT_MAGNET:{front_state},BACK_MAGNET:{back_state}"
        self.status_pub.publish(status_msg)

    def publish_magnet_feedback(self):
        """Publish magnet status feedback"""
        any_active = any(self.magnet_states.values())
        feedback_msg = Bool()
        feedback_msg.data = any_active
        self.magnet_feedback_pub.publish(feedback_msg)

    def log_magnet_status(self):
        """Log current magnet status"""
        self.get_logger().info("🔍 Magnet Status:")
        for magnet, state in self.magnet_states.items():
            status = "ACTIVE" if state else "INACTIVE"
            self.get_logger().info(f"  {magnet.title()}: {status}")

    def log_gpio_status(self):
        """Log comprehensive GPIO status"""
        self.get_logger().info("🔍 GPIO Status:")
        
        # Magnet states
        for magnet, state in self.magnet_states.items():
            relay_pins = self.hw_config.get_relay_pins()
            pin = relay_pins[f'{magnet}_magnet']
            status = "ACTIVE" if state else "INACTIVE"
            self.get_logger().info(f"  {magnet.title()} Magnet (GPIO {pin}): {status}")
        
        # GPIO mode
        if HAS_GPIO:
            self.get_logger().info("  GPIO Mode: Hardware (Raspberry Pi)")
        else:
            self.get_logger().info("  GPIO Mode: Simulation")

    def reset_gpio(self):
        """Reset all GPIO to safe state"""
        self.get_logger().info("🔄 Resetting GPIO to safe state...")
        self.deactivate_all_magnets()
        
        if HAS_GPIO:
            try:
                # Reset status LEDs
                status_pin = self.hw_config.get_status_led_pin()
                error_pin = self.hw_config.get_error_led_pin()
                
                GPIO.output(status_pin, GPIO.HIGH)  # Status ON
                GPIO.output(error_pin, GPIO.LOW)    # Error OFF
                
                self.get_logger().info("✅ GPIO reset complete")
                
            except Exception as e:
                self.get_logger().error(f"❌ GPIO reset failed: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("🛑 Shutting down GPIO Control Node...")
        
        # Deactivate all magnets
        self.deactivate_all_magnets()
        
        # Cancel all timers
        for magnet in ['front', 'back']:
            self.cancel_magnet_timer(magnet)
        
        # Cleanup GPIO
        if HAS_GPIO:
            try:
                GPIO.cleanup()
                self.get_logger().info("✅ GPIO cleanup complete")
            except Exception as e:
                self.get_logger().error(f"❌ GPIO cleanup error: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gpio_node = GpioControlNode()
        rclpy.spin(gpio_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gpio_node' in locals():
            gpio_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
