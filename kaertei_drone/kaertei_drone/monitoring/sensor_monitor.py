#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

# from kaertei_drone.msg import SensorStatus  # TODO: Create custom message
from std_msgs.msg import String, Bool  # Temporary placeholder
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
    import board
    import adafruit_vl53l0x
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False


class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('distance_threshold', 1.5),
                ('center_tolerance', 0.2),
                ('sensor_pins.distance_left', 23),
                ('sensor_pins.distance_right', 24), 
                ('sensor_pins.distance_front', 25),
                ('update_rate', 10.0)
            ]
        )
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        # self.sensor_status_pub = self.create_publisher(SensorStatus, '/sensors/status', qos_profile)
        self.sensor_status_pub = self.create_publisher(String, '/sensors/status', qos_profile)  # Temporary
        self.distance_left_pub = self.create_publisher(Float32, '/sensors/distance_left', qos_profile)
        self.distance_right_pub = self.create_publisher(Float32, '/sensors/distance_right', qos_profile)
        self.distance_front_pub = self.create_publisher(Float32, '/sensors/distance_front', qos_profile)
        self.centered_pub = self.create_publisher(Bool, '/sensors/is_centered', qos_profile)
        
        # Initialize sensors
        self.sensors_initialized = False
        self.left_sensor = None
        self.right_sensor = None
        self.front_sensor = None
        
        if RPI_AVAILABLE:
            self.init_real_sensors()
        else:
            self.get_logger().warn("RPi.GPIO not available - using mock sensor data")
        
        # Timer for sensor reading
        update_rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0 / update_rate, self.read_sensors_callback)
        
        self.get_logger().info("Sensor Monitor Node initialized")
    
    def init_real_sensors(self):
        """Initialize real ToF sensors"""
        try:
            # Initialize I2C
            i2c = board.I2C()
            
            # Initialize VL53L0X sensors
            # Note: In real implementation, you'd need to handle multiple sensors
            # with different I2C addresses or use multiplexer
            self.left_sensor = adafruit_vl53l0x.VL53L0X(i2c)
            self.right_sensor = adafruit_vl53l0x.VL53L0X(i2c)  # Different address needed
            self.front_sensor = adafruit_vl53l0x.VL53L0X(i2c)  # Different address needed
            
            self.sensors_initialized = True
            self.get_logger().info("Real ToF sensors initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize real sensors: {e}")
            self.sensors_initialized = False
    
    def read_sensors_callback(self):
        """Read sensor data and publish status"""
        try:
            # Read distances
            if self.sensors_initialized:
                distance_left = self.read_real_distance('left')
                distance_right = self.read_real_distance('right')
                distance_front = self.read_real_distance('front')
            else:
                distance_left = self.mock_distance_reading('left')
                distance_right = self.mock_distance_reading('right')
                distance_front = self.mock_distance_reading('front')
            
            # Check if drone is centered
            center_tolerance = self.get_parameter('center_tolerance').value
            distance_diff = abs(distance_left - distance_right)
            is_centered = distance_diff <= center_tolerance
            
            # Create status message using standard ROS messages
            # Publish individual values since custom SensorStatus not available
            
            # Publish distances individually
            left_msg = Float32()
            left_msg.data = distance_left
            self.distance_left_pub.publish(left_msg)
            
            right_msg = Float32()
            right_msg.data = distance_right
            self.distance_right_pub.publish(right_msg)
            
            front_msg = Float32()
            front_msg.data = distance_front
            self.distance_front_pub.publish(front_msg)
            
            # Publish centered status
            centered_msg = Bool()
            centered_msg.data = is_centered
            self.centered_pub.publish(centered_msg)
            
            # Determine status
            status = ""
            if distance_front < self.get_parameter('distance_threshold').value:
                status = "OBSTACLE_FRONT"
            elif not is_centered:
                if distance_left < distance_right:
                    status = "DRIFT_LEFT"
                else:
                    status = "DRIFT_RIGHT"
            else:
                status = "CLEAR"
            
            # Publish status as string message
            status_msg = String()
            status_msg.data = status
            
            # Publish messages
            self.sensor_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {e}")
    
    def read_real_distance(self, sensor_position):
        """Read distance from real ToF sensor"""
        try:
            if sensor_position == 'left' and self.left_sensor:
                return self.left_sensor.range / 10.0  # Convert mm to cm
            elif sensor_position == 'right' and self.right_sensor:
                return self.right_sensor.range / 10.0
            elif sensor_position == 'front' and self.front_sensor:
                return self.front_sensor.range / 10.0
            else:
                return float('inf')  # No reading available
                
        except Exception as e:
            self.get_logger().error(f"Error reading {sensor_position} sensor: {e}")
            return float('inf')
    
    def mock_distance_reading(self, sensor_position):
        """Generate mock sensor readings for testing"""
        import random
        
        # Simulate different scenarios based on time
        current_time = time.time()
        scenario = int(current_time / 10) % 4  # Change scenario every 10 seconds
        
        if scenario == 0:  # Normal corridor
            if sensor_position == 'left':
                return 1.0 + random.uniform(-0.1, 0.1)
            elif sensor_position == 'right':
                return 1.0 + random.uniform(-0.1, 0.1)
            else:  # front
                return 5.0 + random.uniform(-0.5, 0.5)
                
        elif scenario == 1:  # Drifting left
            if sensor_position == 'left':
                return 0.5 + random.uniform(-0.05, 0.05)
            elif sensor_position == 'right':
                return 1.5 + random.uniform(-0.1, 0.1)
            else:  # front
                return 5.0 + random.uniform(-0.5, 0.5)
                
        elif scenario == 2:  # Drifting right
            if sensor_position == 'left':
                return 1.5 + random.uniform(-0.1, 0.1)
            elif sensor_position == 'right':
                return 0.5 + random.uniform(-0.05, 0.05)
            else:  # front
                return 5.0 + random.uniform(-0.5, 0.5)
                
        else:  # Open space (turn detection)
            if sensor_position == 'left':
                return 5.0 + random.uniform(-0.5, 0.5)
            elif sensor_position == 'right':
                return 1.0 + random.uniform(-0.1, 0.1)
            else:  # front
                return 3.0 + random.uniform(-0.3, 0.3)
    
    def destroy_node(self):
        """Clean up GPIO resources"""
        if RPI_AVAILABLE and self.sensors_initialized:
            try:
                # Clean up GPIO if used
                pass
            except Exception as e:
                self.get_logger().error(f"Error cleaning up sensors: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_node = SensorMonitorNode()
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sensor_node' in locals():
            sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
