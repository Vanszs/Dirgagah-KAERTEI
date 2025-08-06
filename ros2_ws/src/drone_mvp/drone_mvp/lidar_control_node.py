#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
import serial
import threading
import time
import struct
from .hardware_config import HardwareConfig

class LidarControlNode(Node):
    """
    LiDAR Control Node for KAERTEI 2025 FAIO Drone
    Handles 3x TF Mini Plus LiDAR sensors on Raspberry Pi 5
    
    Features:
    - Front/Left/Right LiDAR distance measurement
    - Serial communication over USB
    - Obstacle detection and avoidance
    - Real-time range publishing
    """

    def __init__(self):
        super().__init__('lidar_control_node')
        self.hw_config = HardwareConfig()
        
        # Initialize variables
        self.lidar_connections = {}
        self.lidar_data = {'front': 0.0, 'left': 0.0, 'right': 0.0}
        self.is_running = False
        
        # ROS Publishers
        self.front_range_pub = self.create_publisher(Range, '/sensors/lidar/front', 10)
        self.left_range_pub = self.create_publisher(Range, '/sensors/lidar/left', 10)
        self.right_range_pub = self.create_publisher(Range, '/sensors/lidar/right', 10)
        self.obstacle_pub = self.create_publisher(String, '/sensors/obstacles', 10)
        
        # ROS Subscribers
        self.command_sub = self.create_subscription(
            String, '/sensors/lidar/command', self.command_callback, 10)
        
        # Initialize LiDAR sensors
        self.initialize_lidars()
        
        # Start reading threads
        self.start_reading_threads()
        
        # Status timer
        self.status_timer = self.create_timer(0.1, self.publish_ranges)  # 10Hz
        self.obstacle_timer = self.create_timer(0.2, self.check_obstacles)  # 5Hz
        
        self.get_logger().info("🔍 LiDAR Control Node initialized")
        self.get_logger().info(f"📍 Front: {self.hw_config.get_lidar_interfaces()['front']}")
        self.get_logger().info(f"📍 Left: {self.hw_config.get_lidar_interfaces()['left']}")
        self.get_logger().info(f"📍 Right: {self.hw_config.get_lidar_interfaces()['right']}")

    def initialize_lidars(self):
        """Initialize serial connections to all LiDAR sensors"""
        interfaces = self.hw_config.get_lidar_interfaces()
        baud_rate = self.hw_config.get_lidar_baud_rate()
        
        for position, interface in interfaces.items():
            try:
                connection = serial.Serial(
                    port=interface,
                    baudrate=baud_rate,
                    timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                self.lidar_connections[position] = connection
                self.get_logger().info(f"✅ {position.title()} LiDAR connected: {interface}")
                
                # Configure TF Mini Plus for continuous measurement
                self.configure_tfmini(connection)
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to connect {position} LiDAR {interface}: {e}")
                self.lidar_connections[position] = None

    def configure_tfmini(self, connection):
        """Configure TF Mini Plus for optimal performance"""
        try:
            # Set measurement frequency to 100Hz
            freq_command = bytes([0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7])
            connection.write(freq_command)
            time.sleep(0.1)
            
            # Enable continuous measurement mode
            continuous_command = bytes([0x5A, 0x05, 0x00, 0x01, 0x60])
            connection.write(continuous_command)
            time.sleep(0.1)
            
            self.get_logger().info("✅ TF Mini Plus configured successfully")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to configure TF Mini Plus: {e}")

    def start_reading_threads(self):
        """Start reading threads for each LiDAR sensor"""
        self.is_running = True
        
        for position, connection in self.lidar_connections.items():
            if connection:
                thread = threading.Thread(
                    target=self.read_lidar_data,
                    args=(position, connection),
                    daemon=True
                )
                thread.start()
                self.get_logger().info(f"🔄 Started reading thread for {position} LiDAR")

    def read_lidar_data(self, position, connection):
        """Read data from a specific LiDAR sensor"""
        while self.is_running and rclpy.ok():
            try:
                # TF Mini Plus data format: Header(2) + Dist(2) + Strength(2) + Temp(2) + Checksum(1)
                data = connection.read(9)
                
                if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                    # Extract distance (in cm, convert to meters)
                    distance_cm = struct.unpack('<H', data[2:4])[0]
                    distance_m = distance_cm / 100.0
                    
                    # Extract signal strength
                    strength = struct.unpack('<H', data[4:6])[0]
                    
                    # Validate measurement
                    max_range = self.hw_config.get_lidar_max_range()
                    if 0.1 <= distance_m <= max_range and strength > 100:
                        self.lidar_data[position] = distance_m
                    else:
                        # Invalid measurement, keep previous value or set to max
                        if distance_m > max_range:
                            self.lidar_data[position] = max_range
                            
                else:
                    # Invalid packet, try to sync
                    connection.reset_input_buffer()
                    
            except Exception as e:
                self.get_logger().error(f"❌ Error reading {position} LiDAR: {e}")
                time.sleep(0.1)

    def publish_ranges(self):
        """Publish range data for all LiDAR sensors"""
        current_time = self.get_clock().now().to_msg()
        max_range = self.hw_config.get_lidar_max_range()
        
        for position, distance in self.lidar_data.items():
            range_msg = Range()
            range_msg.header.stamp = current_time
            range_msg.header.frame_id = f'lidar_{position}_link'
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.04  # ~2.3 degrees
            range_msg.min_range = 0.1
            range_msg.max_range = max_range
            range_msg.range = distance
            
            # Publish to appropriate topic
            if position == 'front':
                self.front_range_pub.publish(range_msg)
            elif position == 'left':
                self.left_range_pub.publish(range_msg)
            elif position == 'right':
                self.right_range_pub.publish(range_msg)

    def check_obstacles(self):
        """Check for obstacles and publish warnings"""
        threshold = self.hw_config.get_lidar_detection_threshold()
        obstacles = []
        
        for position, distance in self.lidar_data.items():
            if 0.1 < distance < threshold:
                obstacles.append(f"{position}:{distance:.2f}m")
        
        if obstacles:
            obstacle_msg = String()
            obstacle_msg.data = f"OBSTACLES_DETECTED:{','.join(obstacles)}"
            self.obstacle_pub.publish(obstacle_msg)
            
            # Log warning
            self.get_logger().warn(f"⚠️  Obstacles detected: {', '.join(obstacles)}")

    def command_callback(self, msg):
        """Handle LiDAR commands"""
        command = msg.data.upper()
        
        if command == "STATUS":
            self.get_logger().info("📊 LiDAR Status:")
            for position, distance in self.lidar_data.items():
                status = "✅ CONNECTED" if self.lidar_connections.get(position) else "❌ DISCONNECTED"
                self.get_logger().info(f"  {position.title()}: {distance:.2f}m - {status}")
                
        elif command == "RESET":
            self.get_logger().info("🔄 Resetting LiDAR connections...")
            self.reset_lidars()
            
        elif command == "CALIBRATE":
            self.get_logger().info("⚙️  Calibrating LiDAR sensors...")
            self.calibrate_lidars()

    def reset_lidars(self):
        """Reset all LiDAR connections"""
        # Stop reading threads
        self.is_running = False
        time.sleep(0.5)
        
        # Close connections
        for connection in self.lidar_connections.values():
            if connection and connection.is_open:
                connection.close()
        
        # Reinitialize
        self.initialize_lidars()
        self.start_reading_threads()

    def calibrate_lidars(self):
        """Calibrate LiDAR sensors"""
        self.get_logger().info("🎯 Starting LiDAR calibration...")
        
        # Take baseline measurements
        baseline = {}
        for i in range(10):
            for position in self.lidar_data:
                if position not in baseline:
                    baseline[position] = []
                baseline[position].append(self.lidar_data[position])
            time.sleep(0.1)
        
        # Calculate averages
        for position, values in baseline.items():
            avg = sum(values) / len(values)
            self.get_logger().info(f"📏 {position.title()} baseline: {avg:.3f}m")

    def get_distances(self):
        """Get current distance readings"""
        return self.lidar_data.copy()

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("🛑 Shutting down LiDAR Control Node...")
        
        # Stop reading threads
        self.is_running = False
        time.sleep(0.5)
        
        # Close all connections
        for position, connection in self.lidar_connections.items():
            if connection and connection.is_open:
                connection.close()
                self.get_logger().info(f"✅ Closed {position} LiDAR connection")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lidar_node = LidarControlNode()
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'lidar_node' in locals():
            lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
