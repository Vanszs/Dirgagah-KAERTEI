#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from drone_mvp.msg import SensorStatus
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class KalibrasiNavigatorNode(Node):
    def __init__(self):
        super().__init__('kalibrasi_navigator')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('center_tolerance', 0.2),
                ('correction_speed', 0.3),
                ('max_correction_time', 5.0),
                ('correction_threshold', 0.1)
            ]
        )
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', qos_profile)
        self.status_pub = self.create_publisher(String, '/navigation/calibration_status', qos_profile)
        
        # Subscribers
        self.sensor_sub = self.create_subscription(
            SensorStatus, '/sensors/status', self.sensor_callback, qos_profile)
        self.mission_command_sub = self.create_subscription(
            String, '/mission/command', self.mission_command_callback, qos_profile)
        
        # Navigation state
        self.current_sensor_data = None
        self.is_calibrating = False
        self.calibration_start_time = None
        self.target_achieved = False
        
        # Control parameters
        self.correction_speed = self.get_parameter('correction_speed').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.max_correction_time = self.get_parameter('max_correction_time').value
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop_callback)  # 10 Hz
        
        self.get_logger().info("Kalibrasi Navigator Node initialized")
    
    def sensor_callback(self, msg):
        """Handle sensor data updates"""
        self.current_sensor_data = msg
        
        # Check if calibration is needed
        if not self.is_calibrating and not msg.is_centered:
            self.get_logger().info(f"Drift detected: {msg.status}")
    
    def mission_command_callback(self, msg):
        """Handle mission commands"""
        if msg.data == "CALIBRATE_CENTER":
            self.start_calibration()
        elif msg.data == "STOP_CALIBRATION":
            self.stop_calibration()
    
    def start_calibration(self):
        """Start centering calibration"""
        if not self.is_calibrating:
            self.is_calibrating = True
            self.calibration_start_time = time.time()
            self.target_achieved = False
            self.get_logger().info("Starting centering calibration")
            
            # Publish status
            status_msg = String()
            status_msg.data = "CALIBRATION_STARTED"
            self.status_pub.publish(status_msg)
    
    def stop_calibration(self):
        """Stop centering calibration"""
        if self.is_calibrating:
            self.is_calibrating = False
            self.calibration_start_time = None
            
            # Stop movement
            self.publish_zero_velocity()
            
            self.get_logger().info("Calibration stopped")
            
            # Publish status
            status_msg = String()
            status_msg.data = "CALIBRATION_STOPPED"
            self.status_pub.publish(status_msg)
    
    def control_loop_callback(self):
        """Main control loop for calibration"""
        if not self.is_calibrating or self.current_sensor_data is None:
            return
        
        # Check timeout
        if (time.time() - self.calibration_start_time) > self.max_correction_time:
            self.get_logger().warn("Calibration timeout - stopping")
            self.stop_calibration()
            return
        
        # Check if already centered
        if self.current_sensor_data.is_centered:
            if not self.target_achieved:
                self.target_achieved = True
                self.get_logger().info("Centering achieved!")
                
                # Publish status
                status_msg = String()
                status_msg.data = "CALIBRATION_COMPLETED"
                self.status_pub.publish(status_msg)
                
                # Stop movement
                self.publish_zero_velocity()
                
                # Continue calibration for a bit to stabilize
                time.sleep(0.5)
                self.stop_calibration()
            return
        
        # Calculate correction needed
        distance_left = self.current_sensor_data.distance_left
        distance_right = self.current_sensor_data.distance_right
        
        # Avoid division by zero or invalid readings
        if distance_left <= 0 or distance_right <= 0:
            self.get_logger().warn("Invalid sensor readings")
            return
        
        # Calculate lateral error (positive means drift right, negative means drift left)
        lateral_error = distance_right - distance_left
        
        # Apply deadband to avoid oscillation
        if abs(lateral_error) < self.get_parameter('correction_threshold').value:
            self.publish_zero_velocity()
            return
        
        # Calculate correction command
        correction_velocity = self.calculate_correction_velocity(lateral_error)
        
        # Publish correction command
        self.publish_correction_velocity(correction_velocity)
        
        self.get_logger().debug(f"Lateral error: {lateral_error:.3f}, Correction: {correction_velocity:.3f}")
    
    def calculate_correction_velocity(self, lateral_error):
        """Calculate correction velocity based on lateral error"""
        # Simple proportional controller
        kp = 1.0  # Proportional gain
        
        # Calculate desired lateral velocity (positive = move right, negative = move left)
        correction_velocity = -kp * lateral_error
        
        # Limit correction speed
        max_speed = self.correction_speed
        correction_velocity = max(-max_speed, min(max_speed, correction_velocity))
        
        return correction_velocity
    
    def publish_correction_velocity(self, lateral_velocity):
        """Publish velocity command for lateral correction"""
        cmd_vel = Twist()
        
        # Set lateral velocity (y-axis in drone body frame)
        cmd_vel.linear.y = lateral_velocity
        
        # Keep other velocities at zero during calibration
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def publish_zero_velocity(self):
        """Publish zero velocity to stop movement"""
        cmd_vel = Twist()
        # All velocities are zero by default
        self.cmd_vel_pub.publish(cmd_vel)
    
    def get_calibration_status(self):
        """Get current calibration status"""
        if not self.is_calibrating:
            return "IDLE"
        elif self.target_achieved:
            return "COMPLETED"
        else:
            return "CALIBRATING"


def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibration_node = KalibrasiNavigatorNode()
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'calibration_node' in locals():
            calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
