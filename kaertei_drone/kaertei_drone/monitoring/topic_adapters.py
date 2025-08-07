#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import json

from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Point, Twist, Vector3, PoseStamped
from sensor_msgs.msg import Image, PointCloud2, NavSatFix
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool as CommandBoolSrv, SetMode as SetModeSrv
from kaertei_drone.hardware.hardware_config import HardwareConfig


class TopicAdaptersNode(Node):
    def __init__(self):
        super().__init__('topic_adapters')
        
        # Load hardware config
        self.hw_config = HardwareConfig()
        
        # QoS profiles for different requirements
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Changed to BEST_EFFORT to match flight_state_monitor.py for compatibility
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 10.0),          # Hz
                ('command_timeout', 2.0),       # seconds
                ('enable_mavros_bridge', True), # Enable MAVROS bridging
                ('enable_vision_bridge', True), # Enable vision bridging
                ('enable_sensor_bridge', True), # Enable sensor bridging
                ('enable_debug_logging', False), # Enable debug output
            ]
        )
        
        # ===========================================
        # MAVROS CONTROL ADAPTERS
        # ===========================================
        
        # Publishers to MAVROS
        self.mavros_rc_override_pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', self.control_qos)
        self.mavros_setpoint_velocity_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self.control_qos)
        self.mavros_setpoint_position_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', self.control_qos)
        
        # Service clients for MAVROS commands
        self.arm_service = self.create_client(CommandBoolSrv, '/mavros/cmd/arming')
        self.set_mode_service = self.create_client(SetModeSrv, '/mavros/set_mode')
        
        # Subscribers from mission control
        self.velocity_command_sub = self.create_subscription(
            Twist, '/control/velocity_command', self.velocity_command_callback, self.control_qos)
        self.position_command_sub = self.create_subscription(
            PoseStamped, '/control/position_command', self.position_command_callback, self.control_qos)
        self.arm_command_sub = self.create_subscription(
            Bool, '/control/arm_command', self.arm_command_callback, self.control_qos)
        self.mode_command_sub = self.create_subscription(
            String, '/control/mode_command', self.mode_command_callback, self.control_qos)
        
        # ===========================================
        # VISION SYSTEM ADAPTERS
        # ===========================================
        
        # Publishers - Vision status bridging
        self.vision_status_pub = self.create_publisher(
            String, '/mission/vision_status', self.sensor_qos)
        self.object_detected_pub = self.create_publisher(
            Bool, '/mission/object_detected', self.sensor_qos)
        self.alignment_status_pub = self.create_publisher(
            Bool, '/mission/aligned', self.sensor_qos)
        self.target_position_pub = self.create_publisher(
            Point, '/mission/target_position', self.sensor_qos)
        
        # Subscribers - Vision system inputs
        self.vision_detection_sub = self.create_subscription(
            Point, '/vision/object_position', self.vision_detection_callback, self.sensor_qos)
        self.vision_aligned_sub = self.create_subscription(
            Bool, '/vision/aligned', self.vision_aligned_callback, self.sensor_qos)
        self.dropzone_detected_sub = self.create_subscription(
            Point, '/vision/dropzone_position', self.dropzone_detection_callback, self.sensor_qos)
        
        # ===========================================
        # SENSOR DATA ADAPTERS
        # ===========================================
        
        # Publishers - Sensor status bridging
        self.lidar_status_pub = self.create_publisher(
            String, '/mission/lidar_status', self.sensor_qos)
        self.obstacle_detected_pub = self.create_publisher(
            Bool, '/mission/obstacle_detected', self.sensor_qos)
        self.sensor_health_pub = self.create_publisher(
            String, '/mission/sensor_health', self.sensor_qos)
        
        # Subscribers - Sensor inputs
        self.lidar_front_sub = self.create_subscription(
            Float32, '/sensors/lidar_front', self.lidar_front_callback, self.sensor_qos)
        self.lidar_left_sub = self.create_subscription(
            Float32, '/sensors/lidar_left', self.lidar_left_callback, self.sensor_qos)
        self.lidar_right_sub = self.create_subscription(
            Float32, '/sensors/lidar_right', self.lidar_right_callback, self.sensor_qos)
        
        # ===========================================
        # PAYLOAD SYSTEM ADAPTERS
        # ===========================================
        
        # Publishers - Payload status bridging
        self.payload_status_pub = self.create_publisher(
            String, '/mission/payload_status', self.sensor_qos)
        self.electromagnet_status_pub = self.create_publisher(
            Bool, '/mission/electromagnet_active', self.sensor_qos)
        
        # Subscribers - Payload control
        self.payload_command_sub = self.create_subscription(
            String, '/control/payload_command', self.payload_command_callback, self.control_qos)
        self.electromagnet_command_sub = self.create_subscription(
            Bool, '/control/electromagnet', self.electromagnet_command_callback, self.control_qos)
        
        # ===========================================
        # GPS WAYPOINT ADAPTERS  
        # ===========================================
        
        # Publishers - GPS status bridging
        self.gps_status_pub = self.create_publisher(
            String, '/mission/gps_status', self.sensor_qos)
        self.waypoint_reached_pub = self.create_publisher(
            Bool, '/mission/waypoint_reached', self.sensor_qos)
        
        # Subscribers - GPS monitoring
        self.gps_quality_sub = self.create_subscription(
            String, '/gps/quality_status', self.gps_quality_callback, self.sensor_qos)
        self.gps_moving_sub = self.create_subscription(
            Bool, '/gps/moving_status', self.gps_moving_callback, self.sensor_qos)
        
        # ===========================================
        # SYSTEM HEALTH ADAPTERS
        # ===========================================
        
        # Publishers - System status bridging
        self.system_status_pub = self.create_publisher(
            String, '/mission/system_status', self.control_qos)
        self.emergency_status_pub = self.create_publisher(
            Bool, '/mission/emergency_active', self.control_qos)
        
        # Subscribers - System monitoring
        self.flight_status_sub = self.create_subscription(
            String, '/flight_controller/status', self.flight_status_callback, self.control_qos)
        self.health_status_sub = self.create_subscription(
            String, '/flight_controller/health_status', self.health_status_callback, self.control_qos)
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency/active', self.emergency_callback, self.control_qos)
        
        # State tracking
        self.last_vision_detection = None
        self.last_object_position = None
        self.is_aligned = False
        self.lidar_distances = {'front': 0.0, 'left': 0.0, 'right': 0.0}
        self.obstacle_detected = False
        self.current_gps_quality = "UNKNOWN"
        self.is_moving = False
        self.current_flight_status = "UNKNOWN"
        self.system_health_status = "UNKNOWN"
        self.emergency_active = False
        
        # Timer for periodic updates
        update_rate = self.get_parameter('update_rate').value
        self.adapter_timer = self.create_timer(1.0 / update_rate, self.adapter_callback)
        
        self.get_logger().info("Topic Adapters Node initialized")
        self.get_logger().info("Bridging between mission control and hardware systems")
    
    # ===========================================
    # MAVROS CONTROL CALLBACKS
    # ===========================================
    
    def velocity_command_callback(self, msg):
        """Forward velocity commands to MAVROS"""
        if self.get_parameter('enable_mavros_bridge').value:
            self.mavros_setpoint_velocity_pub.publish(msg)
            if self.get_parameter('enable_debug_logging').value:
                self.get_logger().debug(f"Velocity command: {msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}")
    
    def position_command_callback(self, msg):
        """Forward position commands to MAVROS"""
        if self.get_parameter('enable_mavros_bridge').value:
            self.mavros_setpoint_position_pub.publish(msg)
            if self.get_parameter('enable_debug_logging').value:
                self.get_logger().debug(f"Position command: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}")
    
    def arm_command_callback(self, msg):
        """Forward arming commands to MAVROS"""
        if self.get_parameter('enable_mavros_bridge').value:
            if self.arm_service.wait_for_service(timeout_sec=1.0):
                request = CommandBoolSrv.Request()
                request.value = msg.data
                future = self.arm_service.call_async(request)
                self.get_logger().info(f"Arm command sent: {msg.data}")
            else:
                self.get_logger().warn("ARM service not available")
    
    def mode_command_callback(self, msg):
        """Forward mode commands to MAVROS"""
        if self.get_parameter('enable_mavros_bridge').value:
            if self.set_mode_service.wait_for_service(timeout_sec=1.0):
                request = SetModeSrv.Request()
                request.custom_mode = msg.data
                future = self.set_mode_service.call_async(request)
                self.get_logger().info(f"Mode command sent: {msg.data}")
            else:
                self.get_logger().warn("SET_MODE service not available")
    
    # ===========================================
    # VISION SYSTEM CALLBACKS
    # ===========================================
    
    def vision_detection_callback(self, msg):
        """Handle vision detection updates"""
        if self.get_parameter('enable_vision_bridge').value:
            self.last_object_position = msg
            self.last_vision_detection = time.time()
            
            # Publish object detection status
            object_detected = Bool()
            object_detected.data = True
            self.object_detected_pub.publish(object_detected)
            
            # Forward target position
            self.target_position_pub.publish(msg)
    
    def vision_aligned_callback(self, msg):
        """Handle vision alignment updates"""
        if self.get_parameter('enable_vision_bridge').value:
            self.is_aligned = msg.data
            self.alignment_status_pub.publish(msg)
    
    def dropzone_detection_callback(self, msg):
        """Handle dropzone detection updates"""
        if self.get_parameter('enable_vision_bridge').value:
            # Forward dropzone position as target
            self.target_position_pub.publish(msg)
    
    # ===========================================
    # SENSOR CALLBACKS
    # ===========================================
    
    def lidar_front_callback(self, msg):
        """Handle front LiDAR data"""
        if self.get_parameter('enable_sensor_bridge').value:
            self.lidar_distances['front'] = msg.data
            self.check_obstacle_detection()
    
    def lidar_left_callback(self, msg):
        """Handle left LiDAR data"""
        if self.get_parameter('enable_sensor_bridge').value:
            self.lidar_distances['left'] = msg.data
            self.check_obstacle_detection()
    
    def lidar_right_callback(self, msg):
        """Handle right LiDAR data"""
        if self.get_parameter('enable_sensor_bridge').value:
            self.lidar_distances['right'] = msg.data
            self.check_obstacle_detection()
    
    def check_obstacle_detection(self):
        """Check if obstacles are detected"""
        obstacle_threshold = 1.0  # 1 meter
        
        obstacle_detected = any(
            distance > 0 and distance < obstacle_threshold 
            for distance in self.lidar_distances.values()
        )
        
        if obstacle_detected != self.obstacle_detected:
            self.obstacle_detected = obstacle_detected
            msg = Bool()
            msg.data = obstacle_detected
            self.obstacle_detected_pub.publish(msg)
    
    # ===========================================
    # PAYLOAD CALLBACKS
    # ===========================================
    
    def payload_command_callback(self, msg):
        """Handle payload commands"""
        # Forward to GPIO control node
        command_msg = String()
        command_msg.data = msg.data
        # This would be forwarded to GPIO control
        if self.get_parameter('enable_debug_logging').value:
            self.get_logger().info(f"Payload command: {msg.data}")
    
    def electromagnet_command_callback(self, msg):
        """Handle electromagnet commands"""
        # Forward to GPIO control node
        electromagnet_msg = Bool()
        electromagnet_msg.data = msg.data
        # This would be forwarded to GPIO control
        if self.get_parameter('enable_debug_logging').value:
            self.get_logger().info(f"Electromagnet command: {msg.data}")
    
    # ===========================================
    # GPS CALLBACKS
    # ===========================================
    
    def gps_quality_callback(self, msg):
        """Handle GPS quality updates"""
        self.current_gps_quality = msg.data
    
    def gps_moving_callback(self, msg):
        """Handle GPS movement updates"""
        self.is_moving = msg.data
    
    # ===========================================
    # SYSTEM STATUS CALLBACKS
    # ===========================================
    
    def flight_status_callback(self, msg):
        """Handle flight controller status updates"""
        self.current_flight_status = msg.data
    
    def health_status_callback(self, msg):
        """Handle system health updates"""
        self.system_health_status = msg.data
    
    def emergency_callback(self, msg):
        """Handle emergency status updates"""
        self.emergency_active = msg.data
        self.emergency_status_pub.publish(msg)
    
    # ===========================================
    # PERIODIC UPDATES
    # ===========================================
    
    def adapter_callback(self):
        """Main adapter loop for periodic updates"""
        try:
            current_time = time.time()
            
            # Update vision status
            self.update_vision_status(current_time)
            
            # Update sensor status
            self.update_sensor_status()
            
            # Update GPS status
            self.update_gps_status()
            
            # Update system status
            self.update_system_status()
            
        except Exception as e:
            self.get_logger().error(f"Error in adapter callback: {e}")
    
    def update_vision_status(self, current_time):
        """Update vision system status"""
        vision_timeout = 2.0  # seconds
        
        if (self.last_vision_detection is None or 
            current_time - self.last_vision_detection > vision_timeout):
            status = "NO_DETECTION"
            # Publish no object detected
            object_detected = Bool()
            object_detected.data = False
            self.object_detected_pub.publish(object_detected)
        elif self.is_aligned:
            status = "ALIGNED"
        else:
            status = "DETECTING"
        
        vision_status = String()
        vision_status.data = status
        self.vision_status_pub.publish(vision_status)
    
    def update_sensor_status(self):
        """Update sensor system status"""
        # Check LiDAR health
        lidar_healthy = all(distance >= 0 for distance in self.lidar_distances.values())
        
        if lidar_healthy:
            if self.obstacle_detected:
                status = "OBSTACLE_DETECTED"
            else:
                status = "CLEAR"
        else:
            status = "SENSOR_ERROR"
        
        lidar_status = String()
        lidar_status.data = status
        self.lidar_status_pub.publish(lidar_status)
        
        # Overall sensor health
        sensor_health = String()
        sensor_health.data = "HEALTHY" if lidar_healthy else "DEGRADED"
        self.sensor_health_pub.publish(sensor_health)
    
    def update_gps_status(self):
        """Update GPS system status"""
        gps_status = String()
        if self.current_gps_quality == "GOOD" and self.is_moving:
            gps_status.data = "TRACKING"
        elif self.current_gps_quality == "GOOD":
            gps_status.data = "FIXED"
        else:
            gps_status.data = self.current_gps_quality
        
        self.gps_status_pub.publish(gps_status)
    
    def update_system_status(self):
        """Update overall system status"""
        if self.emergency_active:
            status = "EMERGENCY"
        elif "ERROR" in self.current_flight_status:
            status = "ERROR"
        elif "WARNING" in self.current_flight_status:
            status = "WARNING"
        elif "ARMED" in self.current_flight_status:
            status = "ACTIVE"
        else:
            status = "READY"
        
        system_status = String()
        system_status.data = status
        self.system_status_pub.publish(system_status)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        topic_adapters = TopicAdaptersNode()
        rclpy.spin(topic_adapters)
    except KeyboardInterrupt:
        pass
    finally:
        if 'topic_adapters' in locals():
            topic_adapters.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
