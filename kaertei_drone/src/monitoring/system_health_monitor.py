#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import psutil
import os

from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from drone_mvp.hardware_config import HardwareConfig


class SystemHealthMonitorNode(Node):
    def __init__(self):
        super().__init__('system_health_monitor')
        
        # Load hardware config
        self.hw_config = HardwareConfig()
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 1.0),               # Hz
                ('cpu_warning_threshold', 80.0),    # %
                ('memory_warning_threshold', 85.0), # %
                ('disk_warning_threshold', 90.0),   # %
                ('temperature_warning_threshold', 70.0), # °C
                ('node_timeout', 10.0),             # seconds
                ('enable_performance_monitoring', True),
                ('enable_hardware_monitoring', True),
                ('enable_ros_monitoring', True),
            ]
        )
        
        # QoS profiles
        self.status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers - System health status
        self.system_health_pub = self.create_publisher(String, '/system/health_status', self.status_qos)
        self.system_errors_pub = self.create_publisher(String, '/system/errors', self.status_qos)
        self.system_warnings_pub = self.create_publisher(String, '/system/warnings', self.status_qos)
        self.system_performance_pub = self.create_publisher(String, '/system/performance', self.status_qos)
        
        # Publishers - Hardware monitoring
        self.cpu_usage_pub = self.create_publisher(Float32, '/system/cpu_usage', self.sensor_qos)
        self.memory_usage_pub = self.create_publisher(Float32, '/system/memory_usage', self.sensor_qos)
        self.disk_usage_pub = self.create_publisher(Float32, '/system/disk_usage', self.sensor_qos)
        self.temperature_pub = self.create_publisher(Float32, '/system/temperature', self.sensor_qos)
        self.uptime_pub = self.create_publisher(Float32, '/system/uptime', self.sensor_qos)
        
        # Publishers - ROS node monitoring
        self.nodes_status_pub = self.create_publisher(String, '/system/ros_nodes_status', self.status_qos)
        self.active_nodes_count_pub = self.create_publisher(Int32, '/system/active_nodes_count', self.sensor_qos)
        self.failed_nodes_pub = self.create_publisher(String, '/system/failed_nodes', self.status_qos)
        
        # Subscribers - Critical system components
        self.flight_status_sub = self.create_subscription(
            String, '/flight_controller/status', self.flight_status_callback, self.sensor_qos)
        self.battery_voltage_sub = self.create_subscription(
            Float32, '/flight_controller/battery_voltage', self.battery_callback, self.sensor_qos)
        self.gps_quality_sub = self.create_subscription(
            String, '/gps/quality_status', self.gps_callback, self.sensor_qos)
        self.vision_status_sub = self.create_subscription(
            String, '/mission/vision_status', self.vision_callback, self.sensor_qos)
        self.lidar_status_sub = self.create_subscription(
            String, '/mission/lidar_status', self.lidar_callback, self.sensor_qos)
        self.emergency_status_sub = self.create_subscription(
            Bool, '/emergency/active', self.emergency_callback, self.sensor_qos)
        
        # System state tracking
        self.system_errors = []
        self.system_warnings = []
        self.system_performance_issues = []
        
        # Hardware monitoring
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.disk_usage = 0.0
        self.system_temperature = 0.0
        self.system_uptime = 0.0
        
        # Component status
        self.component_status = {
            'flight_controller': {'status': 'UNKNOWN', 'last_update': None},
            'battery': {'voltage': 0.0, 'last_update': None},
            'gps': {'quality': 'UNKNOWN', 'last_update': None},
            'vision': {'status': 'UNKNOWN', 'last_update': None},
            'lidar': {'status': 'UNKNOWN', 'last_update': None},
            'emergency': {'active': False, 'last_update': None},
        }
        
        # ROS nodes monitoring
        self.expected_nodes = [
            'checkpoint_mission_node',
            'flight_state_monitor',
            'topic_adapters',
            'gps_monitor',
            'gps_waypoint_monitor',
            'emergency_controller',
            'vision_detector_node',
            'lidar_control_node',
            'gpio_control_node',
        ]
        
        self.active_nodes = []
        self.failed_nodes = []
        self.last_node_check = None
        
        # Performance monitoring
        self.performance_history = []
        self.max_history_size = 60  # Keep 60 seconds of history
        
        # Timer for monitoring
        update_rate = self.get_parameter('update_rate').value
        self.monitor_timer = self.create_timer(1.0 / update_rate, self.monitor_callback)
        
        # Initialize system monitoring
        self.start_time = time.time()
        
        self.get_logger().info("System Health Monitor Node initialized")
        self.get_logger().info(f"Monitoring {len(self.expected_nodes)} ROS nodes")
        self.get_logger().info(f"Update rate: {update_rate} Hz")
    
    # ===========================================
    # COMPONENT STATUS CALLBACKS
    # ===========================================
    
    def flight_status_callback(self, msg):
        """Handle flight controller status updates"""
        self.component_status['flight_controller']['status'] = msg.data
        self.component_status['flight_controller']['last_update'] = time.time()
    
    def battery_callback(self, msg):
        """Handle battery voltage updates"""
        self.component_status['battery']['voltage'] = msg.data
        self.component_status['battery']['last_update'] = time.time()
    
    def gps_callback(self, msg):
        """Handle GPS quality updates"""
        self.component_status['gps']['quality'] = msg.data
        self.component_status['gps']['last_update'] = time.time()
    
    def vision_callback(self, msg):
        """Handle vision system status updates"""
        self.component_status['vision']['status'] = msg.data
        self.component_status['vision']['last_update'] = time.time()
    
    def lidar_callback(self, msg):
        """Handle LiDAR system status updates"""
        self.component_status['lidar']['status'] = msg.data
        self.component_status['lidar']['last_update'] = time.time()
    
    def emergency_callback(self, msg):
        """Handle emergency system status updates"""
        self.component_status['emergency']['active'] = msg.data
        self.component_status['emergency']['last_update'] = time.time()
    
    # ===========================================
    # MONITORING FUNCTIONS
    # ===========================================
    
    def monitor_callback(self):
        """Main system health monitoring loop"""
        try:
            current_time = time.time()
            
            # Monitor hardware performance
            if self.get_parameter('enable_hardware_monitoring').value:
                self.monitor_hardware_performance()
            
            # Monitor ROS nodes
            if self.get_parameter('enable_ros_monitoring').value:
                self.monitor_ros_nodes()
            
            # Monitor component health
            self.monitor_component_health(current_time)
            
            # Analyze system health
            self.analyze_system_health()
            
            # Publish status
            self.publish_system_status()
            
        except Exception as e:
            self.get_logger().error(f"Error in monitor callback: {e}")
    
    def monitor_hardware_performance(self):
        """Monitor hardware performance metrics"""
        try:
            # CPU usage
            self.cpu_usage = psutil.cpu_percent(interval=0.1)
            
            # Memory usage
            memory = psutil.virtual_memory()
            self.memory_usage = memory.percent
            
            # Disk usage
            disk = psutil.disk_usage('/')
            self.disk_usage = disk.percent
            
            # System uptime
            self.system_uptime = time.time() - self.start_time
            
            # System temperature (Raspberry Pi)
            try:
                if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                    with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                        temp_millidegree = int(f.read().strip())
                        self.system_temperature = temp_millidegree / 1000.0
                else:
                    self.system_temperature = 0.0
            except:
                self.system_temperature = 0.0
            
            # Store performance history
            performance_sample = {
                'timestamp': time.time(),
                'cpu': self.cpu_usage,
                'memory': self.memory_usage,
                'disk': self.disk_usage,
                'temperature': self.system_temperature
            }
            
            self.performance_history.append(performance_sample)
            if len(self.performance_history) > self.max_history_size:
                self.performance_history.pop(0)
            
            # Check performance thresholds
            self.check_performance_thresholds()
            
        except Exception as e:
            self.get_logger().error(f"Error monitoring hardware performance: {e}")
    
    def monitor_ros_nodes(self):
        """Monitor ROS node health"""
        try:
            current_time = time.time()
            
            # Get list of active nodes (simplified check)
            # In a real implementation, you would use ROS introspection
            self.active_nodes = []
            self.failed_nodes = []
            
            # Check if expected nodes are responding
            # This is a simplified check - in practice you'd check node lifecycle
            for node_name in self.expected_nodes:
                # Simulate node health check
                # In reality, you'd check if topics are being published, services responding, etc.
                node_healthy = True  # Placeholder
                
                if node_healthy:
                    self.active_nodes.append(node_name)
                else:
                    self.failed_nodes.append(node_name)
            
            self.last_node_check = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error monitoring ROS nodes: {e}")
    
    def monitor_component_health(self, current_time):
        """Monitor health of critical system components"""
        try:
            node_timeout = self.get_parameter('node_timeout').value
            
            for component, status in self.component_status.items():
                last_update = status.get('last_update')
                
                if last_update is None:
                    self.add_system_warning(f"{component.upper()}_NO_DATA")
                elif current_time - last_update > node_timeout:
                    self.add_system_error(f"{component.upper()}_TIMEOUT")
                else:
                    self.remove_system_error(f"{component.upper()}_TIMEOUT")
                    self.remove_system_warning(f"{component.upper()}_NO_DATA")
                    
                    # Check component-specific health
                    self.check_component_specific_health(component, status)
        
        except Exception as e:
            self.get_logger().error(f"Error monitoring component health: {e}")
    
    def check_component_specific_health(self, component, status):
        """Check specific health conditions for each component"""
        try:
            if component == 'flight_controller':
                if 'ERROR' in status.get('status', ''):
                    self.add_system_error("FLIGHT_CONTROLLER_ERROR")
                elif 'WARNING' in status.get('status', ''):
                    self.add_system_warning("FLIGHT_CONTROLLER_WARNING")
                else:
                    self.remove_system_error("FLIGHT_CONTROLLER_ERROR")
                    self.remove_system_warning("FLIGHT_CONTROLLER_WARNING")
            
            elif component == 'battery':
                voltage = status.get('voltage', 0.0)
                if voltage > 0:
                    # Assume 4S LiPo
                    cell_voltage = voltage / 4.0
                    if cell_voltage < 3.1:
                        self.add_system_error("BATTERY_CRITICAL")
                    elif cell_voltage < 3.3:
                        self.add_system_warning("BATTERY_LOW")
                    else:
                        self.remove_system_error("BATTERY_CRITICAL")
                        self.remove_system_warning("BATTERY_LOW")
            
            elif component == 'gps':
                quality = status.get('quality', 'UNKNOWN')
                if quality in ['NO_GPS_DATA', 'NO_FIX']:
                    self.add_system_warning("GPS_NO_FIX")
                elif quality == 'POOR_ACCURACY':
                    self.add_system_warning("GPS_POOR_ACCURACY")
                else:
                    self.remove_system_warning("GPS_NO_FIX")
                    self.remove_system_warning("GPS_POOR_ACCURACY")
            
            elif component == 'emergency':
                if status.get('active', False):
                    self.add_system_error("EMERGENCY_ACTIVE")
                else:
                    self.remove_system_error("EMERGENCY_ACTIVE")
        
        except Exception as e:
            self.get_logger().error(f"Error checking {component} health: {e}")
    
    def check_performance_thresholds(self):
        """Check if performance metrics exceed warning thresholds"""
        try:
            cpu_threshold = self.get_parameter('cpu_warning_threshold').value
            memory_threshold = self.get_parameter('memory_warning_threshold').value
            disk_threshold = self.get_parameter('disk_warning_threshold').value
            temp_threshold = self.get_parameter('temperature_warning_threshold').value
            
            # CPU usage check
            if self.cpu_usage > cpu_threshold:
                self.add_performance_issue(f"CPU_HIGH_{self.cpu_usage:.1f}%")
            else:
                self.remove_performance_issue("CPU_HIGH")
            
            # Memory usage check
            if self.memory_usage > memory_threshold:
                self.add_performance_issue(f"MEMORY_HIGH_{self.memory_usage:.1f}%")
            else:
                self.remove_performance_issue("MEMORY_HIGH")
            
            # Disk usage check
            if self.disk_usage > disk_threshold:
                self.add_performance_issue(f"DISK_HIGH_{self.disk_usage:.1f}%")
            else:
                self.remove_performance_issue("DISK_HIGH")
            
            # Temperature check
            if self.system_temperature > temp_threshold:
                self.add_performance_issue(f"TEMPERATURE_HIGH_{self.system_temperature:.1f}°C")
            else:
                self.remove_performance_issue("TEMPERATURE_HIGH")
        
        except Exception as e:
            self.get_logger().error(f"Error checking performance thresholds: {e}")
    
    def analyze_system_health(self):
        """Analyze overall system health"""
        try:
            # Clear previous analysis
            overall_health = "HEALTHY"
            
            # Check for critical errors
            if len(self.system_errors) > 0:
                overall_health = "ERROR"
            elif len(self.system_warnings) > 0:
                overall_health = "WARNING"
            elif len(self.system_performance_issues) > 0:
                overall_health = "DEGRADED"
            
            # Additional checks
            if len(self.failed_nodes) > 0:
                overall_health = "ERROR" if overall_health == "HEALTHY" else overall_health
            
            # Store overall health assessment
            self.overall_health = overall_health
            
        except Exception as e:
            self.get_logger().error(f"Error analyzing system health: {e}")
            self.overall_health = "ERROR"
    
    # ===========================================
    # HELPER FUNCTIONS
    # ===========================================
    
    def add_system_error(self, error_code):
        """Add system error"""
        if error_code not in self.system_errors:
            self.system_errors.append(error_code)
            self.get_logger().error(f"System error: {error_code}")
    
    def remove_system_error(self, error_code):
        """Remove system error"""
        self.system_errors = [e for e in self.system_errors if not e.startswith(error_code)]
    
    def add_system_warning(self, warning_code):
        """Add system warning"""
        if warning_code not in self.system_warnings:
            self.system_warnings.append(warning_code)
            self.get_logger().warn(f"System warning: {warning_code}")
    
    def remove_system_warning(self, warning_code):
        """Remove system warning"""
        self.system_warnings = [w for w in self.system_warnings if not w.startswith(warning_code)]
    
    def add_performance_issue(self, issue_code):
        """Add performance issue"""
        # Remove old instances of the same issue type
        issue_type = issue_code.split('_')[0] + '_' + issue_code.split('_')[1]
        self.system_performance_issues = [
            p for p in self.system_performance_issues 
            if not p.startswith(issue_type)
        ]
        
        if issue_code not in self.system_performance_issues:
            self.system_performance_issues.append(issue_code)
            self.get_logger().warn(f"Performance issue: {issue_code}")
    
    def remove_performance_issue(self, issue_type):
        """Remove performance issue"""
        self.system_performance_issues = [
            p for p in self.system_performance_issues 
            if not p.startswith(issue_type)
        ]
    
    # ===========================================
    # PUBLISHERS
    # ===========================================
    
    def publish_system_status(self):
        """Publish all system status information"""
        try:
            # Overall system health
            self.system_health_pub.publish(String(data=self.overall_health))
            
            # System errors
            errors_str = ", ".join(self.system_errors) if self.system_errors else "NONE"
            self.system_errors_pub.publish(String(data=errors_str))
            
            # System warnings
            warnings_str = ", ".join(self.system_warnings) if self.system_warnings else "NONE"
            self.system_warnings_pub.publish(String(data=warnings_str))
            
            # Performance issues
            performance_str = ", ".join(self.system_performance_issues) if self.system_performance_issues else "GOOD"
            self.system_performance_pub.publish(String(data=performance_str))
            
            # Hardware metrics
            self.cpu_usage_pub.publish(Float32(data=self.cpu_usage))
            self.memory_usage_pub.publish(Float32(data=self.memory_usage))
            self.disk_usage_pub.publish(Float32(data=self.disk_usage))
            self.temperature_pub.publish(Float32(data=self.system_temperature))
            self.uptime_pub.publish(Float32(data=self.system_uptime))
            
            # ROS nodes status
            nodes_status = f"Active: {len(self.active_nodes)}, Failed: {len(self.failed_nodes)}"
            self.nodes_status_pub.publish(String(data=nodes_status))
            self.active_nodes_count_pub.publish(Int32(data=len(self.active_nodes)))
            
            failed_nodes_str = ", ".join(self.failed_nodes) if self.failed_nodes else "NONE"
            self.failed_nodes_pub.publish(String(data=failed_nodes_str))
            
        except Exception as e:
            self.get_logger().error(f"Error publishing system status: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        system_health_monitor = SystemHealthMonitorNode()
        rclpy.spin(system_health_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'system_health_monitor' in locals():
            system_health_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
