#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool, Float32, Int32
from kaertei_drone.hardware.hardware_config import HardwareConfig


class FlightStateMonitorNode(Node):
    def __init__(self):
        super().__init__('flight_state_monitor')
        
        # Load hardware config
        self.hw_config = HardwareConfig()
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 2.0),           # Hz
                ('connection_timeout', 5.0),    # seconds
                ('battery_warning_voltage', 3.3), # per cell
                ('battery_critical_voltage', 3.1), # per cell
                ('gps_fix_timeout', 10.0),      # seconds
                ('altitude_warning_threshold', 50.0), # meters
            ]
        )
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers - Flight State Status
        self.flight_status_pub = self.create_publisher(String, '/flight_controller/status', qos_profile)
        self.connection_status_pub = self.create_publisher(Bool, '/flight_controller/connected', qos_profile)
        self.armed_status_pub = self.create_publisher(Bool, '/flight_controller/armed', qos_profile)
        self.mode_pub = self.create_publisher(String, '/flight_controller/mode', qos_profile)
        self.battery_percentage_pub = self.create_publisher(Float32, '/flight_controller/battery_percent', qos_profile)
        self.battery_voltage_pub = self.create_publisher(Float32, '/flight_controller/battery_voltage', qos_profile)
        self.gps_satellites_pub = self.create_publisher(Int32, '/flight_controller/gps_satellites', qos_profile)
        self.altitude_pub = self.create_publisher(Float32, '/flight_controller/altitude', qos_profile)
        self.system_health_pub = self.create_publisher(String, '/flight_controller/health_status', qos_profile)
        
        # Subscribers - MAVROS topics
        self.mavros_state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, qos_profile)
        self.extended_state_sub = self.create_subscription(
            ExtendedState, '/mavros/extended_state', self.extended_state_callback, qos_profile)
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, qos_profile)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        self.local_position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_position_callback, qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos_profile)
        
        # Flight controller state
        self.mavros_connected = False
        self.flight_controller_connected = False
        self.armed = False
        self.flight_mode = "UNKNOWN"
        self.landed_state = "UNKNOWN"
        self.last_heartbeat = None
        
        # Battery state
        self.battery_voltage = 0.0
        self.battery_percentage = 0.0
        self.battery_remaining = 0.0
        self.battery_status = "UNKNOWN"
        
        # GPS state
        self.gps_fix_type = 0
        self.satellites_visible = 0
        self.gps_last_update = None
        self.has_gps_fix = False
        
        # Position state
        self.current_altitude = 0.0
        self.local_position = None
        
        # IMU state
        self.imu_last_update = None
        self.has_imu_data = False
        
        # System health
        self.system_errors = []
        self.system_warnings = []
        
        # Timer for monitoring and publishing
        update_rate = self.get_parameter('update_rate').value
        self.monitor_timer = self.create_timer(1.0 / update_rate, self.monitor_callback)
        
        self.get_logger().info("Flight State Monitor Node initialized")
        self.get_logger().info(f"Monitoring Pixhawk4 via MAVROS at {update_rate} Hz")
    
    def mavros_state_callback(self, msg):
        """Handle MAVROS state updates"""
        try:
            self.mavros_connected = msg.connected
            self.flight_controller_connected = msg.connected
            self.armed = msg.armed
            self.flight_mode = msg.mode
            self.last_heartbeat = time.time()
            
            if not msg.connected:
                self.add_system_error("MAVROS_DISCONNECTED")
            else:
                self.remove_system_error("MAVROS_DISCONNECTED")
                
        except Exception as e:
            self.get_logger().error(f"Error processing MAVROS state: {e}")
    
    def extended_state_callback(self, msg):
        """Handle MAVROS extended state updates"""
        try:
            # Convert landed state enum to string
            if msg.landed_state == ExtendedState.LANDED_STATE_UNDEFINED:
                self.landed_state = "UNDEFINED"
            elif msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                self.landed_state = "ON_GROUND"
            elif msg.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
                self.landed_state = "IN_AIR"
            elif msg.landed_state == ExtendedState.LANDED_STATE_TAKEOFF:
                self.landed_state = "TAKEOFF"
            elif msg.landed_state == ExtendedState.LANDED_STATE_LANDING:
                self.landed_state = "LANDING"
            else:
                self.landed_state = "UNKNOWN"
                
        except Exception as e:
            self.get_logger().error(f"Error processing extended state: {e}")
    
    def battery_callback(self, msg):
        """Handle battery status updates"""
        try:
            self.battery_voltage = msg.voltage
            self.battery_percentage = msg.percentage * 100.0  # Convert to percentage
            # For ROS 2, BatteryState doesn't have remaining field
            self.battery_remaining = msg.percentage
            
            # Check battery warnings
            warning_voltage = self.get_parameter('battery_warning_voltage').value
            critical_voltage = self.get_parameter('battery_critical_voltage').value
            
            # Assume 4S LiPo (14.8V nominal) for now
            cell_voltage = self.battery_voltage / 4.0 if self.battery_voltage > 0 else 0.0
            
            if cell_voltage < critical_voltage:
                self.battery_status = "CRITICAL"
                self.add_system_error("BATTERY_CRITICAL")
            elif cell_voltage < warning_voltage:
                self.battery_status = "WARNING"
                self.add_system_warning("BATTERY_LOW")
                self.remove_system_error("BATTERY_CRITICAL")
            else:
                self.battery_status = "GOOD"
                self.remove_system_warning("BATTERY_LOW")
                self.remove_system_error("BATTERY_CRITICAL")
                
        except Exception as e:
            self.get_logger().error(f"Error processing battery status: {e}")
    
    def gps_callback(self, msg):
        """Handle GPS position updates"""
        try:
            self.gps_fix_type = msg.status.status
            # Get satellite count from status if available
            if hasattr(msg.status, 'satellites_visible'):
                self.satellites_visible = msg.status.satellites_visible
            
            self.gps_last_update = time.time()
            
            # Check GPS fix quality
            if msg.status.status >= 2:  # GPS fix available
                self.has_gps_fix = True
                self.remove_system_warning("GPS_NO_FIX")
            else:
                self.has_gps_fix = False
                self.add_system_warning("GPS_NO_FIX")
                
        except Exception as e:
            self.get_logger().error(f"Error processing GPS data: {e}")
    
    def local_position_callback(self, msg):
        """Handle local position updates"""
        try:
            self.local_position = msg
            self.current_altitude = msg.pose.position.z
            
            # Check altitude warnings
            altitude_warning = self.get_parameter('altitude_warning_threshold').value
            if self.current_altitude > altitude_warning:
                self.add_system_warning("HIGH_ALTITUDE")
            else:
                self.remove_system_warning("HIGH_ALTITUDE")
                
        except Exception as e:
            self.get_logger().error(f"Error processing local position: {e}")
    
    def imu_callback(self, msg):
        """Handle IMU data updates"""
        try:
            self.imu_last_update = time.time()
            self.has_imu_data = True
            
        except Exception as e:
            self.get_logger().error(f"Error processing IMU data: {e}")
    
    def monitor_callback(self):
        """Main monitoring loop"""
        try:
            current_time = time.time()
            
            # Check connection timeout
            connection_timeout = self.get_parameter('connection_timeout').value
            if (self.last_heartbeat is None or 
                current_time - self.last_heartbeat > connection_timeout):
                self.flight_controller_connected = False
                self.add_system_error("FC_CONNECTION_TIMEOUT")
            else:
                self.remove_system_error("FC_CONNECTION_TIMEOUT")
            
            # Check GPS timeout
            gps_timeout = self.get_parameter('gps_fix_timeout').value
            if (self.gps_last_update is None or 
                current_time - self.gps_last_update > gps_timeout):
                self.add_system_warning("GPS_TIMEOUT")
            else:
                self.remove_system_warning("GPS_TIMEOUT")
            
            # Check IMU timeout
            if (self.imu_last_update is None or 
                current_time - self.imu_last_update > 5.0):
                self.add_system_warning("IMU_TIMEOUT")
            else:
                self.remove_system_warning("IMU_TIMEOUT")
            
            # Publish all status information
            self.publish_flight_status()
            self.publish_system_health()
            
        except Exception as e:
            self.get_logger().error(f"Error in monitor callback: {e}")
    
    def publish_flight_status(self):
        """Publish flight controller status"""
        try:
            # Overall flight status
            if not self.flight_controller_connected:
                status = "DISCONNECTED"
            elif len(self.system_errors) > 0:
                status = "ERROR"
            elif len(self.system_warnings) > 0:
                status = "WARNING"
            elif self.armed:
                status = f"ARMED_{self.landed_state}"
            else:
                status = f"DISARMED_{self.landed_state}"
            
            # Publish individual status topics
            self.flight_status_pub.publish(String(data=status))
            self.connection_status_pub.publish(Bool(data=self.flight_controller_connected))
            self.armed_status_pub.publish(Bool(data=self.armed))
            self.mode_pub.publish(String(data=self.flight_mode))
            self.battery_percentage_pub.publish(Float32(data=self.battery_percentage))
            self.battery_voltage_pub.publish(Float32(data=self.battery_voltage))
            self.gps_satellites_pub.publish(Int32(data=self.satellites_visible))
            self.altitude_pub.publish(Float32(data=self.current_altitude))
            
        except Exception as e:
            self.get_logger().error(f"Error publishing flight status: {e}")
    
    def publish_system_health(self):
        """Publish system health status"""
        try:
            if len(self.system_errors) > 0:
                health_status = f"ERROR: {', '.join(self.system_errors)}"
            elif len(self.system_warnings) > 0:
                health_status = f"WARNING: {', '.join(self.system_warnings)}"
            else:
                health_status = "HEALTHY"
            
            self.system_health_pub.publish(String(data=health_status))
            
        except Exception as e:
            self.get_logger().error(f"Error publishing system health: {e}")
    
    def add_system_error(self, error_code):
        """Add system error"""
        if error_code not in self.system_errors:
            self.system_errors.append(error_code)
            self.get_logger().error(f"System error: {error_code}")
    
    def remove_system_error(self, error_code):
        """Remove system error"""
        if error_code in self.system_errors:
            self.system_errors.remove(error_code)
            self.get_logger().info(f"System error resolved: {error_code}")
    
    def add_system_warning(self, warning_code):
        """Add system warning"""
        if warning_code not in self.system_warnings:
            self.system_warnings.append(warning_code)
            self.get_logger().warn(f"System warning: {warning_code}")
    
    def remove_system_warning(self, warning_code):
        """Remove system warning"""
        if warning_code in self.system_warnings:
            self.system_warnings.remove(warning_code)
            self.get_logger().info(f"System warning resolved: {warning_code}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        flight_state_monitor = FlightStateMonitorNode()
        rclpy.spin(flight_state_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'flight_state_monitor' in locals():
            flight_state_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
