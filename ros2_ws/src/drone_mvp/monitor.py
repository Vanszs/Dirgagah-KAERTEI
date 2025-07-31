#!/usr/bin/env python3

"""
Real-time System Monitor for KAERTEI 2025 Drone System
Monitors all critical components and displays status in terminal
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import curses
import threading
import time
from datetime import datetime
import json

# ROS2 Messages
from std_msgs.msg import String, Bool, Int32, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, Range, Image
from mavros_msgs.msg import State
from drone_mvp.msg import Detection, MissionStatus

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Data storage
        self.data = {
            'mavros_state': {'connected': False, 'armed': False, 'mode': 'UNKNOWN'},
            'gps': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0, 'status': 0},
            'mission': {'state': 'INIT', 'phase': 'PREPARING', 'progress': 0},
            'vision': {'detections': 0, 'last_detection': 'None', 'confidence': 0.0},
            'sensors': {
                'tof_left': {'distance': 0.0, 'valid': False},
                'tof_right': {'distance': 0.0, 'valid': False}, 
                'tof_front': {'distance': 0.0, 'valid': False}
            },
            'magnets': {'magnet1': False, 'magnet2': False},
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'timestamps': {}
        }
        
        # Locks for thread safety
        self.data_lock = threading.Lock()
        
        # Subscribers
        self.create_subscriptions()
        
        # Update timestamps
        for key in self.data.keys():
            self.data['timestamps'][key] = time.time()
        
        self.get_logger().info("System Monitor initialized")
    
    def create_subscriptions(self):
        # MAVROS State
        self.mavros_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_callback, self.sensor_qos)
        
        # GPS
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, self.sensor_qos)
        
        # Mission status
        self.mission_sub = self.create_subscription(
            String, '/mission/state', self.mission_callback, 10)
        
        # Vision detections
        self.vision_sub = self.create_subscription(
            Detection, '/vision/detection', self.vision_callback, 10)
        
        # ToF sensors
        self.tof_left_sub = self.create_subscription(
            Range, '/sensors/tof_left', self.tof_left_callback, self.sensor_qos)
        self.tof_right_sub = self.create_subscription(
            Range, '/sensors/tof_right', self.tof_right_callback, self.sensor_qos)
        self.tof_front_sub = self.create_subscription(
            Range, '/sensors/tof_front', self.tof_front_callback, self.sensor_qos)
        
        # Magnet status
        self.magnet1_sub = self.create_subscription(
            Bool, '/magnets/magnet1/status', self.magnet1_callback, 10)
        self.magnet2_sub = self.create_subscription(
            Bool, '/magnets/magnet2/status', self.magnet2_callback, 10)
        
        # Position
        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, self.sensor_qos)
        
        # Velocity
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.velocity_callback, self.sensor_qos)
    
    def mavros_callback(self, msg):
        with self.data_lock:
            self.data['mavros_state'] = {
                'connected': msg.connected,
                'armed': msg.armed,
                'mode': msg.mode
            }
            self.data['timestamps']['mavros_state'] = time.time()
    
    def gps_callback(self, msg):
        with self.data_lock:
            self.data['gps'] = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude,
                'status': msg.status.status
            }
            self.data['timestamps']['gps'] = time.time()
    
    def mission_callback(self, msg):
        with self.data_lock:
            try:
                mission_data = json.loads(msg.data) if '{' in msg.data else {'state': msg.data}
                self.data['mission'] = mission_data
            except:
                self.data['mission']['state'] = msg.data
            self.data['timestamps']['mission'] = time.time()
    
    def vision_callback(self, msg):
        with self.data_lock:
            self.data['vision'] = {
                'detections': self.data['vision'].get('detections', 0) + 1,
                'last_detection': msg.class_name,
                'confidence': msg.confidence
            }
            self.data['timestamps']['vision'] = time.time()
    
    def tof_left_callback(self, msg):
        with self.data_lock:
            self.data['sensors']['tof_left'] = {
                'distance': msg.range,
                'valid': msg.range >= msg.min_range and msg.range <= msg.max_range
            }
            self.data['timestamps']['sensors'] = time.time()
    
    def tof_right_callback(self, msg):
        with self.data_lock:
            self.data['sensors']['tof_right'] = {
                'distance': msg.range,
                'valid': msg.range >= msg.min_range and msg.range <= msg.max_range
            }
            self.data['timestamps']['sensors'] = time.time()
    
    def tof_front_callback(self, msg):
        with self.data_lock:
            self.data['sensors']['tof_front'] = {
                'distance': msg.range,
                'valid': msg.range >= msg.min_range and msg.range <= msg.max_range
            }
            self.data['timestamps']['sensors'] = time.time()
    
    def magnet1_callback(self, msg):
        with self.data_lock:
            self.data['magnets']['magnet1'] = msg.data
            self.data['timestamps']['magnets'] = time.time()
    
    def magnet2_callback(self, msg):
        with self.data_lock:
            self.data['magnets']['magnet2'] = msg.data
            self.data['timestamps']['magnets'] = time.time()
    
    def position_callback(self, msg):
        with self.data_lock:
            self.data['position'] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            }
            self.data['timestamps']['position'] = time.time()
    
    def velocity_callback(self, msg):
        with self.data_lock:
            self.data['velocity'] = {
                'vx': msg.twist.linear.x,
                'vy': msg.twist.linear.y,
                'vz': msg.twist.linear.z
            }
            self.data['timestamps']['velocity'] = time.time()
    
    def get_data_copy(self):
        with self.data_lock:
            return self.data.copy()

class MonitorDisplay:
    def __init__(self, monitor_node):
        self.monitor = monitor_node
        self.stdscr = None
        
    def init_display(self, stdscr):
        self.stdscr = stdscr
        curses.curs_set(0)  # Hide cursor
        self.stdscr.nodelay(1)  # Non-blocking input
        
        # Colors
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)  # Good
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)    # Bad
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK) # Warning
        curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)   # Info
        curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_BLACK)  # Normal
        
        # Main loop
        while True:
            try:
                self.update_display()
                time.sleep(0.1)
                
                # Check for exit key
                key = self.stdscr.getch()
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.stdscr.addstr(0, 0, f"Error: {e}")
                self.stdscr.refresh()
    
    def update_display(self):
        self.stdscr.clear()
        data = self.monitor.get_data_copy()
        current_time = time.time()
        
        # Header
        header = f"KAERTEI 2025 Drone System Monitor - {datetime.now().strftime('%H:%M:%S')}"
        self.stdscr.addstr(0, 0, "=" * 80, curses.color_pair(4))
        self.stdscr.addstr(1, 0, header.center(80), curses.color_pair(4))
        self.stdscr.addstr(2, 0, "=" * 80, curses.color_pair(4))
        
        row = 4
        
        # MAVROS Status
        mavros = data['mavros_state']
        age = current_time - data['timestamps'].get('mavros_state', 0)
        color = 1 if mavros['connected'] and age < 2 else 2
        
        self.stdscr.addstr(row, 0, "MAVROS STATUS:", curses.color_pair(5) | curses.A_BOLD)
        row += 1
        self.stdscr.addstr(row, 2, f"Connected: {'YES' if mavros['connected'] else 'NO'}", curses.color_pair(color))
        row += 1
        self.stdscr.addstr(row, 2, f"Armed: {'YES' if mavros['armed'] else 'NO'}", curses.color_pair(1 if mavros['armed'] else 3))
        row += 1
        self.stdscr.addstr(row, 2, f"Mode: {mavros['mode']}", curses.color_pair(1 if mavros['mode'] != 'UNKNOWN' else 2))
        row += 1
        self.stdscr.addstr(row, 2, f"Data age: {age:.1f}s", curses.color_pair(1 if age < 2 else 2))
        row += 2
        
        # GPS Status
        gps = data['gps']
        gps_age = current_time - data['timestamps'].get('gps', 0)
        gps_color = 1 if gps['status'] >= 2 and gps_age < 2 else 2
        
        self.stdscr.addstr(row, 0, "GPS STATUS:", curses.color_pair(5) | curses.A_BOLD)
        row += 1
        self.stdscr.addstr(row, 2, f"Position: {gps['lat']:.6f}, {gps['lon']:.6f}", curses.color_pair(gps_color))
        row += 1
        self.stdscr.addstr(row, 2, f"Altitude: {gps['alt']:.2f}m", curses.color_pair(gps_color))
        row += 1
        self.stdscr.addstr(row, 2, f"Fix Status: {gps['status']} ({'Good' if gps['status'] >= 2 else 'Poor'})", curses.color_pair(gps_color))
        row += 1
        self.stdscr.addstr(row, 2, f"Data age: {gps_age:.1f}s", curses.color_pair(1 if gps_age < 2 else 2))
        row += 2
        
        # Mission Status
        mission = data['mission']
        mission_age = current_time - data['timestamps'].get('mission', 0)
        
        self.stdscr.addstr(row, 0, "MISSION STATUS:", curses.color_pair(5) | curses.A_BOLD)
        row += 1
        self.stdscr.addstr(row, 2, f"State: {mission.get('state', 'UNKNOWN')}", curses.color_pair(1))
        row += 1
        self.stdscr.addstr(row, 2, f"Phase: {mission.get('phase', 'UNKNOWN')}", curses.color_pair(1))
        row += 1
        self.stdscr.addstr(row, 2, f"Progress: {mission.get('progress', 0)}%", curses.color_pair(1))
        row += 1
        self.stdscr.addstr(row, 2, f"Data age: {mission_age:.1f}s", curses.color_pair(1 if mission_age < 5 else 2))
        row += 2
        
        # Vision Status
        vision = data['vision']
        vision_age = current_time - data['timestamps'].get('vision', 0)
        
        self.stdscr.addstr(row, 0, "VISION STATUS:", curses.color_pair(5) | curses.A_BOLD)
        row += 1
        self.stdscr.addstr(row, 2, f"Total Detections: {vision['detections']}", curses.color_pair(1))
        row += 1
        self.stdscr.addstr(row, 2, f"Last Detection: {vision['last_detection']}", curses.color_pair(1))
        row += 1
        self.stdscr.addstr(row, 2, f"Confidence: {vision['confidence']:.2f}", curses.color_pair(1 if vision['confidence'] > 0.5 else 3))
        row += 1
        self.stdscr.addstr(row, 2, f"Data age: {vision_age:.1f}s", curses.color_pair(1 if vision_age < 10 else 2))
        row += 2
        
        # Sensors Status (split into two columns)
        sensors = data['sensors']
        sensor_age = current_time - data['timestamps'].get('sensors', 0)
        
        self.stdscr.addstr(row, 0, "SENSORS STATUS:", curses.color_pair(5) | curses.A_BOLD)
        self.stdscr.addstr(row, 40, "POSITION & VELOCITY:", curses.color_pair(5) | curses.A_BOLD)
        row += 1
        
        # ToF sensors
        for i, (sensor_name, sensor_data) in enumerate(sensors.items()):
            color = 1 if sensor_data['valid'] else 2
            self.stdscr.addstr(row + i, 2, f"{sensor_name}: {sensor_data['distance']:.2f}m", curses.color_pair(color))
        
        # Position and velocity
        pos = data['position']
        vel = data['velocity']
        pos_age = current_time - data['timestamps'].get('position', 0)
        vel_age = current_time - data['timestamps'].get('velocity', 0)
        
        self.stdscr.addstr(row, 42, f"Position: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})", curses.color_pair(1))
        self.stdscr.addstr(row + 1, 42, f"Velocity: ({vel['vx']:.2f}, {vel['vy']:.2f}, {vel['vz']:.2f})", curses.color_pair(1))
        self.stdscr.addstr(row + 2, 42, f"Pos age: {pos_age:.1f}s, Vel age: {vel_age:.1f}s", curses.color_pair(1 if max(pos_age, vel_age) < 2 else 2))
        
        row += 4
        self.stdscr.addstr(row, 2, f"Sensor data age: {sensor_age:.1f}s", curses.color_pair(1 if sensor_age < 2 else 2))
        row += 2
        
        # Magnet Status
        magnets = data['magnets']
        magnet_age = current_time - data['timestamps'].get('magnets', 0)
        
        self.stdscr.addstr(row, 0, "MAGNET STATUS:", curses.color_pair(5) | curses.A_BOLD)
        row += 1
        self.stdscr.addstr(row, 2, f"Magnet 1: {'ON' if magnets['magnet1'] else 'OFF'}", curses.color_pair(3 if magnets['magnet1'] else 1))
        row += 1
        self.stdscr.addstr(row, 2, f"Magnet 2: {'ON' if magnets['magnet2'] else 'OFF'}", curses.color_pair(3 if magnets['magnet2'] else 1))
        row += 1
        self.stdscr.addstr(row, 2, f"Data age: {magnet_age:.1f}s", curses.color_pair(1 if magnet_age < 5 else 2))
        row += 2
        
        # Footer
        footer_row = self.stdscr.getmaxyx()[0] - 2
        self.stdscr.addstr(footer_row, 0, "=" * 80, curses.color_pair(4))
        self.stdscr.addstr(footer_row + 1, 0, "Press 'q' or ESC to quit".center(80), curses.color_pair(4))
        
        self.stdscr.refresh()

def main():
    rclpy.init()
    
    monitor_node = SystemMonitor()
    display = MonitorDisplay(monitor_node)
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(monitor_node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Start curses display
        curses.wrapper(display.init_display)
    except KeyboardInterrupt:
        pass
    finally:
        monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
