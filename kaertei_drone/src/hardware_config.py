#!/usr/bin/env python3

import configparser
import os
from pathlib import Path

class HardwareConfig:
    """
    Centralized hardware configuration manager for KAERTEI 2025 FAIO Drone
    
    Usage:
        config = HardwareConfig()
        port = config.get_px4_port()
        front_magnet_pin = config.get_front_magnet_pin()
    """
    
    def __init__(self, config_file=None):
        self.config = configparser.ConfigParser()
        
        # Default config file path
        if config_file is None:
            current_dir = Path(__file__).parent.parent
            config_file = current_dir / "config" / "hardware_config.conf"
        
        # Load configuration
        if os.path.exists(config_file):
            self.config.read(config_file)
            print(f"✅ Loaded hardware config from: {config_file}")
        else:
            print(f"❌ Config file not found: {config_file}")
            print("⚠️  Using default values")
            self._load_defaults()
    
    def _load_defaults(self):
        """Load default configuration if file not found"""
        self.config.add_section('flight_controller')
        self.config.set('flight_controller', 'connection_port', '/dev/ttyUSB0')  # sesuaikan ini
        self.config.set('flight_controller', 'baud_rate', '57600')  # sesuaikan ini jika perlu ganti ke 115200
        
        self.config.add_section('cameras')
        self.config.set('cameras', 'front_camera_index', '0')  # sesuaikan ini
        self.config.set('cameras', 'back_camera_index', '2')   # sesuaikan ini 
        self.config.set('cameras', 'top_camera_index', '4')    # sesuaikan ini
        
        self.config.add_section('gpio_pins')
        self.config.set('gpio_pins', 'front_magnet_pin', '18')  # sesuaikan ini
        self.config.set('gpio_pins', 'back_magnet_pin', '19')   # sesuaikan ini
    
    # ===========================================
    # PX4 Flight Controller Configuration
    # ===========================================
    
    def get_px4_port(self):
        """Get PX4 connection port"""
        # sesuaikan ini - WAJIB CEK: ls /dev/tty{USB,ACM}* sebelum kompetisi
        # Pixhawk4 biasanya: /dev/ttyACM0 atau /dev/ttyUSB0
        return self.config.get('flight_controller', 'connection_port', fallback='/dev/ttyUSB0')
    
    def get_px4_baud_rate(self):
        """Get PX4 baud rate"""
        # sesuaikan ini - jika Pixhawk4 tidak konek, coba ganti ke 115200
        return self.config.getint('flight_controller', 'baud_rate', fallback=57600)
    
    def get_px4_connection_string(self):
        """Get complete PX4 connection string for MAVLink"""
        port = self.get_px4_port()
        baud = self.get_px4_baud_rate()
        return f"{port}:{baud}"
    
    def get_connection_timeout(self):
        """Get connection timeout in seconds"""
        return self.config.getint('flight_controller', 'connection_timeout', fallback=10)
    
    # ===========================================
    # Camera Configuration
    # ===========================================
    
    def get_front_camera_index(self):
        """Get front camera device index"""
        # sesuaikan ini - WAJIB CEK: v4l2-ctl --list-devices sebelum kompetisi
        # USB camera biasanya: 0, 2, 4 (tergantung urutan colok)
        return self.config.getint('cameras', 'front_camera_index', fallback=0)
    
    def get_back_camera_index(self):
        """Get back camera device index"""
        # sesuaikan ini - WAJIB CEK: v4l2-ctl --list-devices sebelum kompetisi
        # Pastikan berbeda dengan front camera
        return self.config.getint('cameras', 'back_camera_index', fallback=2)
    
    def get_top_camera_index(self):
        """Get top camera device index"""
        # sesuaikan ini - WAJIB CEK: v4l2-ctl --list-devices sebelum kompetisi
        # Pastikan berbeda dengan front dan back camera
        return self.config.getint('cameras', 'top_camera_index', fallback=4)
    
    def get_camera_resolution(self):
        """Get camera resolution as (width, height)"""
        width = self.config.getint('cameras', 'camera_width', fallback=640)
        height = self.config.getint('cameras', 'camera_height', fallback=480)
        return (width, height)
    
    def get_camera_fps(self):
        """Get camera frame rate"""
        return self.config.getint('cameras', 'camera_fps', fallback=30)
    
    def get_camera_offset(self, camera_name):
        """Get camera mounting offset in pixels"""
        x_offset = self.config.getint('cameras', f'{camera_name}_camera_offset_x', fallback=0)
        y_offset = self.config.getint('cameras', f'{camera_name}_camera_offset_y', fallback=0)
        return (x_offset, y_offset)
    
    # ===========================================
    # GPIO Pin Configuration (Raspberry Pi 5)
    # ===========================================
    
    def get_front_magnet_pin(self):
        """Get front electromagnet relay GPIO pin"""
        # sesuaikan ini - WAJIB PENTING: pastikan GPIO pin benar di Pi 5!
        # GPIO 18 = Pin 12, pastikan tidak konflik dengan sistem lain
        return self.config.getint('gpio_pins', 'front_magnet_relay_pin', fallback=18)
    
    def get_back_magnet_pin(self):
        """Get back electromagnet relay GPIO pin"""
        # sesuaikan ini - WAJIB PENTING: pastikan GPIO pin benar di Pi 5!
        # GPIO 19 = Pin 35, pastikan tidak konflik dengan sistem lain
        return self.config.getint('gpio_pins', 'back_magnet_relay_pin', fallback=19)
    
    def get_relay_pins(self):
        """Get both relay GPIO pins for electromagnets"""
        return {
            'front_magnet': self.get_front_magnet_pin(),
            'back_magnet': self.get_back_magnet_pin()
        }
    
    def get_status_led_pin(self):
        """Get status LED GPIO pin"""
        return self.config.getint('gpio_pins', 'status_led_pin', fallback=21)
    
    def get_error_led_pin(self):
        """Get error LED GPIO pin"""
        return self.config.getint('gpio_pins', 'error_led_pin', fallback=20)
    
    def get_emergency_stop_pin(self):
        """Get emergency stop button GPIO pin"""
        return self.config.getint('gpio_pins', 'emergency_stop_pin', fallback=16)
    
    def is_relay_active_high(self):
        """Check if relay is active HIGH or LOW"""
        return self.config.getboolean('gpio_pins', 'relay_active_high', fallback=False)  # Most relays are active LOW
    
    # ===========================================
    # Sensor Configuration (Raspberry Pi 5)
    # ===========================================
    
    def get_lidar_interfaces(self):
        """Get LiDAR sensor interfaces for TF Mini Plus"""
        # sesuaikan ini - WAJIB CEK: ls /dev/ttyUSB* sebelum kompetisi
        # 3x TF Mini Plus via USB serial, port bisa berubah saat cabut-colok!
        return {
            'front': self.config.get('sensors', 'lidar_front_interface', fallback='/dev/ttyUSB1'),
            'left': self.config.get('sensors', 'lidar_left_interface', fallback='/dev/ttyUSB2'), 
            'right': self.config.get('sensors', 'lidar_right_interface', fallback='/dev/ttyUSB3')
        }
    
    def get_lidar_baud_rate(self):
        """Get LiDAR serial communication baud rate"""
        return self.config.getint('sensors', 'lidar_baud_rate', fallback=115200)
    
    def get_lidar_max_range(self):
        """Get LiDAR sensor maximum range in meters"""
        return self.config.getfloat('sensors', 'lidar_max_range', fallback=12.0)
    
    def get_lidar_detection_threshold(self):
        """Get LiDAR obstacle detection threshold in meters"""
        return self.config.getfloat('sensors', 'lidar_detection_threshold', fallback=2.0)
    
    def get_lidar_sample_rate(self):
        """Get LiDAR sampling rate in Hz"""
        return self.config.getint('sensors', 'lidar_sample_rate', fallback=100)
    
    def get_tof_addresses(self):
        """DEPRECATED: Using LiDAR instead of ToF sensors"""
        return self.get_lidar_interfaces()
    
    def get_tof_max_range(self):
        """DEPRECATED: Using LiDAR instead of ToF sensors"""
        return self.get_lidar_max_range()
    
    def get_tof_detection_threshold(self):
        """DEPRECATED: Using LiDAR instead of ToF sensors"""
        return self.get_lidar_detection_threshold()
    
    def get_i2c_bus(self):
        """Get I2C bus number"""
        return self.config.getint('sensors', 'i2c_bus', fallback=1)
    
    # ===========================================
    # Flight Parameters
    # ===========================================
    
    def get_takeoff_altitude(self):
        """Get takeoff altitude in meters"""
        return self.config.getfloat('flight', 'takeoff_altitude', fallback=1.0)
    
    # Mission Parameters
    def get_mission_debug_mode(self):
        """Get debug mode setting"""
        return self.config.getboolean('MISSION', 'debug_mode', fallback=True)
    
    def get_checkpoint_timeout(self):
        """Get checkpoint timeout in seconds"""
        return self.config.getint('MISSION', 'checkpoint_timeout', fallback=300)
    
    def get_indoor_altitude(self):
        """Get indoor cruise altitude in meters"""
        return self.config.getfloat('MISSION', 'indoor_altitude', fallback=1.0)
    
    def get_outdoor_altitude(self):
        """Get outdoor cruise altitude in meters"""
        return self.config.getfloat('MISSION', 'outdoor_altitude', fallback=3.0)
    
    def get_turn_direction(self):
        """Get turn direction preference"""
        return self.config.get('MISSION', 'turn_direction', fallback='right')
    
    # GPS Waypoints for 12-Checkpoint System
    def get_waypoint_1(self):
        """Get waypoint 1 coordinates"""
        return {
            'lat': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_1_lat', fallback=-6.365000),
            'lon': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_1_lon', fallback=106.825000),
            'alt': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_1_alt', fallback=30.0)
        }
    
    def get_waypoint_2(self):
        """Get waypoint 2 coordinates"""
        return {
            'lat': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_2_lat', fallback=-6.364500),
            'lon': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_2_lon', fallback=106.825500),
            'alt': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_2_alt', fallback=30.0)
        }
    
    def get_waypoint_3(self):
        """Get waypoint 3 coordinates"""
        return {
            'lat': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_3_lat', fallback=-6.364000),
            'lon': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_3_lon', fallback=106.826000),
            'alt': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_3_alt', fallback=30.0)
        }
    
    def get_waypoint_4(self):
        """Get waypoint 4 coordinates"""
        return {
            'lat': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_4_lat', fallback=-6.363500),
            'lon': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_4_lon', fallback=106.826500),
            'alt': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_4_alt', fallback=30.0)
        }
    
    def get_waypoint_5(self):
        """Get waypoint 5 coordinates (final waypoint)"""
        return {
            'lat': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_5_lat', fallback=-6.365500),
            'lon': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_5_lon', fallback=106.824500),
            'alt': self.config.getfloat('GPS_WAYPOINTS', 'waypoint_5_alt', fallback=30.0)
        }
    
    def get_all_waypoints(self):
        """Get all GPS waypoints as list"""
        return [
            self.get_waypoint_1(),
            self.get_waypoint_2(),
            self.get_waypoint_3(),
            self.get_waypoint_4(),
            self.get_waypoint_5()
        ]
    
    def get_home_position(self):
        """Get home position coordinates"""
        return {
            'lat': self.config.getfloat('GPS_WAYPOINTS', 'home_lat', fallback=-6.365500),
            'lon': self.config.getfloat('GPS_WAYPOINTS', 'home_lon', fallback=106.824500),
            'alt': self.config.getfloat('GPS_WAYPOINTS', 'home_alt', fallback=0.0)
        }
    
    def get_velocity_limits(self):
        """Get velocity limits as dict"""
        return {
            'horizontal': self.config.getfloat('flight', 'max_horizontal_velocity', fallback=2.0),
            'vertical': self.config.getfloat('flight', 'max_vertical_velocity', fallback=1.0),
            'yaw_rate': self.config.getfloat('flight', 'max_yaw_rate', fallback=45.0)
        }
    
    def get_safety_limits(self):
        """Get safety limits as dict"""
        return {
            'max_altitude': self.config.getfloat('flight', 'max_altitude_limit', fallback=10.0),
            'geofence_radius': self.config.getfloat('flight', 'geofence_radius', fallback=100.0),
            'battery_failsafe': self.config.getfloat('flight', 'battery_failsafe_voltage', fallback=14.4)
        }
    
    # ===========================================
    # Computer Vision Settings
    # ===========================================
    
    def get_detection_confidence(self):
        """Get object detection confidence threshold"""
        return self.config.getfloat('vision', 'detection_confidence_threshold', fallback=0.5)
    
    def get_alignment_tolerance(self):
        """Get alignment tolerance in pixels"""
        return self.config.getint('vision', 'alignment_tolerance_pixels', fallback=30)
    
    def get_models_directory(self):
        """Get YOLO models directory path"""
        models_dir = self.config.get('vision', 'models_directory', fallback='/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone/models')
        return Path(models_dir)
    
    def get_yolo_model_paths(self):
        """Get all YOLO model paths as dict"""
        models_dir = self.get_models_directory()
        return {
            'general': models_dir / self.config.get('vision', 'general_model', fallback='yolov8n.pt'),
            'exit_gate': models_dir / self.config.get('vision', 'exit_gate_model', fallback='yolov8n.pt'),
            'objects': models_dir / self.config.get('vision', 'objects_model', fallback='yolov8n.pt'),
            'dropzone': models_dir / self.config.get('vision', 'dropzone_model', fallback='yolov8n.pt')
        }
    
    def get_debug_visualization(self):
        """Get debug visualization setting"""
        return self.config.getboolean('vision', 'debug_visualization', fallback=True)
    
    def get_color_ranges(self, color='red'):
        """Get HSV color detection ranges"""
        return {
            'lower_hue': self.config.getint('vision', f'{color}_lower_hue', fallback=0),
            'upper_hue': self.config.getint('vision', f'{color}_upper_hue', fallback=10),
            'lower_saturation': self.config.getint('vision', f'{color}_lower_saturation', fallback=50),
            'upper_saturation': self.config.getint('vision', f'{color}_upper_saturation', fallback=255),
            'lower_value': self.config.getint('vision', f'{color}_lower_value', fallback=50),
            'upper_value': self.config.getint('vision', f'{color}_upper_value', fallback=255)
        }
    
    def get_min_object_area(self):
        """Get minimum object area for valid detection"""
        return self.config.getint('vision', 'min_object_area', fallback=1000)
    
    # ===========================================
    # Mission Waypoints
    # ===========================================
    
    def get_outdoor_pickup_waypoint(self):
        """Get outdoor pickup GPS coordinates"""
        # sesuaikan ini - CRITICAL: WAJIB UPDATE koordinat sesuai lokasi kompetisi!
        # Koordinat default Jakarta, HARUS DIGANTI dengan lokasi kompetisi asli!
        return {
            'latitude': self.config.getfloat('waypoints', 'outdoor_pickup_latitude', fallback=-6.365000),
            'longitude': self.config.getfloat('waypoints', 'outdoor_pickup_longitude', fallback=106.825000),
            'altitude': self.config.getfloat('waypoints', 'outdoor_pickup_altitude', fallback=30.0)
        }
    
    def get_outdoor_drop_waypoint(self):
        """Get outdoor drop GPS coordinates"""
        # sesuaikan ini - CRITICAL: WAJIB UPDATE koordinat sesuai lokasi kompetisi!
        # Koordinat default Jakarta, HARUS DIGANTI dengan lokasi kompetisi asli!
        return {
            'latitude': self.config.getfloat('waypoints', 'outdoor_drop_latitude', fallback=-6.364500),
            'longitude': self.config.getfloat('waypoints', 'outdoor_drop_longitude', fallback=106.825500),
            'altitude': self.config.getfloat('waypoints', 'outdoor_drop_altitude', fallback=30.0)
        }
    
    def get_gps_thresholds(self):
        """Get GPS accuracy and waypoint thresholds"""
        return {
            'accuracy_threshold': self.config.getfloat('waypoints', 'gps_accuracy_threshold', fallback=2.0),
            'waypoint_reached': self.config.getfloat('waypoints', 'waypoint_reached_threshold', fallback=3.0)
        }
    
    # ===========================================
    # Timing Parameters
    # ===========================================
    
    def get_checkpoint_timeout(self):
        """Get maximum time per checkpoint in seconds"""
        return self.config.getfloat('timing', 'checkpoint_timeout', fallback=120)
    
    def get_mission_timeout(self):
        """Get total mission timeout in seconds"""
        return self.config.getfloat('timing', 'mission_total_timeout', fallback=900)
    
    def get_hardware_delays(self):
        """Get hardware operation delays"""
        return {
            'magnet_on': self.config.getfloat('timing', 'magnet_activation_delay', fallback=1.0),
            'magnet_off': self.config.getfloat('timing', 'magnet_deactivation_delay', fallback=0.5),
            'camera_startup': self.config.getfloat('timing', 'camera_startup_delay', fallback=2.0),
            'descent_pickup': self.config.getfloat('timing', 'descent_pickup_time', fallback=3.0),
            'ascent_pickup': self.config.getfloat('timing', 'ascent_after_pickup_time', fallback=3.0)
        }
    
    def get_detection_timeouts(self):
        """Get detection operation timeouts"""
        return {
            'object_detection': self.config.getfloat('timing', 'object_detection_timeout', fallback=30.0),
            'alignment': self.config.getfloat('timing', 'alignment_timeout', fallback=15.0),
            'dropzone_search': self.config.getfloat('timing', 'dropzone_search_timeout', fallback=45.0)
        }
    
    # ===========================================
    # Debug & Logging
    # ===========================================
    
    def get_log_level(self):
        """Get logging level"""
        return self.config.get('debug', 'log_level', fallback='INFO')
    
    def get_debug_flags(self):
        """Get debug feature flags"""
        return {
            'checkpoint_logging': self.config.getboolean('debug', 'enable_checkpoint_logging', fallback=True),
            'sensor_logging': self.config.getboolean('debug', 'enable_sensor_logging', fallback=False),
            'camera_recording': self.config.getboolean('debug', 'enable_camera_recording', fallback=False),
            'flight_logging': self.config.getboolean('debug', 'enable_flight_data_logging', fallback=True)
        }
    
    def get_log_directory(self):
        """Get log directory path"""
        return self.config.get('debug', 'log_directory', fallback='/tmp/kaertei_logs')
    
    # ===========================================
    # Utility Methods
    # ===========================================
    
    def print_summary(self):
        """Print configuration summary for verification"""
        print("\n" + "="*50)
        print("🚁 KAERTEI 2025 FAIO - Hardware Configuration")
        print("="*50)
        print(f"PX4 Connection: {self.get_px4_connection_string()}")
        print(f"Front Camera: /dev/video{self.get_front_camera_index()}")
        print(f"Back Camera: /dev/video{self.get_back_camera_index()}")
        print(f"Top Camera: /dev/video{self.get_top_camera_index()}")
        print(f"Front Magnet Pin: GPIO {self.get_front_magnet_pin()}")
        print(f"Back Magnet Pin: GPIO {self.get_back_magnet_pin()}")
        print(f"Takeoff Altitude: {self.get_takeoff_altitude()}m")
        
        print(f"Models Directory: {self.get_models_directory()}")
        yolo_models = self.get_yolo_model_paths()
        for model_type, model_path in yolo_models.items():
            status = "✅" if model_path.exists() else "❌"
            print(f"YOLO {model_type.title()}: {model_path} {status}")
        
        pickup_wp = self.get_outdoor_pickup_waypoint()
        drop_wp = self.get_outdoor_drop_waypoint()
        print(f"Pickup Waypoint: {pickup_wp['latitude']:.6f}, {pickup_wp['longitude']:.6f}")
        print(f"Drop Waypoint: {drop_wp['latitude']:.6f}, {drop_wp['longitude']:.6f}")
        print("="*50 + "\n")
    
    def validate_config(self):
        """Validate configuration and return list of issues"""
        issues = []
        
        # Check PX4 port exists
        px4_port = self.get_px4_port()
        if not os.path.exists(px4_port):
            issues.append(f"❌ PX4 port not found: {px4_port}")
        
        # Check camera devices
        cameras = {
            'front': self.get_front_camera_index(),
            'back': self.get_back_camera_index(),
            'top': self.get_top_camera_index()
        }
        
        for name, index in cameras.items():
            device_path = f"/dev/video{index}"
            if not os.path.exists(device_path):
                issues.append(f"⚠️  {name.title()} camera not found: {device_path}")
        
        # Check GPIO pins are valid (0-27 for RPi)
        gpio_pins = {
            'front_magnet': self.get_front_magnet_pin(),
            'back_magnet': self.get_back_magnet_pin(),
            'status_led': self.get_status_led_pin(),
            'error_led': self.get_error_led_pin()
        }
        
        for name, pin in gpio_pins.items():
            if not (0 <= pin <= 27):
                issues.append(f"❌ Invalid GPIO pin for {name}: {pin} (should be 0-27)")
        
        # Check YOLO model files
        yolo_models = self.get_yolo_model_paths()
        for model_type, model_path in yolo_models.items():
            if not model_path.exists():
                issues.append(f"⚠️  YOLO {model_type} model not found: {model_path}")
        
        # Check models directory exists
        models_dir = self.get_models_directory()
        if not models_dir.exists():
            issues.append(f"❌ Models directory not found: {models_dir}")
        
        return issues

# Example usage and testing
if __name__ == "__main__":
    config = HardwareConfig()
    config.print_summary()
    
    issues = config.validate_config()
    if issues:
        print("⚠️  Configuration Issues Found:")
        for issue in issues:
            print(f"  {issue}")
    else:
        print("✅ Configuration validation passed!")
