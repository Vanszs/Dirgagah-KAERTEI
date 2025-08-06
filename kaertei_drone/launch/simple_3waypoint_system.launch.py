#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Simple 3-Waypoint Mission Launch
Launches complete system for simplified 3-waypoint mission:
- Exit Gate → WP1 (search+pickup) → WP2 (drop) → WP3 (finish)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('kaertei_drone')
    
    # Launch arguments
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug mode with image visualization'
    )
    
    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_path', 
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )
    
    waypoint_preset_arg = DeclareLaunchArgument(
        'waypoint_preset',
        default_value='simple_3wp',
        description='PX4 waypoint preset to use'
    )
    
    # Core mission system
    simple_mission_node = Node(
        package='kaertei_drone',
        executable='simplified_mission_control',
        name='simplified_mission_control',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode'),
            'waypoint_preset': LaunchConfiguration('waypoint_preset'),
            'search_timeout': 30.0,
            'alignment_timeout': 10.0
        }],
        remappings=[
            # Mission control
            ('/mission/start', '/mission/start'),
            ('/mission/checkpoint', '/mission/checkpoint'),
            ('/mission/status', '/mission/status'),
            
            # PX4 waypoint system
            ('/px4/waypoint/command', '/px4/waypoint/command'),
            ('/px4/waypoint/status', '/px4/waypoint/status'),
            
            # Vision control
            ('/vision/detection_mode', '/vision/detection_mode'),
            ('/vision/exit_gate/enable', '/vision/exit_gate/enable'),
            ('/vision/item/enable', '/vision/item/enable'),
            ('/vision/dropzone/enable', '/vision/dropzone/enable'),
            
            # Hardware control
            ('/hardware/magnet/enable', '/hardware/magnet/enable'),
            ('/mavros/setpoint_velocity/cmd_vel_unstamped', '/mavros/setpoint_velocity/cmd_vel_unstamped'),
        ]
    )
    
    # PX4 waypoint navigator
    px4_waypoint_node = Node(
        package='kaertei_drone',
        executable='px4_waypoint_navigator',
        name='px4_waypoint_navigator',
        output='screen',
        parameters=[{
            'default_preset': LaunchConfiguration('waypoint_preset'),
            'waypoint_tolerance': 2.0,
            'altitude_hold': 3.0,
            'max_velocity': 3.0,
            'debug_mode': LaunchConfiguration('debug_mode')
        }],
        remappings=[
            ('/px4/waypoint/command', '/px4/waypoint/command'),
            ('/px4/waypoint/status', '/px4/waypoint/status'),
            ('/mavros/local_position/pose', '/mavros/local_position/pose'),
            ('/mavros/setpoint_position/local', '/mavros/setpoint_position/local'),
        ]
    )
    
    # Vision system nodes - use single unified vision system
    unified_vision_node = Node(
        package='kaertei_drone',
        executable='unified_vision_system',
        name='unified_vision_system',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode'),
            'yolo_model_path': LaunchConfiguration('yolo_model_path'),
            'confidence_threshold': 0.5,
            'front_camera_topic': '/vision/front/image',
            'bottom_camera_topic': '/vision/bottom/image',
            'enable_auto_switching': True
        }]
    )
    
    # System health monitor
    system_health_node = Node(
        package='kaertei_drone',
        executable='system_health_monitor',
        name='system_health_monitor',
        output='screen'
    )
    
    # GPS waypoint monitor
    gps_monitor_node = Node(
        package='kaertei_drone', 
        executable='gps_waypoint_monitor',
        name='gps_waypoint_monitor',
        output='screen'
    )
    
    # Flight state monitor
    flight_state_node = Node(
        package='kaertei_drone',
        executable='flight_state_monitor', 
        name='flight_state_monitor',
        output='screen'
    )
    
    # Emergency controller
    emergency_node = Node(
        package='kaertei_drone',
        executable='emergency_controller',
        name='emergency_controller',
        output='screen'
    )
    
    # Camera control node
    camera_control_node = Node(
        package='kaertei_drone',
        executable='camera_control_node',
        name='camera_control_node',
        output='screen',
        parameters=[{
            'front_camera_device': '/dev/video0',
            'bottom_camera_device': '/dev/video2',
            'camera_width': 640,
            'camera_height': 480,
            'camera_fps': 30
        }],
        remappings=[
            ('/vision/front/image', '/vision/front/image'),
            ('/vision/bottom/image', '/vision/bottom/image'),
            ('/vision/camera_switch', '/vision/camera_switch'),
        ]
    )
    
    # Hardware control nodes
    magnet_control_node = Node(
        package='kaertei_drone',
        executable='magnet_control_node',
        name='magnet_control_node',
        output='screen',
        parameters=[{
            'magnet_gpio_pin': 18,
            'magnet_hold_current': 0.5,
            'debug_mode': LaunchConfiguration('debug_mode')
        }],
        remappings=[
            ('/hardware/magnet/enable', '/hardware/magnet/enable'),
            ('/hardware/magnet/status', '/hardware/magnet/status'),
        ]
    )
    
    return LaunchDescription([
        # Arguments
        debug_mode_arg,
        yolo_model_path_arg,
        waypoint_preset_arg,
        
        # Core system
        simple_mission_node,
        px4_waypoint_node,
        
        # Vision system
        unified_vision_node,
        
        # Monitoring system
        system_health_node,
        gps_monitor_node,
        flight_state_node,
        emergency_node,
        
        # Hardware
        camera_control_node,
        magnet_control_node,
    ])
