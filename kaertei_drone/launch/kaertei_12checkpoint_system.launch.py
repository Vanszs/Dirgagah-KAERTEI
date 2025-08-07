#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - 12 Checkpoint System Launch
Complete launch file for 12-checkpoint autonomous drone mission system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launch file for KAERTEI 2025 FAIO 12-Checkpoint System
    
    System Architecture:
    - Pixhawk4 Flight Controller (6 motors + GPS + IMU + sensors)
    - Raspberry Pi 5 Companion Computer (AI vision + sensor processing)
    - 3x USB Cameras (front_bottom, back, front for navigation)
    - 3x LiDAR TF Mini Plus (front, left, right obstacle detection)
    - 2x Electromagnets (front, back pickup/drop)
    """
    
    # Launch arguments
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode', default_value='true',
        description='Enable debug mode for step-by-step checkpoint execution'
    )
    
    auto_continue_arg = DeclareLaunchArgument(
        'auto_continue', default_value='false',
        description='Automatically continue between checkpoints without user input'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation', default_value='false',
        description='Run in simulation mode (no real hardware)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='INFO',
        description='ROS 2 logging level'
    )
    
    # ===========================================
    # CORE MISSION CONTROL
    # ===========================================
    
    mission_control_node = Node(
        package='kaertei_drone',
        executable='checkpoint_12_mission_node',
        name='mission_control',
        output='screen',
        parameters=[
            {'debug_mode': LaunchConfiguration('debug_mode')},
            {'auto_continue': LaunchConfiguration('auto_continue')},
            {'simulation_mode': LaunchConfiguration('simulation')},
            {'log_level': LaunchConfiguration('log_level')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # ===========================================
    # HARDWARE CONTROL LAYER
    # ===========================================
    
    hardware_group = GroupAction([
        
        # Camera Control - 2x USB cameras (top camera disabled per MISSION_CLARIFICATION.md)
        # Uses centralized hardware_config.conf for parameters
        Node(
            package='kaertei_drone',
            executable='camera_control_node',
            name='camera_controller',
            output='screen'
            # No parameters here - reads from hardware_config.conf
        ),
        
        # Magnet Control - 2x electromagnets via GPIO relays
        # Uses centralized hardware_config.conf for parameters
        Node(
            package='kaertei_drone',
            executable='magnet_control_node',
            name='magnet_controller',
            output='screen'
            # No parameters here - reads from hardware_config.conf
        ),
        
        # LiDAR Control - 3x TF Mini Plus sensors
        # Uses centralized hardware_config.conf for parameters
        Node(
            package='kaertei_drone',
            executable='lidar_control_node',
            name='lidar_controller',
            output='screen'
            # No parameters here - reads from hardware_config.conf
        ),
        
        # GPIO Control - Emergency stop & status LEDs
        # Uses centralized hardware_config.conf for parameters
        Node(
            package='kaertei_drone',
            executable='gpio_control_node',
            name='gpio_controller',
            output='screen'
            # No parameters here - reads from hardware_config.conf
        )
    ])
    
    # ===========================================
    # COMPUTER VISION SYSTEM
    # ===========================================
    
    vision_group = GroupAction([
        
        # Unified Vision System - YOLOv8 object detection
        # Now uses centralized hardware_config.conf for parameters
        Node(
            package='kaertei_drone',
            executable='unified_vision_system',
            name='vision_system',
            output='screen',
            parameters=[
                {'debug_visualization': LaunchConfiguration('debug_mode')}
            ]
        )
    ])
    
    # ===========================================
    # NAVIGATION & FLIGHT CONTROL
    # ===========================================
    
    navigation_group = GroupAction([
        
        # Flight Mode Switcher - MAVROS mode management
        Node(
            package='kaertei_drone',
            executable='flight_mode_switcher',
            name='flight_mode_controller',
            output='screen',
            parameters=[
                {'default_mode': 'STABILIZED'},
                {'armed_timeout': 10.0},
                {'mode_switch_timeout': 5.0}
            ]
        ),
        
        # GPS Waypoint Navigator - 5 waypoint navigation
        # Uses centralized hardware_config.conf for parameters
        # NOTE: Waypoints 1-5 retrieved from PX4/ArduPilot after successful bucket drop
        Node(
            package='kaertei_drone',
            executable='gps_waypoint_monitor',
            name='gps_navigator',
            output='screen'
            # No parameters here - reads from hardware_config.conf
        ),
        
        # Flight State Monitor - System health & telemetry
        Node(
            package='kaertei_drone',
            executable='flight_state_monitor',
            name='flight_monitor',
            output='screen',
            parameters=[
                {'battery_low_threshold': 14.4},
                {'altitude_limit': 50.0},
                {'geofence_radius': 500.0}
            ]
        )
    ])
    
    # ===========================================
    # SYSTEM MONITORING & SAFETY
    # ===========================================
    
    monitoring_group = GroupAction([
        
        # System Health Monitor - Overall system status
        Node(
            package='kaertei_drone',
            executable='system_health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[
                {'health_check_rate': 2.0},
                {'critical_failure_timeout': 30.0}
            ]
        ),
        
        # Sensor Monitor - LiDAR & hardware sensor status
        Node(
            package='kaertei_drone',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen',
            parameters=[
                {'sensor_timeout': 5.0},
                {'sensor_check_rate': 10.0}
            ]
        ),
        
        # Emergency Controller - Safety & emergency procedures
        Node(
            package='kaertei_drone',
            executable='emergency_controller',
            name='emergency_controller',
            output='screen',
            parameters=[
                {'emergency_land_altitude': 0.5},
                {'emergency_timeout': 60.0}
            ]
        )
    ])
    
    # ===========================================
    # MAVROS CONNECTION (DELAYED START)
    # ===========================================
    
    # Import hardware config directly in launch file to configure MAVROS
    import configparser
    from pathlib import Path
    
    # Load hardware config for MAVROS connection
    config = configparser.ConfigParser()
    config_file = str(Path(__file__).parent.parent) + "/config/hardware_config.conf"
    config.read(config_file)
    
    # Get PX4 connection parameters from config
    px4_port = config.get('flight_controller', 'connection_port', fallback='/dev/ttyUSB0')
    px4_baud = config.get('flight_controller', 'baud_rate', fallback='57600')
    px4_url = f"{px4_port}:{px4_baud}"
    
    # Start MAVROS after other nodes are ready
    mavros_node = TimerAction(
        period=3.0,  # Wait 3 seconds for other nodes
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                output='screen',
                parameters=[
                    {'fcu_url': px4_url},  # From hardware_config.conf
                    {'gcs_url': ''},
                    {'target_system_id': 1},
                    {'target_component_id': 1},
                    {'fcu_protocol': 'v2.0'},
                    {'system_id': 255},
                    {'component_id': 240}
                ],
                condition=IfCondition('true')  # Always start MAVROS
            )
        ]
    )
    
    # ===========================================
    # TOPIC BRIDGES & ADAPTERS
    # ===========================================
    
    adapters_group = GroupAction([
        
        # Topic Adapters - Convert between different message types
        Node(
            package='kaertei_drone',
            executable='topic_adapters',
            name='topic_adapters',
            output='screen'
        )
    ])
    
    return LaunchDescription([
        # Launch arguments
        debug_mode_arg,
        auto_continue_arg, 
        simulation_arg,
        log_level_arg,
        
        # Core mission control (highest priority)
        mission_control_node,
        
        # Hardware control layer
        hardware_group,
        
        # Computer vision system
        vision_group,
        
        # Navigation & flight control
        navigation_group,
        
        # System monitoring & safety
        monitoring_group,
        
        # Topic bridges & adapters
        adapters_group,
        
        # MAVROS connection (delayed start)
        mavros_node
    ])
