#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launch file for KAERTEI 2025 FAIO Drone System
    Raspberry Pi 5 + Pixhawk4 Configuration
    
    Hardware Architecture:
    - Pixhawk4: 6 motors + GPS only
    - Raspberry Pi 5: 3 cameras + 3 LiDAR + 2 relays
    """
    
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true',
        description='Enable debug mode'
    )
    
    competition_mode_arg = DeclareLaunchArgument(
        'competition_mode', default_value='false',
        description='Enable competition mode (no debug delays)'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation', default_value='false',
        description='Run in simulation mode (no hardware)'
    )

    # Core mission control node - USE SIMPLIFIED SYSTEM
    mission_node = Node(
        package='kaertei_drone',
        executable='simplified_mission_control',
        name='mission_control',
        output='screen',
        parameters=[
            {'debug_mode': LaunchConfiguration('debug')},
            {'competition_mode': LaunchConfiguration('competition_mode')},
            {'simulation_mode': LaunchConfiguration('simulation')}
        ],
        remappings=[
            ('/mission/status', '/system/mission_status')
        ]
    )

    # Hardware control nodes (Raspberry Pi 5)
    hardware_nodes = GroupAction([
        # Camera control (3x USB cameras)
        Node(
            package='kaertei_drone',
            executable='camera_control_node',
            name='camera_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('simulation')),
            parameters=[
                {'front_camera_index': 0},
                {'back_camera_index': 2},
                {'top_camera_index': 4},
                {'enable_recording': LaunchConfiguration('debug')}
            ]
        ),
        
        # GPIO control (2x relay for electromagnets)
        Node(
            package='kaertei_drone',
            executable='gpio_control_node',
            name='gpio_controller',
            output='screen',
            parameters=[
                {'front_relay_pin': 18},
                {'back_relay_pin': 19},
                {'relay_active_high': False}
            ]
        ),
        
        # LiDAR control (3x TF Mini Plus)
        Node(
            package='kaertei_drone',
            executable='lidar_control_node',
            name='lidar_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('simulation')),
            parameters=[
                {'front_lidar_port': '/dev/ttyUSB1'},
                {'left_lidar_port': '/dev/ttyUSB2'},
                {'right_lidar_port': '/dev/ttyUSB3'},
                {'sample_rate': 100}
            ]
        )
    ])

    # Computer vision nodes - USE UNIFIED SYSTEM
    vision_nodes = GroupAction([
        # Unified vision system 
        Node(
            package='kaertei_drone',
            executable='unified_vision_system',
            name='vision_system',
            output='screen',
            parameters=[
                {'confidence_threshold': 0.6},
                {'alignment_tolerance': 25}
            ]
        )
    ])

    # Navigation and control nodes
    navigation_nodes = GroupAction([
        # GPS monitor and waypoint controller
        Node(
            package='kaertei_drone',
            executable='gps_monitor',
            name='gps_monitor',
            output='screen'
        ),
        
        Node(
            package='kaertei_drone',
            executable='waypoint_navigator',
            name='waypoint_controller',
            output='screen'
        ),
        
        # Flight mode switcher
        Node(
            package='kaertei_drone',
            executable='flight_mode_switcher',
            name='flight_mode_switcher',
            output='screen'
        )
    ])

    # System monitoring nodes
    monitoring_nodes = GroupAction([
        # Sensor monitor
        Node(
            package='kaertei_drone',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen',
            parameters=[
                {'enable_lidar_monitoring': True},
                {'enable_camera_monitoring': True},
                {'obstacle_threshold': 2.0}
            ]
        ),
        
        # Flight state monitor
        Node(
            package='kaertei_drone',
            executable='flight_state_monitor',
            name='flight_state_monitor',
            output='screen',
            parameters=[
                {'update_rate': 2.0},
                {'connection_timeout': 5.0},
                {'battery_warning_voltage': 3.3},
                {'battery_critical_voltage': 3.1}
            ]
        ),
        
        # Topic adapters bridge
        Node(
            package='kaertei_drone',
            executable='topic_adapters',
            name='topic_adapters',
            output='screen',
            parameters=[
                {'update_rate': 10.0},
                {'enable_mavros_bridge': True},
                {'enable_vision_bridge': True},
                {'enable_sensor_bridge': True}
            ]
        ),
        
        # GPS waypoint monitor
        Node(
            package='kaertei_drone',
            executable='gps_waypoint_monitor',
            name='gps_waypoint_monitor',
            output='screen',
            parameters=[
                {'waypoint_radius': 3.0},
                {'update_rate': 2.0},
                {'min_gps_accuracy': 5.0}
            ]
        ),
        
        # Emergency controller
        Node(
            package='kaertei_drone',
            executable='emergency_controller',
            name='emergency_controller',
            output='screen',
            parameters=[
                {'update_rate': 10.0},
                {'enable_gpio_monitoring': True},
                {'battery_critical_voltage': 3.0},
                {'altitude_limit_max': 100.0}
            ]
        ),
        
        # System health monitor
        Node(
            package='kaertei_drone',
            executable='system_health_monitor',
            name='system_health_monitor',
            output='screen',
            parameters=[
                {'update_rate': 1.0},
                {'cpu_warning_threshold': 80.0},
                {'memory_warning_threshold': 85.0},
                {'temperature_warning_threshold': 70.0}
            ]
        ),
        
        # Waypoint Navigator (Custom Navigation)
        Node(
            package='kaertei_drone',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[
                {'waypoint_reached_threshold': 3.0},
                {'waypoint_timeout': 120},
                {'navigation_altitude': 3.0}
            ]
        ),
        
        # PX4 Waypoint Navigator (PX4 Mission-based Navigation)
        Node(
            package='kaertei_drone',
            executable='px4_waypoint_navigator',
            name='px4_waypoint_navigator',
            output='screen',
            parameters=[
                {'waypoint_reached_threshold': 3.0},
                {'sequence_timeout': 120},
                {'auto_mission_altitude': 3.0}
            ]
        )
    ])

    return LaunchDescription([
        # Arguments
        debug_arg,
        competition_mode_arg,
        simulation_arg,
        
        # Core mission control
        mission_node,
        
        # Hardware control (Pi 5)
        hardware_nodes,
        
        # Computer vision
        vision_nodes,
        
        # Navigation
        navigation_nodes,
        
        # System monitoring
        monitoring_nodes
    ])
