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

    # Core mission control node
    mission_node = Node(
        package='drone_mvp',
        executable='checkpoint_mission_node.py',
        name='mission_control',
        output='screen',
        parameters=[
            {'debug_mode': LaunchConfiguration('debug')},
            {'competition_mode': LaunchConfiguration('competition_mode')},
            {'simulation_mode': LaunchConfiguration('simulation')}
        ],
        remappings=[
            ('/mavlink/command', '/flight_controller/command'),
            ('/mission/status', '/system/mission_status')
        ]
    )

    # Hardware control nodes (Raspberry Pi 5)
    hardware_nodes = GroupAction([
        # Camera control (3x USB cameras)
        Node(
            package='drone_mvp',
            executable='camera_control_node.py',
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
            package='drone_mvp',
            executable='gpio_control_node.py',
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
            package='drone_mvp',
            executable='lidar_control_node.py',
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

    # Computer vision nodes
    vision_nodes = GroupAction([
        # Main vision detector
        Node(
            package='drone_mvp',
            executable='vision_detector_node.py',
            name='vision_detector',
            output='screen',
            parameters=[
                {'confidence_threshold': 0.6},
                {'alignment_tolerance': 25}
            ]
        ),
        
        # Exit gate detector (top camera)
        Node(
            package='drone_mvp',
            executable='exit_detector.py',
            name='exit_detector',
            output='screen'
        ),
        
        # Dropzone detector
        Node(
            package='drone_mvp',
            executable='dropzone_detector.py',
            name='dropzone_detector',
            output='screen'
        )
    ])

    # Navigation and control nodes
    navigation_nodes = GroupAction([
        # GPS monitor and waypoint controller
        Node(
            package='drone_mvp',
            executable='gps_monitor.py',
            name='gps_monitor',
            output='screen'
        ),
        
        Node(
            package='drone_mvp',
            executable='waypoint_controller.py',
            name='waypoint_controller',
            output='screen'
        ),
        
        # Flight mode switcher
        Node(
            package='drone_mvp',
            executable='flight_mode_switcher.py',
            name='flight_mode_switcher',
            output='screen'
        )
    ])

    # System monitoring nodes
    monitoring_nodes = GroupAction([
        # Sensor monitor
        Node(
            package='drone_mvp',
            executable='sensor_monitor.py',
            name='sensor_monitor',
            output='screen',
            parameters=[
                {'enable_lidar_monitoring': True},
                {'enable_camera_monitoring': True},
                {'obstacle_threshold': 2.0}
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
