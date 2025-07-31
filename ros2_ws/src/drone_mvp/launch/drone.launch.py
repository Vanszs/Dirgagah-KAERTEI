#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """Generate launch description for KAERTEI 2025 FAIO Drone System with ArduPilot"""
    
    # Package directory
    pkg_share = FindPackageShare(package='drone_mvp').find('drone_mvp')
    
    # Config file paths
    config_file = PathJoinSubstitution([
        FindPackageShare('drone_mvp'),
        'config',
        'params.yaml'
    ])
    
    mavros_config_file = PathJoinSubstitution([
        FindPackageShare('drone_mvp'),
        'config',
        'mavros_config.yaml'
    ])
    
    # Launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyUSB0:57600',
        description='FCU connection URL'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@127.0.0.1:14550',
        description='GCS connection URL'
    )
    
    camera_front_arg = DeclareLaunchArgument(
        'camera_front',
        default_value='0',
        description='Front camera device index'
    )
    
    camera_back_arg = DeclareLaunchArgument(
        'camera_back', 
        default_value='1',
        description='Back camera device index'
    )
    
    camera_top_arg = DeclareLaunchArgument(
        'camera_top',
        default_value='2', 
        description='Top camera device index'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # MAVROS launch
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch.py'
            ])
        ]),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'config_yaml': mavros_config_file
        }.items()
    )
    
    # Core mission node
    mission_node = Node(
        package='drone_mvp',
        executable='mission_node.py',
        name='mission_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/mission/state', '/mission/state'),
            ('/mission/command', '/mission/command')
        ]
    )
    
    # Vision detector nodes for each camera
    vision_front_node = Node(
        package='drone_mvp',
        executable='vision_detector_node.py',
        name='vision_detector_front',
        output='screen',
        parameters=[config_file],
        arguments=['--camera', 'front'],
        remappings=[
            ('/vision/detection', '/vision/detection'),
            ('/vision/front/image', '/vision/front/image')
        ]
    )
    
    vision_back_node = Node(
        package='drone_mvp',
        executable='vision_detector_node.py',
        name='vision_detector_back',
        output='screen',
        parameters=[config_file],
        arguments=['--camera', 'back'],
        remappings=[
            ('/vision/detection', '/vision/detection'),
            ('/vision/back/image', '/vision/back/image')
        ]
    )
    
    vision_top_node = Node(
        package='drone_mvp',
        executable='vision_detector_node.py',
        name='vision_detector_top',
        output='screen',
        parameters=[config_file],
        arguments=['--camera', 'top'],
        remappings=[
            ('/vision/detection', '/vision/detection'),
            ('/vision/top/image', '/vision/top/image')
        ]
    )
    
    # Sensor monitoring node
    sensor_monitor_node = Node(
        package='drone_mvp',
        executable='sensor_monitor.py',
        name='sensor_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    # Navigation calibration node  
    kalibrasi_navigator_node = Node(
        package='drone_mvp',
        executable='kalibrasi_navigator.py',
        name='kalibrasi_navigator',
        output='screen',
        parameters=[config_file]
    )
    
    # Magnet control node
    magnet_control_node = Node(
        package='drone_mvp',
        executable='magnet_control.py',
        name='magnet_control',
        output='screen',
        parameters=[config_file]
    )
    
    # Exit detector node
    exit_detector_node = Node(
        package='drone_mvp',
        executable='exit_detector.py',
        name='exit_detector',
        output='screen',
        parameters=[config_file]
    )
    
    # Dropzone detector node
    dropzone_detector_node = Node(
        package='drone_mvp',
        executable='dropzone_detector.py',
        name='dropzone_detector',
        output='screen', 
        parameters=[config_file]
    )
    
    # GPS monitor node
    gps_monitor_node = Node(
        package='drone_mvp',
        executable='gps_monitor.py',
        name='gps_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    # Waypoint controller node
    waypoint_controller_node = Node(
        package='drone_mvp',
        executable='waypoint_controller.py',
        name='waypoint_controller',
        output='screen',
        parameters=[config_file]
    )
    
    # Flight mode switcher node
    flight_mode_switcher_node = Node(
        package='drone_mvp',
        executable='flight_mode_switcher.py',
        name='flight_mode_switcher',
        output='screen',
        parameters=[config_file]
    )
    
    # Optional: Static transform publishers for camera frames
    camera_front_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_front_tf',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_front_link']
    )
    
    camera_back_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='camera_back_tf',
        arguments=['-0.1', '0', '0', '3.14159', '0', '0', 'base_link', 'camera_back_link']
    )
    
    camera_top_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='camera_top_tf',
        arguments=['0', '0', '0.05', '0', '-1.5708', '0', 'base_link', 'camera_top_link']
    )
    
    # RQT console for monitoring (optional, comment out if not needed)
    rqt_console = ExecuteProcess(
        cmd=['rqt_console'],
        output='screen',
        condition='false'  # Set to 'true' to enable
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        fcu_url_arg,
        gcs_url_arg,
        camera_front_arg,
        camera_back_arg,
        camera_top_arg,
        use_sim_time_arg,
        
        # MAVROS launch (ArduPilot communication)
        mavros_launch,
        
        # Core nodes
        mission_node,
        sensor_monitor_node,
        kalibrasi_navigator_node,
        magnet_control_node,
        gps_monitor_node,
        waypoint_controller_node,
        flight_mode_switcher_node,
        
        # Vision nodes
        vision_front_node,
        vision_back_node,
        vision_top_node,
        
        # Specialized detector nodes
        exit_detector_node,
        dropzone_detector_node,
        
        # Transform publishers
        camera_front_tf,
        camera_back_tf,
        camera_top_tf,
        
        # Optional monitoring tools
        # rqt_console,
    ])
