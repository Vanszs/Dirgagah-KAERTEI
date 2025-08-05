#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'px4_connection',
            default_value='/dev/ttyUSB0:57600',
            description='PX4 MAVLink connection string'
        ),
        
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug mode (manual next command) or autonomous mode'
        ),
        
        DeclareLaunchArgument(
            'use_mavros',
            default_value='true',
            description='Use MAVROS (true) or direct MAVLink (false)'
        ),
        
        # Checkpoint Mission Control Node (MAVROS version)
        Node(
            package='drone_mvp',
            executable='checkpoint_mission_mavros.py',
            name='checkpoint_mission_node',
            output='screen',
            parameters=[{
                'debug_mode': LaunchConfiguration('debug_mode')
            }],
            condition=IfCondition(LaunchConfiguration('use_mavros'))
        ),
        
        # Checkpoint Mission Control Node (Direct MAVLink version)
        Node(
            package='drone_mvp',
            executable='checkpoint_mission_node.py',
            name='checkpoint_mission_node',
            output='screen',
            parameters=[{
                'px4_connection': LaunchConfiguration('px4_connection'),
                'debug_mode': LaunchConfiguration('debug_mode')
            }],
            condition=UnlessCondition(LaunchConfiguration('use_mavros'))
        ),
        
        # Camera Control Node
        Node(
            package='drone_mvp',
            executable='camera_control_node.py',
            name='camera_control_node',
            output='screen'
        ),
        
        # Magnet Control Node
        Node(
            package='drone_mvp',
            executable='magnet_control_node.py',
            name='magnet_control_node',
            output='screen'
        ),
        
        # Optional: Start QGroundControl for monitoring
        # ExecuteProcess(
        #     cmd=['qgroundcontrol'],
        #     output='screen'
        # )
    ])
