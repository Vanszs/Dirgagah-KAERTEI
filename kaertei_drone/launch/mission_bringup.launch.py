#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Mission Bringup (12 CP)
Modular launch with mode and cfg (YAML) arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition


def generate_launch_description():
    debug_arg = DeclareLaunchArgument('debug_mode', default_value='true')
    auto_arg = DeclareLaunchArgument('auto_continue', default_value='false')
    cfg_arg = DeclareLaunchArgument('cfg', default_value='config/hardware_config.yaml')
    start_vision_arg = DeclareLaunchArgument('start_vision', default_value='false')
    start_health_arg = DeclareLaunchArgument('start_health', default_value='false')
    start_gpsmon_arg = DeclareLaunchArgument('start_gpsmon', default_value='false')
    start_emerg_arg = DeclareLaunchArgument('start_emerg', default_value='false')
    start_adapters_arg = DeclareLaunchArgument('start_adapters', default_value='false')

    debug_mode = LaunchConfiguration('debug_mode')
    auto_continue = LaunchConfiguration('auto_continue')
    # cfg is read directly by the node from package path; not passed as ROS params

    mission_node = Node(
        package='kaertei_drone',
        executable='checkpoint_mission_mavros',
        name='mission_fsm',
        output='screen',
        parameters=[{'log_level': 'INFO'}, {'debug_mode': debug_mode}, {'auto_continue': auto_continue}]
    )

    health = Node(
        package='kaertei_drone', executable='system_health_monitor', name='health', output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR'],
        condition=IfCondition(LaunchConfiguration('start_health'))
    )
    gpsmon = Node(
        package='kaertei_drone', executable='gps_waypoint_monitor', name='gpsmon', output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('start_gpsmon'))
    )
    emerg = Node(
        package='kaertei_drone', executable='emergency_controller', name='emerg', output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('start_emerg'))
    )
    vision = Node(
        package='kaertei_drone', executable='unified_vision_system', name='vision', output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR'],
        condition=IfCondition(LaunchConfiguration('start_vision'))
    )
    adapters = Node(
        package='kaertei_drone', executable='topic_adapters', name='adapters', output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('start_adapters'))
    )

    # Start MAVROS a bit later to ensure serial ready
    mavros_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='mavros', executable='mavros_node', namespace='mavros_node', name='mavros_node', output='screen',
            parameters=[{'fcu_url': '/dev/ttyACM0:115200'}]
        )]
    )

    return LaunchDescription([
        debug_arg,
        auto_arg,
        cfg_arg,
        start_vision_arg,
        start_health_arg,
        start_gpsmon_arg,
        start_emerg_arg,
        start_adapters_arg,
        mission_node,
        health,
        gpsmon,
        emerg,
        vision,
        adapters,
        mavros_node,
    ])
