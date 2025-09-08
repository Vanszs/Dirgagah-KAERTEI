from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_node',
            output='screen',
            parameters=[
                {'fcu_url': '/dev/ttyACM0:115200'},
                {'pluginlists_yaml': '/opt/ros/foxy/share/mavros/apm_pluginlists.yaml'},
                {'config_yaml': '/opt/ros/foxy/share/mavros/apm_config.yaml'},
            ],
        )
    ])
