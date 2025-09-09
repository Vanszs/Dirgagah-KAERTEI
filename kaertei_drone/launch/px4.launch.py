from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros_node',
            name='mavros_node',
            output='screen',
            parameters=[
                {
                    'fcu_url': 'serial:///dev/serial/by-id/usb-ArduPilot_Pixhawk1_3A0023001051333036333432-if00:115200',
                    'gcs_url': '',
                    'tgt_system': 1,
                }
            ],
        )
    ])
