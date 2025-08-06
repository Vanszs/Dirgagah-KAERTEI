from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'kaertei_drone'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KAERTEI Team',
    maintainer_email='kaertei@example.com',
    description='KAERTEI 2025 FAIO - Autonomous Drone System (Clean Structure)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Mission Control
            'checkpoint_mission_mavros = kaertei_drone.mission.checkpoint_mission_mavros:main',
            'simplified_mission_control = kaertei_drone.mission.simplified_mission_control:main',
            
            # Hardware Control
            'camera_control_node = kaertei_drone.hardware.camera_control_node:main',
            'gpio_control_node = kaertei_drone.hardware.gpio_control_node:main',
            'magnet_control_node = kaertei_drone.hardware.magnet_control_node:main',
            'lidar_control_node = kaertei_drone.hardware.lidar_control_node:main',
            'emergency_controller = kaertei_drone.hardware.emergency_controller:main',
            
            # Navigation
            'px4_waypoint_navigator = kaertei_drone.navigation.px4_waypoint_navigator:main',
            'flight_state_monitor = kaertei_drone.navigation.flight_state_monitor:main',
            'gps_monitor = kaertei_drone.navigation.gps_monitor:main',
            'gps_waypoint_monitor = kaertei_drone.navigation.gps_waypoint_monitor:main',
            'flight_mode_switcher = kaertei_drone.navigation.flight_mode_switcher:main',
            'kalibrasi_navigator = kaertei_drone.navigation.kalibrasi_navigator:main',
            'waypoint_navigator = kaertei_drone.navigation.waypoint_navigator:main',
            
            # Vision System
            'unified_vision_system = kaertei_drone.vision.unified_vision_system:main',
            
            # Monitoring
            'system_health_monitor = kaertei_drone.monitoring.system_health_monitor:main',
            'sensor_monitor = kaertei_drone.monitoring.sensor_monitor:main',
            'topic_adapters = kaertei_drone.monitoring.topic_adapters:main',
        ],
    },
)
