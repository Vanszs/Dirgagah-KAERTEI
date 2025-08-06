from setuptools import setup, find_packages

package_name = 'drone_mvp'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/kaertei_pi5_system.launch.py']),
        ('share/' + package_name + '/config', ['config/hardware_config.conf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KAERTEI Team',
    maintainer_email='kaertei@example.com',
    description='VTOL Autonomous Drone System for KAERTEI 2025 FAIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core system nodes
            'magnet_control_node = drone_mvp.magnet_control_node:main',
            'sensor_monitor = drone_mvp.sensor_monitor:main',
            'kalibrasi_navigator = drone_mvp.kalibrasi_navigator:main',
            'gps_monitor = drone_mvp.gps_monitor:main',
            'flight_mode_switcher = drone_mvp.flight_mode_switcher:main',
            'lidar_control_node = drone_mvp.lidar_control_node:main',
            'gpio_control_node = drone_mvp.gpio_control_node:main',
            'camera_control_node = drone_mvp.camera_control_node:main',
            'flight_state_monitor = drone_mvp.flight_state_monitor:main',
            'topic_adapters = drone_mvp.topic_adapters:main',
            'gps_waypoint_monitor = drone_mvp.gps_waypoint_monitor:main',
            'emergency_controller = drone_mvp.emergency_controller:main',
            'system_health_monitor = drone_mvp.system_health_monitor:main',
            
            # Navigation system
            'px4_waypoint_navigator = drone_mvp.px4_waypoint_navigator:main',
            'px4_waypoint_config = drone_mvp.px4_waypoint_config:main',
            
            # Unified vision system (main)
            'unified_vision_system = drone_mvp.vision.unified_vision_system:main',
            
            # Simplified mission control (main)
            'simplified_mission_control = drone_mvp.simplified_mission_control:main',
            
            # Vision system (unified)
            'unified_vision_system = drone_mvp.vision.unified_vision_system:main',
            
            # Simplified mission control (main)
            'simplified_mission_control = drone_mvp.simplified_mission_control:main',
        ],
    },
)
