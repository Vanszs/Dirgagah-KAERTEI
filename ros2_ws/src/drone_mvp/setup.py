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
            'test_node = drone_mvp.test_node:main',
            'mission_node = drone_mvp.mission_node:main',
            'checkpoint_mission_node = drone_mvp.checkpoint_mission_node:main',
            'camera_control_node = drone_mvp.camera_control_node:main',
            'magnet_control_node = drone_mvp.magnet_control_node:main',
            'vision_detector_node = drone_mvp.vision_detector_node:main',
            'sensor_monitor = drone_mvp.sensor_monitor:main',
            'kalibrasi_navigator = drone_mvp.kalibrasi_navigator:main',
            'magnet_control = drone_mvp.magnet_control:main',
            'exit_detector = drone_mvp.exit_detector:main',
            'dropzone_detector = drone_mvp.dropzone_detector:main',
            'gps_monitor = drone_mvp.gps_monitor:main',
            'waypoint_controller = drone_mvp.waypoint_controller:main',
            'flight_mode_switcher = drone_mvp.flight_mode_switcher:main',
            'lidar_control_node = drone_mvp.lidar_control_node:main',
            'gpio_control_node = drone_mvp.gpio_control_node:main',
        ],
    },
)
