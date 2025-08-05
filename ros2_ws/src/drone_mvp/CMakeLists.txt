cmake_minimum_required(VERSION 3.10)
project(drone_mvp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorStatus.msg"
  "srv/MagnetControl.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules (only once to avoid duplicate targets)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  drone_mvp/mission_node.py
  drone_mvp/checkpoint_mission_node.py
  drone_mvp/camera_control_node.py
  drone_mvp/magnet_control_node.py
  drone_mvp/vision_detector_node.py
  drone_mvp/sensor_monitor.py
  drone_mvp/kalibrasi_navigator.py
  drone_mvp/magnet_control.py
  drone_mvp/exit_detector.py
  drone_mvp/dropzone_detector.py
  drone_mvp/gps_monitor.py
  drone_mvp/waypoint_controller.py
  drone_mvp/flight_mode_switcher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
