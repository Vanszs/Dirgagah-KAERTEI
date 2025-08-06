#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - System Validation Script
Ubuntu 22.04 LTS System Validation
"""

import os
import sys
import subprocess
import importlib
from pathlib import Path
import argparse

# Colors for output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_success(msg):
    print(f"{Colors.GREEN}✅ {msg}{Colors.END}")

def print_error(msg):
    print(f"{Colors.RED}❌ {msg}{Colors.END}")

def print_warning(msg):
    print(f"{Colors.YELLOW}⚠️  {msg}{Colors.END}")

def print_info(msg):
    print(f"{Colors.BLUE}ℹ️  {msg}{Colors.END}")

def print_header(msg):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*50}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{msg}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*50}{Colors.END}")

def check_os():
    """Check Ubuntu version"""
    print_header("System Check")
    try:
        with open('/etc/os-release', 'r') as f:
            content = f.read()
            if 'VERSION_ID="22.04"' in content:
                print_success("Ubuntu 22.04 LTS detected")
                return True
            else:
                print_warning("Not Ubuntu 22.04 LTS")
                return False
    except:
        print_error("Cannot detect OS version")
        return False

def check_python():
    """Check Python version"""
    print_header("Python Environment")
    version = sys.version_info
    if version.major == 3 and version.minor >= 10:
        print_success(f"Python {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print_error(f"Python {version.major}.{version.minor}.{version.micro} - Need 3.10+")
        return False

def check_ros2():
    """Check ROS 2 installation"""
    print_header("ROS 2 Environment")
    
    # Check ROS 2 installation directory
    ros2_path = "/opt/ros/humble/setup.bash"
    if not os.path.exists(ros2_path):
        print_error("ROS 2 Humble not found at /opt/ros/humble/")
        return False
    
    try:
        # Try to source ROS 2 and check if ros2 command works
        temp_script = """#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 -h 2>/dev/null | head -1
"""
        result = subprocess.run(
            temp_script,
            shell=True,
            capture_output=True,
            text=True,
            executable='/bin/bash'
        )
        
        if result.returncode == 0 and "usage: ros2" in result.stdout.lower():
            print_success("ROS 2 Humble installed and working")
            return True
        else:
            print_error("ROS 2 command not working properly")
            return False
    except Exception as e:
        print_error(f"ROS 2 check failed: {e}")
        return False

def check_python_packages():
    """Check essential Python packages"""
    print_header("Python Dependencies")
    essential_packages = [
        'pymavlink',
        'cv2',
        'numpy',
        'ultralytics',
        'torch',
        'yaml',
        'serial',
    ]
    
    success_count = 0
    for package in essential_packages:
        try:
            if package == 'cv2':
                importlib.import_module('cv2')
            else:
                importlib.import_module(package)
            print_success(f"{package} - OK")
            success_count += 1
        except ImportError:
            print_error(f"{package} - Missing")
    
    return success_count == len(essential_packages)

def check_hardware():
    """Check hardware connections"""
    print_header("Hardware Detection")
    
    # Check flight controllers
    fc_devices = list(Path('/dev').glob('tty[AU][CS][BM]*'))
    if fc_devices:
        print_success(f"Flight controller: {len(fc_devices)} device(s)")
        for device in fc_devices:
            print_info(f"  - {device}")
    else:
        print_warning("No flight controllers detected")
    
    # Check cameras
    camera_devices = list(Path('/dev').glob('video*'))
    if camera_devices:
        print_success(f"Cameras: {len(camera_devices)} device(s)")
        for device in camera_devices:
            print_info(f"  - {device}")
    else:
        print_warning("No cameras detected")
    
    # Check if we can access GPIO (Raspberry Pi)
    try:
        import RPi.GPIO
        print_success("GPIO interface available")
    except:
        print_warning("GPIO not available (not on Raspberry Pi)")
    
    return True

def main():
    parser = argparse.ArgumentParser(description='KAERTEI 2025 System Validation')
    parser.add_argument('--competition', action='store_true', 
                       help='Run comprehensive competition readiness check')
    parser.add_argument('--quick', action='store_true',
                       help='Run quick validation only')
    
    args = parser.parse_args()
    
    print(f"{Colors.BOLD}{Colors.BLUE}")
    print("🚁 KAERTEI 2025 FAIO System Validation")
    print("=====================================")
    print(f"{Colors.END}")
    
    all_checks_passed = True
    
    # Basic checks
    all_checks_passed &= check_os()
    all_checks_passed &= check_python()
    
    if not args.quick:
        all_checks_passed &= check_ros2()
        all_checks_passed &= check_python_packages()
        all_checks_passed &= check_hardware()
    
    # Final result
    print_header("Validation Result")
    if all_checks_passed:
        print_success("System validation PASSED ✅")
        print_info("System ready for KAERTEI 2025 competition!")
        return 0
    else:
        print_error("System validation FAILED ❌")
        print_info("Please fix the issues above before proceeding.")
        return 1

if __name__ == "__main__":
    sys.exit(main())

import os
import sys
import subprocess
import importlib
import platform
from pathlib import Path
from typing import Dict, List, Tuple

class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_header(title: str):
    print(f"\n{Colors.BLUE}{Colors.BOLD}{'='*60}{Colors.END}")
    print(f"{Colors.BLUE}{Colors.BOLD}{title.center(60)}{Colors.END}")
    print(f"{Colors.BLUE}{Colors.BOLD}{'='*60}{Colors.END}\n")

def success(msg: str):
    print(f"{Colors.GREEN}✅ {msg}{Colors.END}")

def warning(msg: str):
    print(f"{Colors.YELLOW}⚠️  {msg}{Colors.END}")

def error(msg: str):
    print(f"{Colors.RED}❌ {msg}{Colors.END}")

def info(msg: str):
    print(f"{Colors.BLUE}ℹ️  {msg}{Colors.END}")

def check_ubuntu_version() -> bool:
    """Check if running Ubuntu 22.04 LTS"""
    print_header("UBUNTU VERSION CHECK")
    
    try:
        with open('/etc/os-release', 'r') as f:
            os_info = f.read()
        
        if 'Ubuntu' in os_info and '22.04' in os_info:
            success("Ubuntu 22.04 LTS detected")
            return True
        else:
            warning(f"Not Ubuntu 22.04 LTS (detected: {platform.platform()})")
            warning("KAERTEI 2025 is optimized for Ubuntu 22.04 LTS")
            return False
    except Exception as e:
        error(f"Cannot detect OS version: {e}")
        return False

def check_python_version() -> bool:
    """Check Python version compatibility"""
    print_header("PYTHON VERSION CHECK")
    
    python_version = sys.version_info
    info(f"Python version: {python_version.major}.{python_version.minor}.{python_version.micro}")
    
    if python_version >= (3, 8):
        success("Python version compatible")
        return True
    else:
        error("Python 3.8+ required for competition")
        return False

def check_ros2_installation() -> bool:
    """Check ROS 2 Humble installation"""
    print_header("ROS 2 HUMBLE CHECK")
    
    try:
        # Check ROS 2 command
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            success(f"ROS 2 found: {result.stdout.strip()}")
        else:
            error("ROS 2 command not working")
            return False
            
        # Check ROS_DISTRO environment
        ros_distro = os.environ.get('ROS_DISTRO')
        if ros_distro == 'humble':
            success("ROS 2 Humble environment active")
        else:
            warning(f"ROS_DISTRO={ros_distro} (expected: humble)")
            info("Run: source /opt/ros/humble/setup.bash")
            
        # Check key ROS 2 packages
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=10)
        packages = result.stdout
        
        essential_packages = [
            'geometry_msgs', 'sensor_msgs', 'std_msgs', 'nav_msgs',
            'rclpy', 'tf2_ros'
        ]
        
        for package in essential_packages:
            if package in packages:
                success(f"Package found: {package}")
            else:
                error(f"Missing package: {package}")
                
        return True
        
    except subprocess.TimeoutExpired:
        error("ROS 2 command timeout")
        return False
    except FileNotFoundError:
        error("ROS 2 not installed")
        error("Run: just setup-ros2")
        return False
    except Exception as e:
        error(f"ROS 2 check failed: {e}")
        return False

def check_mavros_installation() -> bool:
    """Check MAVROS installation"""
    print_header("MAVROS CHECK")
    
    try:
        # Source ROS 2 environment first
        os.environ['ROS_DISTRO'] = 'humble'
        
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=10)
        packages = result.stdout
        
        mavros_packages = ['mavros', 'mavros_msgs', 'mavros_extras']
        mavros_found = False
        
        for package in mavros_packages:
            if package in packages:
                success(f"MAVROS package found: {package}")
                mavros_found = True
            else:
                warning(f"MAVROS package not found: {package}")
        
        if mavros_found:
            # Check GeographicLib datasets
            geographiclib_path = "/opt/ros/humble/share/GeographicLib"
            if os.path.exists(geographiclib_path):
                success("GeographicLib datasets found")
            else:
                warning("GeographicLib datasets not found")
                info("Run: sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh")
            
            return True
        else:
            error("MAVROS not installed")
            error("Run: just setup-mavros")
            return False
            
    except Exception as e:
        error(f"MAVROS check failed: {e}")
        return False

def check_python_packages() -> bool:
    """Check essential Python packages"""
    print_header("PYTHON PACKAGES CHECK")
    
    essential_packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'pymavlink': 'pymavlink',
        'ultralytics': 'ultralytics',
        'torch': 'torch',
        'scipy': 'scipy',
        'matplotlib': 'matplotlib',
        'yaml': 'PyYAML',
        'serial': 'pyserial',
        'psutil': 'psutil'
    }
    
    all_packages_ok = True
    
    for module_name, package_name in essential_packages.items():
        try:
            module = importlib.import_module(module_name)
            version = getattr(module, '__version__', 'unknown')
            success(f"{package_name}: {version}")
        except ImportError:
            error(f"Missing package: {package_name}")
            all_packages_ok = False
        except Exception as e:
            warning(f"Package check error for {package_name}: {e}")
    
    if not all_packages_ok:
        error("Some Python packages are missing")
        info("Run: pip3 install --user -r requirements.txt")
    
    return all_packages_ok

def check_hardware_connectivity() -> bool:
    """Check hardware device connectivity"""
    print_header("HARDWARE CONNECTIVITY CHECK")
    
    # Check serial devices (Pixhawk/PX4)
    serial_devices = []
    for device_pattern in ['/dev/ttyUSB*', '/dev/ttyACM*']:
        import glob
        devices = glob.glob(device_pattern)
        serial_devices.extend(devices)
    
    if serial_devices:
        success(f"Serial devices found: {len(serial_devices)}")
        for device in serial_devices[:3]:  # Show first 3
            info(f"  {device}")
        
        # Check permissions
        for device in serial_devices[:1]:  # Check first device
            try:
                with open(device, 'r'):
                    success(f"Device accessible: {device}")
            except PermissionError:
                error(f"Permission denied: {device}")
                info("Run: just fix-permissions")
            except Exception:
                pass  # Device might be in use
    else:
        warning("No serial devices found")
        info("Connect Pixhawk/PX4 via USB or use simulation mode")
    
    # Check camera devices
    camera_devices = []
    for i in range(5):  # Check /dev/video0 to /dev/video4
        device = f'/dev/video{i}'
        if os.path.exists(device):
            camera_devices.append(device)
    
    if camera_devices:
        success(f"Camera devices found: {len(camera_devices)}")
        for device in camera_devices:
            info(f"  {device}")
    else:
        warning("No camera devices found")
        info("Connect cameras or use simulation mode")
    
    return len(serial_devices) > 0 or len(camera_devices) > 0

def check_workspace_build() -> bool:
    """Check ROS 2 workspace build"""
    print_header("WORKSPACE BUILD CHECK")
    
    workspace_path = "/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone"
    
    # Check workspace structure
    if not os.path.exists(workspace_path):
        error(f"Workspace not found: {workspace_path}")
        return False
    
    success(f"Workspace found: {workspace_path}")
    
    # Check install directory
    install_path = f"{workspace_path}/install"
    if os.path.exists(install_path):
        success("Install directory found")
    else:
        error("Install directory not found")
        error("Run: just build-workspace")
        return False
    
    # Check kaertei_drone package
    try:
        os.environ['ROS_DISTRO'] = 'humble'
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=10)
        if 'kaertei_drone' in result.stdout:
            success("kaertei_drone package found")
            return True
        else:
            error("kaertei_drone package not found")
            error("Run: just build-workspace")
            return False
    except Exception as e:
        error(f"Package check failed: {e}")
        return False

def check_mission_files() -> bool:
    """Check mission-critical files"""
    print_header("MISSION FILES CHECK")
    
    base_path = "/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone"
    
    critical_files = [
        'checkpoint_mission_node.py',
        'checkpoint_mission_mavros.py',
        'hardware_config.conf',
        'requirements.txt',
        'Justfile',
        'setup_ubuntu22.sh',
        'competition_launch.sh'
    ]
    
    all_files_ok = True
    
    for filename in critical_files:
        filepath = f"{base_path}/{filename}"
        if os.path.exists(filepath):
            success(f"File found: {filename}")
        else:
            error(f"Missing file: {filename}")
            all_files_ok = False
    
    return all_files_ok

def check_competition_readiness() -> Dict[str, bool]:
    """Run comprehensive competition readiness check"""
    results = {}
    
    print(f"{Colors.BOLD}🏆 KAERTEI 2025 FAIO - System Validation{Colors.END}")
    print(f"{Colors.BOLD}Ubuntu 22.04 LTS Competition Readiness Check{Colors.END}")
    
    # Run all checks
    results['ubuntu'] = check_ubuntu_version()
    results['python'] = check_python_version()
    results['ros2'] = check_ros2_installation()
    results['mavros'] = check_mavros_installation()
    results['python_packages'] = check_python_packages()
    results['hardware'] = check_hardware_connectivity()
    results['workspace'] = check_workspace_build()
    results['mission_files'] = check_mission_files()
    
    return results

def print_summary(results: Dict[str, bool]):
    """Print validation summary"""
    print_header("VALIDATION SUMMARY")
    
    passed = sum(results.values())
    total = len(results)
    
    print(f"Tests passed: {passed}/{total}")
    print()
    
    for test_name, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name.upper().replace('_', ' '):<20} {status}")
    
    print()
    
    if passed == total:
        success("🎉 ALL SYSTEMS GO - READY FOR COMPETITION!")
        print()
        info("Next steps:")
        info("1. Connect hardware (Pixhawk, cameras)")
        info("2. Update GPS coordinates in hardware_config.conf")
        info("3. Run: just mission-debug")
        return True
    else:
        error(f"❌ {total - passed} issues found - system not ready")
        print()
        info("Fix the issues above, then run this validation again")
        info("Quick fixes:")
        info("• just setup-full      # Complete setup")
        info("• just fix-permissions # Fix hardware access")
        info("• just build-workspace # Rebuild workspace")
        return False

def main():
    """Main validation function"""
    try:
        results = check_competition_readiness()
        ready = print_summary(results)
        
        # Exit with appropriate code
        sys.exit(0 if ready else 1)
        
    except KeyboardInterrupt:
        print("\n\nValidation interrupted by user")
        sys.exit(1)
    except Exception as e:
        error(f"Validation failed with error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
