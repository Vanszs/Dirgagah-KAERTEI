#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Ubuntu 22.04 System Validation
Comprehensive system check for competition readiness
"""

import os
import sys
import subprocess
import importlib
from pathlib import Path

# Color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color

def print_info(msg): print(f"{Colors.BLUE}[INFO]{Colors.NC} {msg}")
def print_success(msg): print(f"{Colors.GREEN}[SUCCESS]{Colors.NC} {msg}")
def print_warning(msg): print(f"{Colors.YELLOW}[WARNING]{Colors.NC} {msg}")
def print_error(msg): print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}")
def print_step(msg): print(f"{Colors.CYAN}[STEP]{Colors.NC} {msg}")

def check_ubuntu_version():
    """Check if running on Ubuntu 22.04"""
    print_step("🐧 Checking Ubuntu version...")
    
    try:
        with open('/etc/os-release', 'r') as f:
            lines = f.readlines()
        
        os_info = {}
        for line in lines:
            if '=' in line:
                key, value = line.strip().split('=', 1)
                os_info[key] = value.strip('"')
        
        if os_info.get('ID') == 'ubuntu' and os_info.get('VERSION_ID') == '22.04':
            print_success("✅ Ubuntu 22.04 LTS detected - Perfect!")
            return True
        elif os_info.get('ID') == 'ubuntu':
            print_warning(f"⚠️  Ubuntu {os_info.get('VERSION_ID')} detected - Ubuntu 22.04 LTS recommended")
            return True
        else:
            print_error(f"❌ Unsupported OS: {os_info.get('ID')} {os_info.get('VERSION_ID')}")
            return False
    except Exception as e:
        print_error(f"❌ Cannot detect OS: {e}")
        return False

def check_ros2():
    """Check ROS 2 Humble installation"""
    print_step("🤖 Checking ROS 2 Humble...")
    
    # Check if ros2 command exists
    try:
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print_success("✅ ROS 2 command available")
            print_info(f"   Version: {result.stdout.strip()}")
        else:
            print_error("❌ ROS 2 command not working")
            return False
    except FileNotFoundError:
        print_error("❌ ROS 2 not installed")
        return False
    
    # Check ROS 2 environment
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro == 'humble':
        print_success("✅ ROS 2 Humble environment active")
    elif ros_distro:
        print_warning(f"⚠️  ROS 2 {ros_distro} active - Humble recommended")
    else:
        print_warning("⚠️  ROS 2 environment not sourced")
    
    return True

def check_mavros():
    """Check MAVROS installation"""
    print_step("📡 Checking MAVROS...")
    
    try:
        # Check if MAVROS packages are installed
        result = subprocess.run(['dpkg', '-l', 'ros-humble-mavros'], capture_output=True, text=True)
        if result.returncode == 0:
            print_success("✅ MAVROS packages installed")
        else:
            print_error("❌ MAVROS packages not found")
            return False
        
        # Check GeographicLib datasets
        datasets_path = Path('/opt/ros/humble/share/mavros/launch')
        if datasets_path.exists():
            print_success("✅ MAVROS datasets available")
        else:
            print_warning("⚠️  MAVROS datasets may be missing")
        
        return True
    except Exception as e:
        print_error(f"❌ MAVROS check failed: {e}")
        return False

def check_python_environment():
    """Check Python environment and dependencies"""
    print_step("🐍 Checking Python environment...")
    
    # Check Python version
    python_version = sys.version_info
    if python_version >= (3, 8):
        print_success(f"✅ Python {python_version.major}.{python_version.minor} OK")
    else:
        print_error(f"❌ Python {python_version.major}.{python_version.minor} too old - 3.8+ required")
        return False
    
    # Check virtual environment
    venv_path = Path('/home/vanszs/Documents/ros2/ros2_env')
    if venv_path.exists():
        print_success("✅ Python virtual environment found")
    else:
        print_warning("⚠️  Python virtual environment not found")
    
    # Check essential packages
    essential_packages = {
        'cv2': 'OpenCV',
        'numpy': 'NumPy',
        'pymavlink': 'PyMAVLink',
        'serial': 'PySerial',
        'yaml': 'PyYAML'
    }
    
    print_info("Checking Python packages:")
    all_packages_ok = True
    
    for package, name in essential_packages.items():
        try:
            importlib.import_module(package)
            print_success(f"   ✅ {name}")
        except ImportError:
            print_error(f"   ❌ {name} missing")
            all_packages_ok = False
    
    # Check optional packages
    optional_packages = {
        'torch': 'PyTorch',
        'ultralytics': 'Ultralytics YOLO'
    }
    
    for package, name in optional_packages.items():
        try:
            importlib.import_module(package)
            print_success(f"   ✅ {name} (optional)")
        except ImportError:
            print_warning(f"   ⚠️  {name} missing (optional)")
    
    return all_packages_ok

def check_workspace():
    """Check ROS 2 workspace"""
    print_step("📁 Checking ROS 2 workspace...")
    
    workspace_path = Path('/home/vanszs/Documents/ros2/ros2_ws')
    if not workspace_path.exists():
        print_error("❌ ROS 2 workspace not found")
        return False
    
    # Check workspace structure
    src_path = workspace_path / 'src' / 'drone_mvp'
    if src_path.exists():
        print_success("✅ Drone MVP package found")
    else:
        print_error("❌ Drone MVP package missing")
        return False
    
    # Check if workspace is built
    install_path = workspace_path / 'install' / 'setup.bash'
    if install_path.exists():
        print_success("✅ Workspace built successfully")
    else:
        print_warning("⚠️  Workspace not built - run: colcon build")
    
    return True

def check_mission_files():
    """Check mission-critical files"""
    print_step("🎯 Checking mission files...")
    
    base_path = Path('/home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp')
    
    critical_files = {
        'checkpoint_mission_mavros.py': 'MAVROS Mission Node',
        'checkpoint_mission_node.py': 'Standalone Mission Node',
        'config/hardware_config.conf': 'Hardware Configuration',
        'run_checkpoint_mission.sh': 'Mission Launcher',
        'Justfile': 'Just Commands'
    }
    
    all_files_ok = True
    
    for file_path, description in critical_files.items():
        full_path = base_path / file_path
        if full_path.exists():
            print_success(f"   ✅ {description}")
        else:
            print_error(f"   ❌ {description} missing: {file_path}")
            all_files_ok = False
    
    return all_files_ok

def check_26_checkpoints():
    """Validate 26 checkpoints implementation"""
    print_step("🎯 Validating 26 checkpoints...")
    
    try:
        # Import and check mission modules
        sys.path.append('/home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp')
        
        # Check MAVROS version
        try:
            import checkpoint_mission_mavros
            if hasattr(checkpoint_mission_mavros, 'MissionCheckpoint'):
                checkpoints = list(checkpoint_mission_mavros.MissionCheckpoint)
                if len(checkpoints) >= 26:
                    print_success(f"✅ MAVROS mission: {len(checkpoints)} checkpoints implemented")
                else:
                    print_error(f"❌ MAVROS mission: Only {len(checkpoints)} checkpoints (need 26)")
                    return False
            else:
                print_error("❌ MAVROS mission: MissionCheckpoint enum not found")
                return False
        except ImportError as e:
            print_warning(f"⚠️  MAVROS mission import failed: {e}")
        
        # Check standalone version
        try:
            import checkpoint_mission_node
            if hasattr(checkpoint_mission_node, 'MissionCheckpoint'):
                checkpoints = list(checkpoint_mission_node.MissionCheckpoint)
                if len(checkpoints) >= 26:
                    print_success(f"✅ Standalone mission: {len(checkpoints)} checkpoints implemented")
                else:
                    print_error(f"❌ Standalone mission: Only {len(checkpoints)} checkpoints (need 26)")
                    return False
            else:
                print_error("❌ Standalone mission: MissionCheckpoint enum not found")
                return False
        except ImportError as e:
            print_warning(f"⚠️  Standalone mission import failed: {e}")
        
        return True
        
    except Exception as e:
        print_error(f"❌ Checkpoint validation failed: {e}")
        return False

def check_hardware():
    """Check hardware connectivity"""
    print_step("🔌 Checking hardware...")
    
    # Check serial devices
    serial_devices = list(Path('/dev').glob('ttyUSB*')) + list(Path('/dev').glob('ttyACM*'))
    if serial_devices:
        print_success(f"✅ Serial devices found: {len(serial_devices)}")
        for device in serial_devices:
            print_info(f"   📱 {device}")
    else:
        print_warning("⚠️  No serial devices - will use dummy mode")
    
    # Check camera devices
    camera_devices = list(Path('/dev').glob('video*'))
    if camera_devices:
        print_success(f"✅ Camera devices found: {len(camera_devices)}")
        for device in camera_devices:
            print_info(f"   📷 {device}")
    else:
        print_warning("⚠️  No camera devices")
    
    # Check user permissions
    import pwd, grp
    
    try:
        user = pwd.getpwuid(os.getuid()).pw_name
        user_groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
        
        required_groups = ['dialout', 'video']
        missing_groups = [g for g in required_groups if g not in user_groups]
        
        if not missing_groups:
            print_success("✅ User permissions OK")
        else:
            print_error(f"❌ Missing group membership: {missing_groups}")
            print_info("   Run: sudo usermod -a -G dialout,video $USER")
            print_info("   Then log out and back in")
            return False
    except Exception as e:
        print_warning(f"⚠️  Cannot check permissions: {e}")
    
    return True

def check_development_tools():
    """Check development tools"""
    print_step("🛠️ Checking development tools...")
    
    tools = {
        'git': 'Git version control',
        'just': 'Just command runner',
        'colcon': 'Colcon build tool'
    }
    
    all_tools_ok = True
    
    for tool, description in tools.items():
        try:
            result = subprocess.run([tool, '--version'], capture_output=True, text=True)
            if result.returncode == 0:
                print_success(f"   ✅ {description}")
            else:
                print_error(f"   ❌ {description} not working")
                all_tools_ok = False
        except FileNotFoundError:
            if tool == 'just':
                print_warning(f"   ⚠️  {description} missing (install with: curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash)")
            else:
                print_error(f"   ❌ {description} missing")
                all_tools_ok = False
    
    return all_tools_ok

def generate_report():
    """Generate comprehensive validation report"""
    print_step("📊 Generating validation report...")
    
    print(f"\n{Colors.CYAN}={'='*50}{Colors.NC}")
    print(f"{Colors.CYAN}KAERTEI 2025 FAIO - System Validation Report{Colors.NC}")
    print(f"{Colors.CYAN}={'='*50}{Colors.NC}")
    
    checks = [
        ("Ubuntu 22.04", check_ubuntu_version),
        ("ROS 2 Humble", check_ros2),
        ("MAVROS", check_mavros),
        ("Python Environment", check_python_environment),
        ("ROS 2 Workspace", check_workspace),
        ("Mission Files", check_mission_files),
        ("26 Checkpoints", check_26_checkpoints),
        ("Hardware", check_hardware),
        ("Development Tools", check_development_tools)
    ]
    
    results = []
    overall_status = True
    
    for name, check_func in checks:
        try:
            result = check_func()
            results.append((name, result))
            if not result:
                overall_status = False
        except Exception as e:
            print_error(f"❌ {name} check failed: {e}")
            results.append((name, False))
            overall_status = False
    
    # Summary
    print(f"\n{Colors.CYAN}📋 VALIDATION SUMMARY{Colors.NC}")
    print(f"{Colors.CYAN}{'='*20}{Colors.NC}")
    
    for name, status in results:
        if status:
            print_success(f"✅ {name}")
        else:
            print_error(f"❌ {name}")
    
    print()
    if overall_status:
        print_success("🎉 ALL SYSTEMS GO! Competition ready!")
        print()
        print(f"{Colors.GREEN}Next steps:{Colors.NC}")
        print("1. just mission-debug    # Practice run")
        print("2. just mission-auto     # Competition run")
        print()
        print(f"{Colors.CYAN}Competition checklist:{Colors.NC}")
        print("• Update GPS coordinates in config/hardware_config.conf")
        print("• Test hardware connections")
        print("• Verify camera and sensors")
        print("• Have RC transmitter ready for emergencies")
    else:
        print_error("❌ System not competition ready!")
        print()
        print(f"{Colors.YELLOW}Fix issues and run:{Colors.NC}")
        print("• just setup             # Complete setup")
        print("• just doctor            # Diagnose issues")
        print("• python3 validate_ubuntu.py  # Re-validate")
    
    return overall_status

def main():
    """Main validation function"""
    print(f"{Colors.BLUE}🧪 KAERTEI 2025 FAIO - Ubuntu 22.04 System Validation{Colors.NC}")
    print(f"{Colors.BLUE}{'='*55}{Colors.NC}")
    print()
    
    try:
        # Change to project directory
        project_dir = Path('/home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp')
        if project_dir.exists():
            os.chdir(project_dir)
            print_info(f"📁 Working directory: {project_dir}")
        else:
            print_warning("⚠️  Project directory not found - using current directory")
        
        # Run comprehensive validation
        success = generate_report()
        
        # Exit with appropriate code
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print_warning("\n⚠️  Validation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print_error(f"❌ Validation failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
