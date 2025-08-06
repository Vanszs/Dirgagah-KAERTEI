#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Ubuntu 22.04 System Doctor
Diagnose and fix common competition setup issues
"""

import os
import sys
import subprocess
import platform
from pathlib import Path
import shutil

# Color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    NC = '\033[0m'  # No Color

def print_info(msg): print(f"{Colors.BLUE}[INFO]{Colors.NC} {msg}")
def print_success(msg): print(f"{Colors.GREEN}[SUCCESS]{Colors.NC} {msg}")
def print_warning(msg): print(f"{Colors.YELLOW}[WARNING]{Colors.NC} {msg}")
def print_error(msg): print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}")
def print_step(msg): print(f"{Colors.CYAN}[STEP]{Colors.NC} {msg}")
def print_fix(msg): print(f"{Colors.MAGENTA}[FIX]{Colors.NC} {msg}")

def run_command(cmd, capture_output=True, shell=False):
    """Run a command and return the result"""
    try:
        if shell:
            result = subprocess.run(cmd, shell=True, capture_output=capture_output, text=True)
        else:
            result = subprocess.run(cmd.split(), capture_output=capture_output, text=True)
        return result
    except Exception as e:
        print_error(f"Command failed: {cmd} - {e}")
        return None

def check_system_info():
    """Display system information"""
    print_step("🔍 System Information")
    
    # OS Information
    try:
        with open('/etc/os-release', 'r') as f:
            lines = f.readlines()
        
        os_info = {}
        for line in lines:
            if '=' in line:
                key, value = line.strip().split('=', 1)
                os_info[key] = value.strip('"')
        
        print_info(f"OS: {os_info.get('PRETTY_NAME', 'Unknown')}")
        print_info(f"Kernel: {platform.release()}")
        print_info(f"Architecture: {platform.machine()}")
    except Exception as e:
        print_warning(f"Cannot read OS info: {e}")
    
    # Python version
    print_info(f"Python: {sys.version.split()[0]}")
    
    # Current user and groups
    try:
        import pwd, grp
        user = pwd.getpwuid(os.getuid()).pw_name
        groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
        print_info(f"User: {user}")
        print_info(f"Groups: {', '.join(groups)}")
    except Exception as e:
        print_warning(f"Cannot get user info: {e}")

def diagnose_ros2():
    """Diagnose ROS 2 installation"""
    print_step("🤖 ROS 2 Diagnosis")
    
    # Check if ROS 2 is installed
    result = run_command("ros2 --version")
    if result and result.returncode == 0:
        print_success("✅ ROS 2 command available")
        print_info(f"   Version: {result.stdout.strip()}")
    else:
        print_error("❌ ROS 2 not available")
        print_fix("🔧 Fix: Run 'just setup' or install ROS 2 Humble manually")
        return False
    
    # Check ROS 2 environment
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        if ros_distro == 'humble':
            print_success(f"✅ ROS 2 {ros_distro} environment active")
        else:
            print_warning(f"⚠️  ROS 2 {ros_distro} active (Humble recommended)")
    else:
        print_warning("⚠️  ROS 2 environment not sourced")
        print_fix("🔧 Fix: Run 'source /opt/ros/humble/setup.bash'")
    
    # Check MAVROS
    result = run_command("dpkg -l ros-humble-mavros")
    if result and result.returncode == 0:
        print_success("✅ MAVROS installed")
    else:
        print_error("❌ MAVROS not installed")
        print_fix("🔧 Fix: sudo apt install ros-humble-mavros ros-humble-mavros-extras")
    
    return True

def diagnose_python():
    """Diagnose Python environment"""
    print_step("🐍 Python Environment Diagnosis")
    
    # Check Python version
    version = sys.version_info
    if version >= (3, 8):
        print_success(f"✅ Python {version.major}.{version.minor} OK")
    else:
        print_error(f"❌ Python {version.major}.{version.minor} too old")
        print_fix("🔧 Fix: Upgrade to Python 3.8+")
        return False
    
    # Check virtual environment
    venv_path = Path('/home/vanszs/Documents/ros2/ros2_env')
    if venv_path.exists():
        print_success("✅ Virtual environment found")
        
        # Check if we're in the virtual environment
        if hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix):
            print_success("✅ Virtual environment active")
        else:
            print_warning("⚠️  Virtual environment not active")
            print_fix("🔧 Fix: source ros2_env/bin/activate")
    else:
        print_error("❌ Virtual environment not found")
        print_fix("🔧 Fix: Run 'just setup' to create virtual environment")
    
    # Check essential packages
    packages = ['cv2', 'numpy', 'pymavlink', 'serial', 'yaml']
    missing_packages = []
    
    for package in packages:
        try:
            __import__(package)
            print_success(f"   ✅ {package}")
        except ImportError:
            print_error(f"   ❌ {package} missing")
            missing_packages.append(package)
    
    if missing_packages:
        print_fix(f"🔧 Fix: pip install {' '.join(missing_packages)}")
    
    return len(missing_packages) == 0

def diagnose_workspace():
    """Diagnose ROS 2 workspace"""
    print_step("📁 Workspace Diagnosis")
    
    workspace_path = Path('/home/vanszs/Documents/ros2/ros2_ws')
    
    if not workspace_path.exists():
        print_error("❌ Workspace directory not found")
        print_fix("🔧 Fix: Create workspace or check path")
        return False
    
    # Check workspace structure
    src_path = workspace_path / 'src'
    if src_path.exists():
        print_success("✅ src directory found")
    else:
        print_error("❌ src directory missing")
        print_fix("🔧 Fix: mkdir -p ros2_ws/src")
    
    # Check drone_mvp package
    package_path = src_path / 'drone_mvp'
    if package_path.exists():
        print_success("✅ drone_mvp package found")
    else:
        print_error("❌ drone_mvp package missing")
        print_fix("🔧 Fix: Check if code is in correct location")
        return False
    
    # Check if workspace is built
    install_path = workspace_path / 'install'
    if install_path.exists():
        print_success("✅ Workspace built")
        
        # Check setup files
        setup_file = install_path / 'setup.bash'
        if setup_file.exists():
            print_success("✅ Setup files available")
        else:
            print_warning("⚠️  Setup files incomplete")
    else:
        print_warning("⚠️  Workspace not built")
        print_fix("🔧 Fix: Run 'just build' or 'colcon build'")
    
    return True

def diagnose_hardware():
    """Diagnose hardware connectivity"""
    print_step("🔌 Hardware Diagnosis")
    
    # Check serial devices
    serial_devices = list(Path('/dev').glob('ttyUSB*')) + list(Path('/dev').glob('ttyACM*'))
    if serial_devices:
        print_success(f"✅ Serial devices found: {len(serial_devices)}")
        for device in serial_devices:
            # Check permissions
            try:
                stat = device.stat()
                import grp
                group = grp.getgrgid(stat.st_gid).gr_name
                print_info(f"   📱 {device} (group: {group})")
                
                # Test read access
                if os.access(device, os.R_OK):
                    print_info(f"      ✅ Read access OK")
                else:
                    print_warning(f"      ⚠️  No read access")
            except Exception as e:
                print_warning(f"      ⚠️  Cannot check permissions: {e}")
    else:
        print_warning("⚠️  No serial devices found")
        print_info("   This is OK for dummy mode operation")
    
    # Check user permissions
    try:
        import pwd, grp
        user = pwd.getpwuid(os.getuid()).pw_name
        user_groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
        
        required_groups = ['dialout', 'video']
        missing_groups = [g for g in required_groups if g not in user_groups]
        
        if not missing_groups:
            print_success("✅ User permissions OK")
        else:
            print_error(f"❌ Missing group membership: {', '.join(missing_groups)}")
            print_fix(f"🔧 Fix: sudo usermod -a -G {','.join(missing_groups)} $USER")
            print_fix("   Then log out and back in")
    except Exception as e:
        print_warning(f"⚠️  Cannot check permissions: {e}")
    
    # Check camera devices
    camera_devices = list(Path('/dev').glob('video*'))
    if camera_devices:
        print_success(f"✅ Camera devices found: {len(camera_devices)}")
        for device in camera_devices:
            print_info(f"   📷 {device}")
    else:
        print_warning("⚠️  No camera devices found")
    
    return True

def diagnose_network():
    """Diagnose network connectivity"""
    print_step("🌐 Network Diagnosis")
    
    # Check internet connectivity
    result = run_command("ping -c 1 8.8.8.8")
    if result and result.returncode == 0:
        print_success("✅ Internet connectivity OK")
    else:
        print_error("❌ No internet connectivity")
        print_fix("🔧 Fix: Check network connection")
        return False
    
    # Check DNS resolution
    result = run_command("nslookup google.com")
    if result and result.returncode == 0:
        print_success("✅ DNS resolution OK")
    else:
        print_warning("⚠️  DNS issues detected")
        print_fix("🔧 Fix: Check DNS settings")
    
    return True

def diagnose_disk_space():
    """Check disk space"""
    print_step("💾 Disk Space Diagnosis")
    
    # Check disk usage
    result = run_command("df -h /")
    if result and result.returncode == 0:
        lines = result.stdout.strip().split('\n')
        if len(lines) > 1:
            fields = lines[1].split()
            if len(fields) >= 5:
                used_percent = fields[4].rstrip('%')
                try:
                    used_percent_int = int(used_percent)
                    if used_percent_int < 80:
                        print_success(f"✅ Disk space OK ({used_percent}% used)")
                    elif used_percent_int < 90:
                        print_warning(f"⚠️  Disk space getting low ({used_percent}% used)")
                    else:
                        print_error(f"❌ Disk space critically low ({used_percent}% used)")
                        print_fix("🔧 Fix: Free up disk space")
                except ValueError:
                    print_warning("⚠️  Cannot parse disk usage")
    
    # Check workspace disk usage
    workspace_path = Path('/home/vanszs/Documents/ros2')
    if workspace_path.exists():
        result = run_command(f"du -sh {workspace_path}")
        if result and result.returncode == 0:
            size = result.stdout.split()[0]
            print_info(f"Workspace size: {size}")

def diagnose_processes():
    """Check running processes"""
    print_step("⚙️  Process Diagnosis")
    
    # Check for conflicting processes
    processes_to_check = [
        ('ros2', 'ROS 2 processes'),
        ('mavros', 'MAVROS processes'),
        ('python3.*checkpoint', 'Mission processes'),
        ('gazebo', 'Gazebo simulator'),
        ('qgroundcontrol', 'QGroundControl')
    ]
    
    for process_pattern, description in processes_to_check:
        result = run_command(f"pgrep -f {process_pattern}")
        if result and result.returncode == 0:
            count = len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
            if count > 0:
                print_info(f"🔄 {description}: {count} running")
            else:
                print_info(f"⭕ {description}: none running")
        else:
            print_info(f"⭕ {description}: none running")

def suggest_fixes():
    """Suggest common fixes"""
    print_step("🔧 Common Fix Suggestions")
    
    print_info("Quick fixes to try:")
    print("1. Complete setup reset:")
    print("   just clean && just setup")
    print()
    print("2. Environment issues:")
    print("   source /opt/ros/humble/setup.bash")
    print("   source ros2_env/bin/activate")
    print()
    print("3. Permission issues:")
    print("   sudo usermod -a -G dialout,video $USER")
    print("   (then log out and back in)")
    print()
    print("4. Hardware reset:")
    print("   sudo rmmod usbserial; sudo modprobe usbserial")
    print("   sudo udevadm control --reload-rules")
    print()
    print("5. Process cleanup:")
    print("   pkill -f ros2; pkill -f mavros; pkill -f python3")
    print()
    print("6. Emergency mission launch:")
    print("   python3 checkpoint_mission_node.py --debug")

def main():
    """Main diagnosis function"""
    print(f"{Colors.CYAN}🏥 KAERTEI 2025 FAIO - System Doctor{Colors.NC}")
    print(f"{Colors.CYAN}{'='*40}{Colors.NC}")
    print()
    
    # Change to project directory if possible
    try:
        project_dir = Path('/home/vanszs/Documents/ros2/ros2_ws/src/drone_mvp')
        if project_dir.exists():
            os.chdir(project_dir)
            print_info(f"📁 Working directory: {project_dir}")
        else:
            print_warning("⚠️  Project directory not found")
    except Exception as e:
        print_warning(f"⚠️  Cannot change directory: {e}")
    
    print()
    
    # Run all diagnostics
    diagnostics = [
        ("System Info", check_system_info),
        ("ROS 2", diagnose_ros2),
        ("Python", diagnose_python),
        ("Workspace", diagnose_workspace),
        ("Hardware", diagnose_hardware),
        ("Network", diagnose_network),
        ("Disk Space", diagnose_disk_space),
        ("Processes", diagnose_processes)
    ]
    
    results = []
    
    for name, diagnostic_func in diagnostics:
        try:
            print()
            if diagnostic_func == check_system_info or diagnostic_func == diagnose_processes:
                diagnostic_func()  # These don't return True/False
                results.append((name, True))
            else:
                result = diagnostic_func()
                results.append((name, result))
        except Exception as e:
            print_error(f"❌ {name} diagnostic failed: {e}")
            results.append((name, False))
    
    # Summary
    print()
    print(f"{Colors.CYAN}📋 DIAGNOSIS SUMMARY{Colors.NC}")
    print(f"{Colors.CYAN}{'='*20}{Colors.NC}")
    
    issues_found = 0
    for name, status in results:
        if name in ["System Info", "Processes"]:
            continue  # Skip info-only diagnostics
        
        if status:
            print_success(f"✅ {name}")
        else:
            print_error(f"❌ {name}")
            issues_found += 1
    
    print()
    if issues_found == 0:
        print_success("🎉 No critical issues found!")
        print_info("💡 If you're still having problems, try:")
        print("   • just test          # Run system validation")
        print("   • just mission-debug # Test mission in debug mode")
    else:
        print_error(f"❌ {issues_found} issue(s) found")
        print()
        suggest_fixes()
    
    print()
    print(f"{Colors.CYAN}🏆 KAERTEI 2025 FAIO System Doctor Complete{Colors.NC}")
    
    return issues_found == 0

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print_warning("\n⚠️  Diagnosis interrupted by user")
        sys.exit(1)
    except Exception as e:
        print_error(f"❌ System doctor failed: {e}")
        sys.exit(1)
