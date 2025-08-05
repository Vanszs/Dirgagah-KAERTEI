#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Debug System v2
Advanced debugging with legacy compatibility and USB error handling
"""

import os
import sys
import time
import json
import subprocess
import traceback
from datetime import datetime
from typing import Dict, Any, List, Optional
from pathlib import Path

# Import dummy hardware for non-USB testing
try:
    from dummy_hardware import DummySystemChecker, DummyMAVLink, DummyCamera
    DUMMY_AVAILABLE = True
except ImportError:
    DUMMY_AVAILABLE = False

class DebugLogger:
    """Enhanced logging system for debug v2"""
    
    def __init__(self, log_file: str = "debug_v2.log"):
        self.log_file = log_file
        self.start_time = datetime.now()
        
        # Create logs directory
        log_dir = Path("logs")
        log_dir.mkdir(exist_ok=True)
        self.log_path = log_dir / log_file
        
        self.log("🔧 Debug System v2 initialized")
    
    def log(self, message: str, level: str = "INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] [{level}] {message}"
        
        print(log_entry)
        
        with open(self.log_path, "a") as f:
            f.write(log_entry + "\n")
    
    def error(self, message: str, exception: Exception = None):
        """Log error with traceback"""
        self.log(f"❌ {message}", "ERROR")
        if exception:
            self.log(f"Exception: {str(exception)}", "ERROR")
            self.log(f"Traceback: {traceback.format_exc()}", "ERROR")
    
    def warning(self, message: str):
        """Log warning"""
        self.log(f"⚠️  {message}", "WARN")
    
    def success(self, message: str):
        """Log success"""
        self.log(f"✅ {message}", "SUCCESS")
    
    def info(self, message: str):
        """Log info"""
        self.log(f"ℹ️  {message}", "INFO")

class USBErrorHandler:
    """Handle USB connection errors gracefully"""
    
    def __init__(self, logger: DebugLogger):
        self.logger = logger
        self.usb_devices = {}
        self.fallback_mode = False
    
    def detect_usb_devices(self) -> Dict[str, Any]:
        """Detect available USB devices"""
        devices = {}
        
        try:
            # Check for ttyACM devices (flight controller)
            acm_devices = []
            for i in range(10):  # Check ttyACM0-9
                device = f"/dev/ttyACM{i}"
                if os.path.exists(device):
                    acm_devices.append(device)
            
            devices['flight_controller'] = {
                'available': acm_devices,
                'count': len(acm_devices),
                'status': len(acm_devices) > 0
            }
            
            # Check for USB cameras (video devices)
            video_devices = []
            for i in range(10):  # Check video0-9
                device = f"/dev/video{i}"
                if os.path.exists(device):
                    video_devices.append(device)
            
            devices['cameras'] = {
                'available': video_devices,
                'count': len(video_devices),
                'status': len(video_devices) > 0
            }
            
            # Check USB permissions
            permission_issues = []
            for device_type, info in devices.items():
                for device in info['available']:
                    if not os.access(device, os.R_OK | os.W_OK):
                        permission_issues.append(device)
            
            devices['permissions'] = {
                'issues': permission_issues,
                'status': len(permission_issues) == 0
            }
            
            self.logger.success(f"USB scan complete: {sum(info['count'] for info in devices.values() if 'count' in info)} devices found")
            
        except Exception as e:
            self.logger.error("USB device detection failed", e)
            devices['error'] = str(e)
        
        self.usb_devices = devices
        return devices
    
    def check_usb_permissions(self) -> bool:
        """Check and fix USB permissions"""
        try:
            # Add user to dialout group for serial access
            result = subprocess.run(['groups'], capture_output=True, text=True)
            groups = result.stdout.strip()
            
            if 'dialout' not in groups:
                self.logger.warning("User not in dialout group - USB serial access may fail")
                self.logger.info("Run: sudo usermod -a -G dialout $USER && newgrp dialout")
                return False
            
            # Check video group for camera access
            if 'video' not in groups:
                self.logger.warning("User not in video group - USB camera access may fail")
                self.logger.info("Run: sudo usermod -a -G video $USER && newgrp video")
                return False
            
            self.logger.success("USB permissions OK")
            return True
            
        except Exception as e:
            self.logger.error("Permission check failed", e)
            return False
    
    def enable_fallback_mode(self):
        """Enable fallback mode for testing without USB"""
        self.fallback_mode = True
        self.logger.warning("USB fallback mode enabled - using dummy hardware")
    
    def get_connection_strategy(self, device_type: str) -> Dict[str, Any]:
        """Get connection strategy based on USB availability"""
        if self.fallback_mode or not DUMMY_AVAILABLE:
            return {'mode': 'dummy', 'message': 'Using dummy hardware'}
        
        devices = self.usb_devices.get(device_type, {})
        
        if devices.get('status', False):
            return {
                'mode': 'usb',
                'devices': devices['available'],
                'message': f"USB {device_type} available"
            }
        else:
            return {
                'mode': 'dummy',
                'message': f"No USB {device_type} found, using dummy"
            }

class LegacyDebugSupport:
    """Support for legacy debug system"""
    
    def __init__(self, logger: DebugLogger):
        self.logger = logger
        self.legacy_methods = {}
        self.load_legacy_configs()
    
    def load_legacy_configs(self):
        """Load legacy debug configurations"""
        legacy_config = {
            'mavros_topics': [
                '/mavros/state',
                '/mavros/local_position/pose',
                '/mavros/global_position/global',
                '/mavros/battery',
                '/mavros/imu/data'
            ],
            'debug_levels': ['DEBUG', 'INFO', 'WARN', 'ERROR'],
            'hardware_checks': [
                'flight_controller',
                'cameras',
                'gps',
                'sensors'
            ]
        }
        
        self.legacy_methods = legacy_config
        self.logger.info("Legacy debug configurations loaded")
    
    def run_legacy_checks(self) -> Dict[str, Any]:
        """Run legacy compatibility checks"""
        results = {}
        
        # Check ROS 2 topics (legacy method)
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                mavros_topics = [t for t in topics if 'mavros' in t]
                
                results['ros2_topics'] = {
                    'total': len(topics),
                    'mavros': len(mavros_topics),
                    'status': len(topics) > 0
                }
            else:
                results['ros2_topics'] = {
                    'status': False,
                    'error': 'ROS 2 topic list failed'
                }
        except Exception as e:
            results['ros2_topics'] = {
                'status': False,
                'error': str(e)
            }
        
        # Check ROS 2 nodes (legacy method)
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                results['ros2_nodes'] = {
                    'count': len([n for n in nodes if n.strip()]),
                    'status': len(nodes) > 0
                }
            else:
                results['ros2_nodes'] = {
                    'status': False,
                    'error': 'ROS 2 node list failed'
                }
        except Exception as e:
            results['ros2_nodes'] = {
                'status': False,
                'error': str(e)
            }
        
        return results

class DebugSystemV2:
    """Main debug system v2 with USB error handling"""
    
    def __init__(self):
        self.logger = DebugLogger()
        self.usb_handler = USBErrorHandler(self.logger)
        self.legacy_support = LegacyDebugSupport(self.logger)
        
        # Check for environment variables to force modes
        usb_mode = os.environ.get('USB_MODE', 'auto').lower()
        if usb_mode == 'dummy':
            self.usb_handler.enable_fallback_mode()
            self.logger.info("Forced dummy mode from environment")
        elif usb_mode == 'real':
            self.logger.info("Forced real hardware mode from environment")
        
        self.logger.log("🔧 KAERTEI 2025 Debug System v2 starting...")
    
    def comprehensive_system_check(self) -> Dict[str, Any]:
        """Run comprehensive system check with USB error handling"""
        results = {
            'timestamp': datetime.now().isoformat(),
            'system': 'debug_v2',
            'checks': {}
        }
        
        # 1. USB Device Check
        self.logger.info("1. Checking USB devices...")
        usb_devices = self.usb_handler.detect_usb_devices()
        results['checks']['usb_devices'] = usb_devices
        
        # 2. USB Permissions
        self.logger.info("2. Checking USB permissions...")
        permissions_ok = self.usb_handler.check_usb_permissions()
        results['checks']['usb_permissions'] = {'status': permissions_ok}
        
        # 3. Determine connection strategy
        if not any(info.get('status', False) for info in usb_devices.values() if isinstance(info, dict)):
            self.logger.warning("No USB devices available - enabling fallback mode")
            self.usb_handler.enable_fallback_mode()
        
        # 4. Test flight controller connection
        self.logger.info("3. Testing flight controller connection...")
        fc_strategy = self.usb_handler.get_connection_strategy('flight_controller')
        results['checks']['flight_controller'] = self.test_flight_controller(fc_strategy)
        
        # 5. Test camera connections
        self.logger.info("4. Testing camera connections...")
        cam_strategy = self.usb_handler.get_connection_strategy('cameras')
        results['checks']['cameras'] = self.test_cameras(cam_strategy)
        
        # 6. Legacy system checks
        self.logger.info("5. Running legacy compatibility checks...")
        legacy_results = self.legacy_support.run_legacy_checks()
        results['checks']['legacy'] = legacy_results
        
        # 7. Python dependencies
        self.logger.info("6. Checking Python dependencies...")
        results['checks']['python_deps'] = self.check_python_dependencies()
        
        # 8. ROS 2 workspace
        self.logger.info("7. Checking ROS 2 workspace...")
        results['checks']['ros2_workspace'] = self.check_ros2_workspace()
        
        return results
    
    def test_flight_controller(self, strategy: Dict[str, Any]) -> Dict[str, Any]:
        """Test flight controller connection with fallback"""
        try:
            if strategy['mode'] == 'usb':
                # Try real USB connection
                import serial
                for device in strategy['devices']:
                    try:
                        with serial.Serial(device, 57600, timeout=1) as ser:
                            self.logger.success(f"Flight controller connected: {device}")
                            return {
                                'status': True,
                                'mode': 'usb',
                                'device': device,
                                'message': 'Real hardware connected'
                            }
                    except Exception as e:
                        self.logger.warning(f"Failed to connect to {device}: {e}")
                        continue
                
                # USB devices exist but connection failed
                self.logger.warning("USB devices found but connection failed - using dummy")
                return self.test_dummy_flight_controller()
            
            else:
                # Use dummy mode
                return self.test_dummy_flight_controller()
                
        except Exception as e:
            self.logger.error("Flight controller test failed", e)
            return {'status': False, 'error': str(e)}
    
    def test_dummy_flight_controller(self) -> Dict[str, Any]:
        """Test dummy flight controller"""
        if not DUMMY_AVAILABLE:
            return {'status': False, 'error': 'Dummy hardware not available'}
        
        try:
            dummy_fc = DummyMAVLink("test")
            # Fix: Create heartbeat properly
            heartbeat = dummy_fc.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            dummy_fc.close()
            
            return {
                'status': True,
                'mode': 'dummy',
                'message': 'Dummy flight controller working'
            }
        except Exception as e:
            return {'status': False, 'error': f'Dummy FC failed: {e}'}
    
    def test_cameras(self, strategy: Dict[str, Any]) -> Dict[str, Any]:
        """Test camera connections with fallback"""
        results = {'cameras': [], 'working_count': 0}
        
        try:
            if strategy['mode'] == 'usb':
                # Try real cameras
                try:
                    import cv2
                    for i, device in enumerate(strategy['devices']):
                        try:
                            # Extract device number from /dev/videoX
                            cam_id = int(device.split('video')[1])
                            cap = cv2.VideoCapture(cam_id)
                            
                            if cap.isOpened():
                                ret, frame = cap.read()
                                if ret:
                                    results['cameras'].append({
                                        'id': cam_id,
                                        'device': device,
                                        'status': True,
                                        'mode': 'usb'
                                    })
                                    results['working_count'] += 1
                                    self.logger.success(f"Camera {cam_id} working")
                                else:
                                    results['cameras'].append({
                                        'id': cam_id,
                                        'device': device,
                                        'status': False,
                                        'error': 'No frame received'
                                    })
                            cap.release()
                        except Exception as e:
                            self.logger.warning(f"Camera {device} failed: {e}")
                            continue
                except ImportError:
                    self.logger.warning("OpenCV not available - will use dummy cameras")
                    results['error'] = 'OpenCV not installed'
            
            # If no real cameras or all failed, use dummy
            if results['working_count'] == 0 and DUMMY_AVAILABLE:
                for i in range(3):  # Test 3 dummy cameras
                    dummy_cam = DummyCamera(i)
                    ret, frame = dummy_cam.read()
                    dummy_cam.release()
                    
                    if ret:
                        results['cameras'].append({
                            'id': i,
                            'status': True,
                            'mode': 'dummy'
                        })
                        results['working_count'] += 1
            
            results['status'] = results['working_count'] > 0
            return results
            
        except Exception as e:
            self.logger.error("Camera test failed", e)
            return {'status': False, 'error': str(e)}
    
    def check_python_dependencies(self) -> Dict[str, Any]:
        """Check Python dependencies"""
        deps = {
            'pymavlink': 'pymavlink',
            'opencv': 'cv2',
            'numpy': 'numpy',
            'ultralytics': 'ultralytics'
        }
        
        results = {}
        for name, module in deps.items():
            try:
                __import__(module)
                results[name] = {'status': True, 'available': True}
            except ImportError:
                results[name] = {'status': False, 'available': False}
        
        return results
    
    def check_ros2_workspace(self) -> Dict[str, Any]:
        """Check ROS 2 workspace status"""
        workspace_path = Path("/home/vanszs/Documents/ros2/ros2_ws")
        
        checks = {
            'workspace_exists': workspace_path.exists(),
            'src_exists': (workspace_path / "src").exists(),
            'build_exists': (workspace_path / "build").exists(),
            'install_exists': (workspace_path / "install").exists(),
            'drone_mvp_exists': (workspace_path / "src" / "drone_mvp").exists()
        }
        
        # Check if package is built
        if checks['install_exists']:
            install_path = workspace_path / "install" / "drone_mvp"
            checks['drone_mvp_built'] = install_path.exists()
        else:
            checks['drone_mvp_built'] = False
        
        return {
            'status': all(checks.values()),
            'checks': checks
        }
    
    def generate_report(self, results: Dict[str, Any]) -> str:
        """Generate comprehensive debug report"""
        report = []
        report.append("=" * 60)
        report.append("🔧 KAERTEI 2025 DEBUG SYSTEM V2 REPORT")
        report.append("=" * 60)
        report.append(f"Timestamp: {results['timestamp']}")
        report.append(f"System: {results['system']}")
        report.append("")
        
        # Summary
        total_checks = len(results['checks'])
        passed_checks = sum(1 for check in results['checks'].values() 
                          if isinstance(check, dict) and check.get('status', False))
        
        report.append(f"📊 SUMMARY: {passed_checks}/{total_checks} checks passed")
        report.append("")
        
        # Detailed results
        for check_name, check_result in results['checks'].items():
            if isinstance(check_result, dict):
                status = "✅" if check_result.get('status', False) else "❌"
                report.append(f"{status} {check_name.upper().replace('_', ' ')}")
                
                if 'message' in check_result:
                    report.append(f"   {check_result['message']}")
                
                if 'error' in check_result:
                    report.append(f"   Error: {check_result['error']}")
                
                report.append("")
        
        # Recommendations
        report.append("💡 RECOMMENDATIONS:")
        if results['checks'].get('usb_devices', {}).get('status', False):
            report.append("✅ USB devices detected - hardware mode available")
        else:
            report.append("⚠️  No USB devices - using dummy mode for testing")
        
        if not results['checks'].get('usb_permissions', {}).get('status', False):
            report.append("🔧 Fix USB permissions: sudo usermod -a -G dialout,video $USER")
        
        report.append("")
        report.append("🚀 NEXT STEPS:")
        report.append("1. ./dummy_hardware.py - Test dummy system")
        report.append("2. ./run_checkpoint_mission.sh - Run mission")
        report.append("3. ./arch_build_fix.sh - Fix Arch build if needed")
        
        return "\n".join(report)
    
    def save_report(self, results: Dict[str, Any]):
        """Save debug report to file"""
        report = self.generate_report(results)
        
        # Save to file
        report_file = Path("logs") / f"debug_v2_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(report_file, 'w') as f:
            f.write(report)
        
        # Also save JSON
        json_file = Path("logs") / f"debug_v2_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(json_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.logger.success(f"Report saved: {report_file}")
        self.logger.success(f"Data saved: {json_file}")
        
        return report

def main():
    """Main debug system v2 entry point"""
    try:
        debug_system = DebugSystemV2()
        
        print("🚁 KAERTEI 2025 FAIO - Debug System v2")
        print("=====================================")
        print("🔧 Advanced debugging with USB error handling")
        print("🔧 Legacy compatibility maintained")
        print("🔧 Dummy hardware fallback available")
        print()
        
        # Run comprehensive check
        results = debug_system.comprehensive_system_check()
        
        # Generate and display report
        report = debug_system.save_report(results)
        print(report)
        
        # Determine exit code
        total_critical_checks = ['flight_controller', 'python_deps', 'ros2_workspace']
        critical_passed = sum(1 for check in total_critical_checks 
                            if results['checks'].get(check, {}).get('status', False))
        
        if critical_passed >= len(total_critical_checks) * 0.8:
            print("\n🎉 SYSTEM READY FOR TESTING!")
            return 0
        else:
            print("\n🚨 CRITICAL ISSUES DETECTED")
            return 1
            
    except Exception as e:
        print(f"❌ Debug system v2 failed: {e}")
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())
