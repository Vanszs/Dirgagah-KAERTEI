#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Hardware Dummy System
Dummy hardware interface for testing without actual USB connections
"""

import time
import random
import json
from typing import Dict, Any, Optional, Tuple

class DummyMAVLink:
    """Dummy MAVLink connection for testing without hardware"""
    
    def __init__(self, device_name: str = "dummy"):
        self.device_name = device_name
        self.connected = True
        self.armed = False
        self.mode = "STABILIZE"
        self.gps_fix = 3
        self.battery_voltage = 16.8
        self.altitude = 0.0
        self.lat = -6.2088  # Jakarta coordinates for testing
        self.lon = 106.8456
        
        print(f"🔧 [DUMMY] MAVLink connection to {device_name} - SIMULATED")
        
    def recv_match(self, type=None, blocking=False, timeout=1):
        """Simulate receiving MAVLink messages"""
        if not blocking:
            return None
            
        # Simulate different message types
        if type == "HEARTBEAT":
            return self._create_heartbeat()
        elif type == "GPS_RAW_INT":
            return self._create_gps_message()
        elif type == "BATTERY_STATUS":
            return self._create_battery_message()
        elif type == "GLOBAL_POSITION_INT":
            return self._create_position_message()
        else:
            return None
    
    def _create_heartbeat(self):
        """Create dummy heartbeat message"""
        class DummyMsg:
            def __init__(self):
                self.type = 0  # MAV_TYPE_GENERIC
                self.autopilot = 3  # MAV_AUTOPILOT_ARDUPILOTMEGA
                self.base_mode = 81 if self.armed else 17
                self.custom_mode = 0
                self.system_status = 4  # MAV_STATE_ACTIVE
                
        msg = DummyMsg()
        msg.armed = self.armed
        return msg
    
    def _create_gps_message(self):
        """Create dummy GPS message"""
        class DummyGPS:
            def __init__(self):
                self.fix_type = 3  # 3D fix
                self.lat = int(-6.2088 * 1e7)  # Convert to int32
                self.lon = int(106.8456 * 1e7)
                self.alt = int(50 * 1000)  # 50m in mm
                self.satellites_visible = 12
                
        return DummyGPS()
    
    def _create_battery_message(self):
        """Create dummy battery message"""
        class DummyBattery:
            def __init__(self):
                self.voltages = [int(self.battery_voltage * 1000)] * 6
                self.current_battery = 5000  # 5A in cA
                self.battery_remaining = 85
                
        return DummyBattery()
    
    def _create_position_message(self):
        """Create dummy position message"""
        class DummyPosition:
            def __init__(self):
                self.lat = int(self.lat * 1e7)
                self.lon = int(self.lon * 1e7)
                self.alt = int(self.altitude * 1000)
                self.relative_alt = int(self.altitude * 1000)
                self.vx = 0  # velocity
                self.vy = 0
                self.vz = 0
                self.hdg = 0  # heading
                
        return DummyPosition()
    
    def mav_cmd_do_set_mode_send(self, target_system, mode):
        """Simulate mode change"""
        modes = ["STABILIZE", "GUIDED", "AUTO", "LAND", "RTL"]
        if mode in range(len(modes)):
            self.mode = modes[mode]
            print(f"🔧 [DUMMY] Mode changed to: {self.mode}")
            return True
        return False
    
    def arducopter_arm(self):
        """Simulate arming"""
        self.armed = True
        print("🔧 [DUMMY] Vehicle ARMED")
        return True
    
    def arducopter_disarm(self):
        """Simulate disarming"""
        self.armed = False
        print("🔧 [DUMMY] Vehicle DISARMED")
        return True
    
    def close(self):
        """Close dummy connection"""
        self.connected = False
        print("🔧 [DUMMY] Connection closed")

class DummyCamera:
    """Dummy camera interface for testing without hardware"""
    
    def __init__(self, camera_id: int = 0):
        self.camera_id = camera_id
        self.is_opened = True
        self.frame_count = 0
        
        print(f"🔧 [DUMMY] Camera {camera_id} initialized")
    
    def read(self):
        """Return dummy frame"""
        import numpy as np
        
        # Create dummy frame (640x480, 3 channels)
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Add some fake objects for detection testing
        if self.frame_count % 30 == 0:  # Every 30 frames
            # Draw a fake "object" (rectangle)
            frame[100:200, 200:300] = [255, 0, 0]  # Red rectangle
        
        self.frame_count += 1
        return True, frame
    
    def release(self):
        """Release dummy camera"""
        self.is_opened = False
        print(f"🔧 [DUMMY] Camera {self.camera_id} released")
    
    def isOpened(self):
        """Check if camera is opened"""
        return self.is_opened

class DummyToFSensor:
    """Dummy Time-of-Flight sensor for distance measurement"""
    
    def __init__(self, sensor_id: str):
        self.sensor_id = sensor_id
        self.connected = True
        
        print(f"🔧 [DUMMY] ToF Sensor {sensor_id} initialized")
    
    def get_distance(self) -> float:
        """Return dummy distance measurement"""
        # Simulate distance between 0.1m to 5.0m
        base_distance = 2.0
        noise = random.uniform(-0.5, 0.5)
        return max(0.1, base_distance + noise)
    
    def close(self):
        """Close sensor connection"""
        self.connected = False
        print(f"🔧 [DUMMY] ToF Sensor {self.sensor_id} closed")

class DummyElectromagnet:
    """Dummy electromagnet controller"""
    
    def __init__(self, relay_pin: int):
        self.relay_pin = relay_pin
        self.is_active = False
        
        print(f"🔧 [DUMMY] Electromagnet on pin {relay_pin} initialized")
    
    def activate(self):
        """Activate electromagnet"""
        self.is_active = True
        print(f"🔧 [DUMMY] Electromagnet {self.relay_pin} ACTIVATED")
    
    def deactivate(self):
        """Deactivate electromagnet"""
        self.is_active = False
        print(f"🔧 [DUMMY] Electromagnet {self.relay_pin} DEACTIVATED")
    
    def status(self) -> bool:
        """Get electromagnet status"""
        return self.is_active

class DummySystemChecker:
    """Comprehensive dummy system checker"""
    
    def __init__(self):
        self.systems = {
            'mavlink': False,
            'cameras': False,
            'tof_sensors': False,
            'electromagnets': False,
            'gps': False,
            'ros2': False
        }
    
    def check_all_systems(self) -> Dict[str, Any]:
        """Check all dummy systems"""
        results = {}
        
        print("🔧 [DUMMY] Running comprehensive system check...")
        
        # Check MAVLink
        try:
            mavlink = DummyMAVLink("test")
            mavlink.close()
            results['mavlink'] = {'status': True, 'message': 'Dummy MAVLink OK'}
            self.systems['mavlink'] = True
        except Exception as e:
            results['mavlink'] = {'status': False, 'message': f'MAVLink error: {e}'}
        
        # Check Cameras
        try:
            cameras_ok = 0
            for i in range(3):  # Test 3 cameras
                cam = DummyCamera(i)
                ret, frame = cam.read()
                if ret:
                    cameras_ok += 1
                cam.release()
            
            results['cameras'] = {
                'status': cameras_ok > 0,
                'count': cameras_ok,
                'message': f'{cameras_ok}/3 dummy cameras OK'
            }
            self.systems['cameras'] = cameras_ok > 0
        except Exception as e:
            results['cameras'] = {'status': False, 'message': f'Camera error: {e}'}
        
        # Check ToF Sensors
        try:
            sensors = ['front', 'back', 'bottom']
            sensor_results = []
            
            for sensor_id in sensors:
                tof = DummyToFSensor(sensor_id)
                distance = tof.get_distance()
                sensor_results.append({
                    'id': sensor_id,
                    'distance': distance,
                    'status': 0.1 <= distance <= 5.0
                })
                tof.close()
            
            working_sensors = sum(1 for s in sensor_results if s['status'])
            results['tof_sensors'] = {
                'status': working_sensors > 0,
                'sensors': sensor_results,
                'message': f'{working_sensors}/3 dummy ToF sensors OK'
            }
            self.systems['tof_sensors'] = working_sensors > 0
        except Exception as e:
            results['tof_sensors'] = {'status': False, 'message': f'ToF error: {e}'}
        
        # Check Electromagnets
        try:
            magnets = []
            for pin in [18, 19]:  # GPIO pins
                magnet = DummyElectromagnet(pin)
                magnet.activate()
                time.sleep(0.1)
                status = magnet.status()
                magnet.deactivate()
                magnets.append({'pin': pin, 'status': status})
            
            working_magnets = sum(1 for m in magnets if m['status'])
            results['electromagnets'] = {
                'status': working_magnets > 0,
                'magnets': magnets,
                'message': f'{working_magnets}/2 dummy electromagnets OK'
            }
            self.systems['electromagnets'] = working_magnets > 0
        except Exception as e:
            results['electromagnets'] = {'status': False, 'message': f'Electromagnet error: {e}'}
        
        # Check GPS (via MAVLink)
        try:
            mavlink = DummyMAVLink("gps_test")
            gps_msg = mavlink._create_gps_message()
            gps_ok = gps_msg.fix_type >= 3 and gps_msg.satellites_visible >= 6
            
            results['gps'] = {
                'status': gps_ok,
                'fix_type': gps_msg.fix_type,
                'satellites': gps_msg.satellites_visible,
                'message': f'Dummy GPS: {gps_msg.satellites_visible} sats, fix type {gps_msg.fix_type}'
            }
            self.systems['gps'] = gps_ok
            mavlink.close()
        except Exception as e:
            results['gps'] = {'status': False, 'message': f'GPS error: {e}'}
        
        # Check ROS 2 (basic check)
        try:
            import subprocess
            result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
            ros2_ok = result.returncode == 0
            
            results['ros2'] = {
                'status': ros2_ok,
                'version': result.stdout.strip() if ros2_ok else 'Not found',
                'message': 'ROS 2 available' if ros2_ok else 'ROS 2 not found'
            }
            self.systems['ros2'] = ros2_ok
        except Exception as e:
            results['ros2'] = {'status': False, 'message': f'ROS 2 error: {e}'}
        
        return results
    
    def print_summary(self, results: Dict[str, Any]):
        """Print system check summary"""
        print("\n" + "="*50)
        print("🔧 DUMMY SYSTEM CHECK SUMMARY")
        print("="*50)
        
        total_systems = len(results)
        working_systems = sum(1 for r in results.values() if r.get('status', False))
        
        for system, result in results.items():
            status_icon = "✅" if result.get('status', False) else "❌"
            print(f"{status_icon} {system.upper()}: {result.get('message', 'Unknown')}")
        
        print("="*50)
        print(f"📊 OVERALL: {working_systems}/{total_systems} systems working")
        
        if working_systems == total_systems:
            print("🎉 ALL DUMMY SYSTEMS OPERATIONAL!")
        elif working_systems >= total_systems * 0.7:
            print("⚠️  MOSTLY WORKING - Some issues detected")
        else:
            print("🚨 MAJOR ISSUES - Multiple systems failing")
        
        return working_systems / total_systems

def main():
    """Main dummy system test"""
    print("🚁 KAERTEI 2025 FAIO - Dummy Hardware Test")
    print("=========================================")
    print("ℹ️  Running without actual hardware - all systems simulated")
    print()
    
    checker = DummySystemChecker()
    results = checker.check_all_systems()
    success_rate = checker.print_summary(results)
    
    print(f"\n🔧 Success Rate: {success_rate*100:.1f}%")
    
    if success_rate >= 0.8:
        print("✅ DUMMY SYSTEM READY FOR TESTING!")
        return 0
    else:
        print("❌ DUMMY SYSTEM ISSUES DETECTED")
        return 1

if __name__ == "__main__":
    exit(main())
