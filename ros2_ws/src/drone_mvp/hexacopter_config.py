# KAERTEI 2025 FAIO - Hexacopter Hardware Configuration
# Flexible configuration for custom hexacopter setup

hardware_config = {
    # Hexacopter Configuration
    "flight_controller": {
        "type": "hexacopter",
        "motors": 6,
        "frame_type": "X6",
        "auto_detect_port": True,
        "possible_ports": [
            "/dev/ttyACM*",
            "/dev/ttyUSB*", 
            "/dev/serial/by-id/*"
        ],
        "baud_rates": [57600, 115200, 921600]
    },
    
    # Camera Configuration (3 cameras)
    "cameras": {
        "count": 3,
        "auto_detect": True,
        "positions": {
            "front": {"id": "auto", "resolution": [640, 480]},
            "back": {"id": "auto", "resolution": [640, 480]},
            "bottom": {"id": "auto", "resolution": [640, 480]}
        },
        "possible_devices": [
            "/dev/video*",
            "/dev/v4l/by-id/*",
            "/dev/v4l/by-path/*"
        ]
    },
    
    # Relay & Electromagnet Configuration
    "electromagnets": {
        "count": 2,
        "auto_detect_gpio": True,
        "possible_pins": [18, 19, 20, 21, 22, 23, 24, 25],
        "voltage": "12V",
        "relay_type": "active_low"
    },
    
    # Sensor Configuration
    "sensors": {
        "tof_sensors": {
            "count": 3,
            "positions": ["front", "back", "bottom"],
            "auto_detect_i2c": True,
            "i2c_addresses": ["auto"]
        },
        "imu": {
            "integrated": True,  # In flight controller
            "external": False
        },
        "gps": {
            "integrated": True,  # In flight controller
            "external": False
        }
    },
    
    # Power Management
    "power": {
        "battery_cells": "6S",
        "voltage_range": [20.0, 25.2],
        "current_sensor": True,
        "power_module": True
    },
    
    # Safety Configuration
    "safety": {
        "geofence": True,
        "failsafe_modes": ["RTL", "LAND", "STABILIZE"],
        "emergency_stop": True,
        "low_battery_action": "RTL"
    }
}

# Auto-detection functions
def detect_flight_controller():
    """Auto-detect flight controller port"""
    import glob
    import serial
    
    possible_ports = []
    for pattern in hardware_config["flight_controller"]["possible_ports"]:
        possible_ports.extend(glob.glob(pattern))
    
    for port in possible_ports:
        for baud in hardware_config["flight_controller"]["baud_rates"]:
            try:
                with serial.Serial(port, baud, timeout=1) as ser:
                    # Try to read some data
                    ser.write(b'\r\n')
                    response = ser.read(100)
                    if response:
                        return {"port": port, "baud": baud, "status": "detected"}
            except:
                continue
    
    return {"port": None, "baud": None, "status": "not_found"}

def detect_cameras():
    """Auto-detect available cameras"""
    import glob
    
    detected_cameras = []
    possible_devices = []
    
    for pattern in hardware_config["cameras"]["possible_devices"]:
        possible_devices.extend(glob.glob(pattern))
    
    for i, device in enumerate(possible_devices[:3]):  # Max 3 cameras
        try:
            # Try to open camera
            import cv2
            cap = cv2.VideoCapture(device)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    positions = ["front", "back", "bottom"]
                    detected_cameras.append({
                        "position": positions[i] if i < len(positions) else f"camera_{i}",
                        "device": device,
                        "resolution": frame.shape[:2][::-1] if ret else [640, 480],
                        "status": "working"
                    })
                cap.release()
        except:
            continue
    
    return detected_cameras

def detect_gpio_pins():
    """Auto-detect available GPIO pins"""
    import os
    
    available_pins = []
    for pin in hardware_config["electromagnets"]["possible_pins"]:
        gpio_path = f"/sys/class/gpio/gpio{pin}"
        if os.path.exists(gpio_path) or os.path.exists("/sys/class/gpio/export"):
            available_pins.append(pin)
    
    return available_pins[:hardware_config["electromagnets"]["count"]]

def get_hardware_summary():
    """Get complete hardware detection summary"""
    import datetime
    
    summary = {
        "timestamp": datetime.datetime.now().isoformat(),
        "flight_controller": detect_flight_controller(),
        "cameras": detect_cameras(),
        "gpio_pins": detect_gpio_pins(),
        "config": hardware_config
    }
    
    return summary

def main():
    """Main function to run hardware detection"""
    print("🚁 KAERTEI 2025 FAIO - Hexacopter Hardware Detection")
    print("=" * 50)
    
    summary = get_hardware_summary()
    
    print(f"🕒 Detection Time: {summary['timestamp']}")
    print()
    
    # Flight Controller
    fc = summary['flight_controller']
    print("🎛️  FLIGHT CONTROLLER:")
    if fc['status'] == 'detected':
        print(f"   ✅ Port: {fc['port']}")
        print(f"   ✅ Baud Rate: {fc['baud']}")
        print("   ✅ Status: Connected")
    else:
        print("   ❌ Status: Not detected")
        print("   💡 Check USB connection and permissions")
    print()
    
    # Cameras
    cameras = summary['cameras']
    print("📷 CAMERAS:")
    if cameras:
        for i, cam in enumerate(cameras):
            print(f"   ✅ Camera {i+1} ({cam['position']}): {cam['device']}")
            print(f"      Resolution: {cam['resolution']}")
    else:
        print("   ❌ No cameras detected")
        print("   💡 Check USB camera connections")
    print()
    
    # GPIO Pins
    gpio_pins = summary['gpio_pins']
    print("🔌 GPIO PINS (Electromagnets):")
    if gpio_pins:
        for pin in gpio_pins:
            print(f"   ✅ GPIO Pin {pin}: Available")
    else:
        print("   ❌ No GPIO pins available")
        print("   💡 Check if running on compatible hardware")
    print()
    
    # Configuration Summary
    config = summary['config']
    print("⚙️  CONFIGURATION:")
    print(f"   Aircraft Type: {config['flight_controller']['type']}")
    print(f"   Motors: {config['flight_controller']['motors']}")
    print(f"   Camera Count: {config['cameras']['count']}")
    print(f"   Electromagnet Count: {config['electromagnets']['count']}")
    print(f"   Battery: {config['power']['battery_cells']}")
    print()
    
    # Overall Status
    fc_ok = fc['status'] == 'detected'
    cam_ok = len(cameras) > 0
    gpio_ok = len(gpio_pins) > 0
    
    total_systems = 3
    working_systems = sum([fc_ok, cam_ok, gpio_ok])
    
    print("📊 OVERALL STATUS:")
    print(f"   Working Systems: {working_systems}/{total_systems}")
    
    if working_systems == total_systems:
        print("   🎉 ALL SYSTEMS READY!")
        return 0
    elif working_systems >= 2:
        print("   ⚠️  MOSTLY READY - Some hardware missing")
        return 1
    else:
        print("   🚨 HARDWARE ISSUES - Check connections")
        return 2

if __name__ == "__main__":
    exit(main())
