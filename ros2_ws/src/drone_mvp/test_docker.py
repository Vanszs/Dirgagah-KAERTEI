#!/usr/bin/env python3
"""
Simple test script to verify KAERTEI 2025 system functionality in Docker
"""

import sys
import os

# Add current directory to Python path
sys.path.insert(0, '/workspace/ros2_ws/src/drone_mvp')

try:
    print("🔄 Testing KAERTEI 2025 FAIO System...")
    print("=====================================")
    
    # Test 1: Basic imports
    print("1. Testing basic imports...")
    import rclpy
    import cv2
    import numpy as np
    import pymavlink
    print("   ✅ All basic dependencies imported successfully")
    
    # Test 2: Hardware configuration
    print("2. Testing hardware configuration...")
    from drone_mvp.hardware_config import HardwareConfig
    hw_config = HardwareConfig()
    print(f"   ✅ Hardware config loaded (debug_mode: {hw_config.get_mission_debug_mode()})")
    
    # Test 3: Checkpoint system
    print("3. Testing checkpoint mission system...")
    from drone_mvp.checkpoint_mission_node import MissionCheckpoint, CheckpointMissionNode
    checkpoints = list(MissionCheckpoint)
    print(f"   ✅ Found {len(checkpoints)} mission checkpoints")
    print(f"   📝 First checkpoint: {checkpoints[0].value}")
    print(f"   📝 Last checkpoint: {checkpoints[-1].value}")
    
    # Test 4: ROS 2 initialization
    print("4. Testing ROS 2 initialization...")
    rclpy.init()
    print("   ✅ ROS 2 initialized successfully")
    rclpy.shutdown()
    print("   ✅ ROS 2 shutdown successfully")
    
    print("")
    print("🎉 All tests passed! KAERTEI 2025 system is ready!")
    print("🐳 Docker environment is working correctly.")
    print("")
    print("Available commands:")
    print("  - ./run_checkpoint_mission.sh debug   # Run mission in debug mode")
    print("  - ./run_checkpoint_mission.sh auto    # Run autonomous mission")
    print("  - python3 /workspace/ros2_ws/src/drone_mvp/drone_mvp/checkpoint_mission_node.py")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
