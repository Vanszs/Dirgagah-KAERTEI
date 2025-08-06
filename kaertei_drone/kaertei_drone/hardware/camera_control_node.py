#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Camera Control Node
Handles 3x USB cameras for 12-checkpoint mission system
Integrated with YOLOv8 vision system for object detection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np
import time
import threading

# Import OpenCV with error handling
try:
    import cv2
except ImportError:
    print("❌ OpenCV not installed. Run: pip3 install opencv-python")
    cv2 = None

# Import CV Bridge with error handling
try:
    from cv_bridge import CvBridge
except ImportError:
    print("❌ cv_bridge not installed. Run: sudo apt install ros-humble-cv-bridge")
    CvBridge = None

# Import hardware configuration
from .hardware_config import HardwareConfig

class CameraControlNode(Node):
    """
    Camera Control Node for 12-checkpoint mission system
    Handles multi-camera setup for YOLOv8 vision integration
    
    Features:
    - 3x USB camera management (front_bottom, back, front_nav)
    - Hardware-optimized for Raspberry Pi 5
    - Integration with 12-checkpoint mission flow
    - Object detection pipeline for vision system
    """
    
    def __init__(self):
        super().__init__('camera_control_node')
        
        # Load hardware configuration for 12-checkpoint system
        self.hw_config = HardwareConfig()
        
        # Check dependencies
        if cv2 is None or CvBridge is None:
            self.get_logger().error("❌ Required dependencies missing")
            return
        
        # Camera states for 12-checkpoint system
        self.camera_states = {
            'front_bottom': {'enabled': False, 'cap': None, 'active': False},  # Primary pickup camera
            'back': {'enabled': False, 'cap': None, 'active': False},          # Landing/dropzone camera  
            'front_nav': {'enabled': False, 'cap': None, 'active': False}      # Navigation/gate camera
        }
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera configuration for mission requirements
        self.camera_config = {
            'width': 640,
            'height': 480, 
            'fps': 30,
            'fourcc': cv2.VideoWriter_fourcc(*'MJPG'),
            'buffer_size': 1  # Low latency for real-time mission
        }
        
        # Get hardware configuration
        self.front_bottom_index = self.hw_config.get_front_bottom_camera_index()
        self.back_camera_index = self.hw_config.get_back_camera_index()
        self.front_nav_index = self.hw_config.get_front_nav_camera_index()
        
        # Publishers aligned with 12-checkpoint system
        self.front_bottom_pub = self.create_publisher(Image, '/camera/front_bottom/image_raw', 10)
        self.back_camera_pub = self.create_publisher(Image, '/camera/back/image_raw', 10) 
        self.front_nav_pub = self.create_publisher(Image, '/camera/front_nav/image_raw', 10)
        self.camera_status_pub = self.create_publisher(String, '/hardware/camera/status', 10)
        
        # Subscribers for checkpoint-specific camera control
        self.checkpoint_sub = self.create_subscription(
            String, '/mission/checkpoint_status', self.checkpoint_callback, 10)
        self.camera_enable_sub = self.create_subscription(
            String, '/hardware/camera/enable', self.camera_enable_callback, 10)
        
        # Processing timer
        self.timer = self.create_timer(0.033, self.process_cameras)  # ~30 FPS
        
        self.get_logger().info("📷 12-Checkpoint Camera Control Node initialized")
        self.get_logger().info(f"   Front Bottom: /dev/video{self.front_bottom_index}")
        self.get_logger().info(f"   Back Camera: /dev/video{self.back_camera_index}")
        self.get_logger().info(f"   Front Nav: /dev/video{self.front_nav_index}")
    
    def checkpoint_callback(self, msg):
        """Handle checkpoint-specific camera activation"""
        checkpoint = msg.data
        
        # CP1-CP3: Navigation cameras for takeoff and exit gate
        if checkpoint in ["CP1_INIT_ARM", "CP2_TAKEOFF_ASCEND", "CP3_EXIT_GATE_PASS"]:
            self.enable_camera('front_nav')
            self.disable_camera('front_bottom')
            self.disable_camera('back')
            
        # CP4-CP7: Item detection and pickup cameras  
        elif checkpoint in ["CP4_NAVIGATE_WP1", "CP5_SEARCH_ITEM_WP1", "CP6_ALIGN_PICKUP_WP1", "CP7_PICKUP_ITEM"]:
            self.enable_camera('front_bottom')
            self.enable_camera('front_nav') 
            self.disable_camera('back')
            
        # CP8-CP10: Navigation and dropzone cameras
        elif checkpoint in ["CP8_NAVIGATE_WP2", "CP9_ALIGN_DROP_WP2", "CP10_DROP_ITEM"]:
            self.enable_camera('back')
            self.enable_camera('front_nav')
            self.disable_camera('front_bottom')
            
        # CP11-CP12: Landing cameras
        elif checkpoint in ["CP11_NAVIGATE_WP5_LAND", "CP12_FINAL_DESCENT_DISARM"]:
            self.enable_camera('back')
            self.enable_camera('front_nav')
            self.disable_camera('front_bottom')
            
        self.get_logger().info(f"📷 Camera configuration updated for {checkpoint}")
        
    def camera_enable_callback(self, msg):
        """Handle manual camera enable/disable commands"""
        command = msg.data.lower().strip()
        
        if command == "front_bottom:on":
            self.enable_camera('front_bottom')
        elif command == "front_bottom:off":
            self.disable_camera('front_bottom')
        elif command == "back:on":
            self.enable_camera('back')
        elif command == "back:off":
            self.disable_camera('back')
        elif command == "front_nav:on":
            self.enable_camera('front_nav')
        elif command == "front_nav:off":
            self.disable_camera('front_nav')
        elif command == "all:on":
            self.enable_all_cameras()
        elif command == "all:off":
            self.disable_all_cameras()
            
    def enable_camera(self, camera_name):
        """Enable specific camera"""
        if camera_name not in self.camera_states:
            return
            
        if not self.camera_states[camera_name]['enabled']:
            try:
                if camera_name == 'front_bottom':
                    camera_index = self.front_bottom_index
                elif camera_name == 'back':
                    camera_index = self.back_camera_index 
                elif camera_name == 'front_nav':
                    camera_index = self.front_nav_index
                else:
                    return
                    
                cap = cv2.VideoCapture(camera_index)
                
                # Configure camera
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config['width'])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config['height'])
                cap.set(cv2.CAP_PROP_FPS, self.camera_config['fps'])
                cap.set(cv2.CAP_PROP_FOURCC, self.camera_config['fourcc'])
                cap.set(cv2.CAP_PROP_BUFFERSIZE, self.camera_config['buffer_size'])
                
                self.camera_states[camera_name]['cap'] = cap
                self.camera_states[camera_name]['enabled'] = True
                
                self.get_logger().info(f"✅ {camera_name} camera enabled (index: {camera_index})")
                
                # Warmup delay
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable {camera_name}: {e}")
                
    def disable_camera(self, camera_name):
        """Disable specific camera"""
        if camera_name not in self.camera_states:
            return
            
        if self.camera_states[camera_name]['enabled']:
            cap = self.camera_states[camera_name]['cap']
            if cap:
                cap.release()
                
            self.camera_states[camera_name]['cap'] = None
            self.camera_states[camera_name]['enabled'] = False
            
            self.get_logger().info(f"🔄 {camera_name} camera disabled")
            
    def enable_all_cameras(self):
        """Enable all cameras"""
        for camera_name in self.camera_states.keys():
            self.enable_camera(camera_name)
            
    def disable_all_cameras(self):
        """Disable all cameras"""
        for camera_name in self.camera_states.keys():
            self.disable_camera(camera_name)
            
    def process_cameras(self):
        """Process active cameras and publish frames"""
        active_cameras = 0
        
        # Process front_bottom camera (pickup operations)
        if self.camera_states['front_bottom']['enabled']:
            cap = self.camera_states['front_bottom']['cap']
            if cap and cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    # Publish image for YOLOv8 processing
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "front_bottom_camera"
                    self.front_bottom_pub.publish(img_msg)
                    active_cameras += 1
                    
        # Process back camera (landing/dropzone operations)  
        if self.camera_states['back']['enabled']:
            cap = self.camera_states['back']['cap']
            if cap and cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "back_camera"
                    self.back_camera_pub.publish(img_msg)
                    active_cameras += 1
                    
        # Process front_nav camera (navigation/gate operations)
        if self.camera_states['front_nav']['enabled']:
            cap = self.camera_states['front_nav']['cap']
            if cap and cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg() 
                    img_msg.header.frame_id = "front_nav_camera"
                    self.front_nav_pub.publish(img_msg)
                    active_cameras += 1
                    
        # Publish camera status for monitoring
        status_msg = String()
        status_msg.data = f"active_cameras:{active_cameras}"
        self.camera_status_pub.publish(status_msg)
        
    def cleanup(self):
        """Cleanup cameras on node shutdown"""
        self.disable_all_cameras()
        self.get_logger().info("📷 Camera cleanup completed")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
