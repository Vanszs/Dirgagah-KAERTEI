#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np
import time

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
    def __init__(self):
        super().__init__('camera_control_node')
        
        # Load hardware configuration
        self.hw_config = HardwareConfig()
        
        # Check dependencies
        if cv2 is None or CvBridge is None:
            self.get_logger().error("❌ Required dependencies missing")
            return
        
        # Camera states
        self.front_camera_enabled = False
        self.back_camera_enabled = False
        self.top_camera_enabled = False
        
        # OpenCV captures
        self.front_cap = None
        self.back_cap = None
        self.top_cap = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Get camera configuration
        self.camera_width, self.camera_height = self.hw_config.get_camera_resolution()
        self.camera_fps = self.hw_config.get_camera_fps()
        self.alignment_tolerance = self.hw_config.get_alignment_tolerance()
        self.min_object_area = self.hw_config.get_min_object_area()
        
        # Get color detection ranges
        self.red_ranges = self.hw_config.get_color_ranges('red')
        self.blue_ranges = self.hw_config.get_color_ranges('blue')
        
        # Publishers
        self.front_image_pub = self.create_publisher(Image, '/camera/front/image', 10)
        self.back_image_pub = self.create_publisher(Image, '/camera/back/image', 10)
        self.top_image_pub = self.create_publisher(Image, '/camera/top/image', 10)
        self.detection_pub = self.create_publisher(Point, '/vision/detection', 10)
        self.aligned_pub = self.create_publisher(Bool, '/vision/aligned', 10)
        
        # Subscribers
        self.camera_command_sub = self.create_subscription(
            String, '/camera/enable', self.camera_command_callback, 10)
        
        # Timer for camera processing
        self.timer = self.create_timer(0.1, self.process_cameras)
        
        self.get_logger().info("📷 Camera Control Node Started")
    
    def camera_command_callback(self, msg):
        """Handle camera enable/disable commands"""
        command = msg.data.lower()
        
        if command == "enable:front":
            self.enable_front_camera()
        elif command == "disable:front":
            self.disable_front_camera()
        elif command == "enable:back":
            self.enable_back_camera()
        elif command == "disable:back":
            self.disable_back_camera()
        elif command == "enable:top":
            self.enable_top_camera()
        elif command == "disable:top":
            self.disable_top_camera()
        elif command == "disable:all":
            self.disable_all_cameras()
    
    def enable_front_camera(self):
        """Enable front camera"""
        if not self.front_camera_enabled:
            try:
                camera_index = self.hw_config.get_front_camera_index()
                self.front_cap = cv2.VideoCapture(camera_index)
                
                # Set camera properties
                self.front_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                self.front_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                self.front_cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                
                self.front_camera_enabled = True
                self.get_logger().info(f"✅ Front camera enabled (index: {camera_index})")
                
                # Wait for camera startup
                time.sleep(self.hw_config.get_hardware_delays()['camera_startup'])
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable front camera: {e}")
    
    def disable_front_camera(self):
        """Disable front camera"""
        if self.front_camera_enabled and self.front_cap:
            self.front_cap.release()
            self.front_camera_enabled = False
            self.get_logger().info("🔄 Front camera disabled")
    
    def enable_back_camera(self):
        """Enable back camera"""
        if not self.back_camera_enabled:
            try:
                camera_index = self.hw_config.get_back_camera_index()
                self.back_cap = cv2.VideoCapture(camera_index)
                
                self.back_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                self.back_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                self.back_cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                
                self.back_camera_enabled = True
                self.get_logger().info(f"✅ Back camera enabled (index: {camera_index})")
                time.sleep(self.hw_config.get_hardware_delays()['camera_startup'])
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable back camera: {e}")
    
    def disable_back_camera(self):
        """Disable back camera"""
        if self.back_camera_enabled and self.back_cap:
            self.back_cap.release()
            self.back_camera_enabled = False
            self.get_logger().info("🔄 Back camera disabled")
    
    def enable_top_camera(self):
        """Enable top camera"""
        if not self.top_camera_enabled:
            try:
                camera_index = self.hw_config.get_top_camera_index()
                self.top_cap = cv2.VideoCapture(camera_index)
                
                self.top_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                self.top_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                self.top_cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                
                self.top_camera_enabled = True
                self.get_logger().info(f"✅ Top camera enabled (index: {camera_index})")
                time.sleep(self.hw_config.get_hardware_delays()['camera_startup'])
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable top camera: {e}")
    
    def disable_top_camera(self):
        """Disable top camera"""
        if self.top_camera_enabled and self.top_cap:
            self.top_cap.release()
            self.top_camera_enabled = False
            self.get_logger().info("🔄 Top camera disabled")
    
    def disable_all_cameras(self):
        """Disable all cameras"""
        self.disable_front_camera()
        self.disable_back_camera()
        self.disable_top_camera()
        self.get_logger().info("🔄 All cameras disabled")
    
    def process_cameras(self):
        """Process active cameras"""
        # Process front camera
        if self.front_camera_enabled and self.front_cap:
            ret, frame = self.front_cap.read()
            if ret:
                # Publish raw image
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "front_camera"
                self.front_image_pub.publish(img_msg)
                
                # Simple object detection (replace with YOLO if needed)
                detection = self.detect_object(frame)
                if detection:
                    self.publish_detection(detection)
        
        # Process back camera
        if self.back_camera_enabled and self.back_cap:
            ret, frame = self.back_cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "back_camera"
                self.back_image_pub.publish(img_msg)
                
                detection = self.detect_object(frame)
                if detection:
                    self.publish_detection(detection)
        
        # Process top camera
        if self.top_camera_enabled and self.top_cap:
            ret, frame = self.top_cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "top_camera"
                self.top_image_pub.publish(img_msg)
    
    def detect_object(self, frame):
        """Simple object detection (replace with YOLO for better results)"""
        try:
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define range for detecting objects (example: red objects)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            
            # Create mask
            mask = cv2.inRange(hsv, lower_red, upper_red)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Get largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Only return detection if object is large enough
                if cv2.contourArea(largest_contour) > 1000:
                    return (center_x, center_y)
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return None
    
    def publish_detection(self, detection):
        """Publish object detection"""
        if detection:
            point = Point()
            point.x = float(detection[0])
            point.y = float(detection[1])
            point.z = 0.0
            self.detection_pub.publish(point)
            
            # Check if object is aligned (center of frame)
            frame_center_x = 320  # Assuming 640px width
            frame_center_y = 240  # Assuming 480px height
            
            error_x = abs(detection[0] - frame_center_x)
            error_y = abs(detection[1] - frame_center_y)
            
            aligned = Bool()
            aligned.data = (error_x < 30 and error_y < 30)  # Alignment threshold
            self.aligned_pub.publish(aligned)

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
