#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import argparse
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class VisionDetectorNode(Node):
    def __init__(self, camera_type="front"):
        super().__init__(f'vision_detector_{camera_type}')
        
        # Camera configuration
        self.camera_type = camera_type
        self.camera_index = self.get_camera_index(camera_type)
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.7),
                ('image_width', 640),
                ('image_height', 480),
                ('detection_timeout', 5.0)
            ]
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('image_width').value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('image_height').value)
        
        # Initialize YOLO model
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO('yolov8n.pt')  # Using nano model for speed
                self.get_logger().info("YOLOv8 model loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.model = None
        else:
            self.get_logger().warn("YOLOv8 not available - using mock detection")
            self.model = None
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(String, '/vision/detection', qos_profile)
        self.image_pub = self.create_publisher(Image, f'/vision/{camera_type}/image', qos_profile)
        self.target_point_pub = self.create_publisher(Point, f'/vision/{camera_type}/target_point', qos_profile)
        
        # Subscribers
        self.camera_enable_sub = self.create_subscription(
            String, '/vision/camera_enable', self.camera_enable_callback, qos_profile)
        
        # State variables
        self.camera_enabled = True  # Start enabled
        self.current_target = None
        self.last_detection_time = None
        
        # Detection classes mapping
        self.detection_classes = {
            'item': ['bottle', 'cup', 'bowl', 'book', 'cell phone'],
            'dropzone': ['bowl', 'sink', 'oven', 'microwave'],
            'exit': ['door', 'window'],
            'person': ['person']
        }
        
        # Timer for image processing
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 FPS
        
        self.get_logger().info(f"Vision Detector Node initialized for {camera_type} camera")
    
    def get_camera_index(self, camera_type):
        """Get camera index based on type"""
        camera_indices = {
            'front': 0,
            'back': 1,
            'top': 2
        }
        return camera_indices.get(camera_type, 0)
    
    def camera_enable_callback(self, msg):
        """Handle camera enable/disable commands"""
        if msg.data == self.camera_type or msg.data == "all":
            self.camera_enabled = True
            self.get_logger().info(f"{self.camera_type} camera enabled")
        elif msg.data == "disable_all" or (msg.data != self.camera_type and msg.data in ['front', 'back', 'top']):
            self.camera_enabled = False
            self.get_logger().info(f"{self.camera_type} camera disabled")
    
    def process_frame(self):
        """Main image processing loop"""
        if not self.camera_enabled:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn(f"Failed to read from {self.camera_type} camera")
            return
        
        # Publish raw image
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")
        
        # Perform detection
        detection_result = self.detect_objects(frame)
        
        # Publish detection result
        detection_msg = String()
        detection_msg.data = detection_result
        self.detection_pub.publish(detection_msg)
        
        # If target detected, publish target point
        if detection_result != "NONE" and self.current_target is not None:
            self.target_point_pub.publish(self.current_target)
    
    def detect_objects(self, frame):
        """Detect objects using YOLO or mock detection"""
        if self.model is None:
            return self.mock_detection(frame)
        
        try:
            # Run YOLO inference
            results = self.model(frame, conf=self.get_parameter('confidence_threshold').value)
            
            # Process results
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    # Get the most confident detection
                    max_conf_idx = boxes.conf.argmax()
                    box = boxes.xyxy[max_conf_idx]
                    conf = boxes.conf[max_conf_idx]
                    class_id = int(boxes.cls[max_conf_idx])
                    class_name = self.model.names[class_id]
                    
                    # Calculate center point
                    x1, y1, x2, y2 = box
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Update target point
                    self.current_target = Point()
                    self.current_target.x = float(center_x)
                    self.current_target.y = float(center_y)
                    self.current_target.z = float(conf)
                    
                    # Classify detection type
                    detection_type = self.classify_detection(class_name)
                    
                    if detection_type != "NONE":
                        self.get_logger().info(f"Detected {detection_type}: {class_name} (conf: {conf:.2f})")
                        return detection_type
            
            return "NONE"
            
        except Exception as e:
            self.get_logger().error(f"YOLO detection error: {e}")
            return "NONE"
    
    def classify_detection(self, class_name):
        """Classify detected object into mission-relevant categories"""
        for category, classes in self.detection_classes.items():
            if class_name.lower() in [c.lower() for c in classes]:
                if category == 'item':
                    return "ITEM_DETECTED"
                elif category == 'dropzone':
                    return "DROPZONE_DETECTED" 
                elif category == 'exit':
                    return "EXIT_DETECTED"
                elif category == 'person':
                    return "PERSON_DETECTED"
        
        return "NONE"
    
    def mock_detection(self, frame):
        """Mock detection for testing without YOLO"""
        # Simple color-based detection for testing
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for different objects
        # Red objects (items)
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        
        # Blue objects (dropzones)
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        
        # Green objects (exit)
        green_lower = np.array([40, 100, 100])
        green_upper = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        
        # Check for detections
        if cv2.countNonZero(red_mask) > 1000:
            # Find contours and get center
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    self.current_target = Point()
                    self.current_target.x = float(cx)
                    self.current_target.y = float(cy)
                    self.current_target.z = 0.8  # Mock confidence
                    
                    return "ITEM_DETECTED"
        
        elif cv2.countNonZero(blue_mask) > 1000:
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    self.current_target = Point()
                    self.current_target.x = float(cx)
                    self.current_target.y = float(cy)
                    self.current_target.z = 0.8
                    
                    return "DROPZONE_DETECTED"
        
        elif cv2.countNonZero(green_mask) > 1000:
            contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    self.current_target = Point()
                    self.current_target.x = float(cx)
                    self.current_target.y = float(cy)
                    self.current_target.z = 0.8
                    
                    return "EXIT_DETECTED"
        
        return "NONE"
    
    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Vision Detector Node')
    parser.add_argument('--camera', type=str, default='front', 
                       choices=['front', 'back', 'top'],
                       help='Camera type to use (front, back, top)')
    
    # Parse known args to separate ROS args from our args
    known_args, unknown_args = parser.parse_known_args()
    
    rclpy.init(args=unknown_args)
    
    try:
        vision_node = VisionDetectorNode(camera_type=known_args.camera)
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_node' in locals():
            vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
