#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - YOLO Object Detection Integration
Critical component for Checkpoint 16-20 object detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from pathlib import Path

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️  YOLO not available - using fallback color detection")

class YOLOObjectDetector(Node):
    """
    YOLO-based object detection for competitive object pickup
    """
    
    def __init__(self):
        super().__init__('yolo_object_detector')
        
        # Initialize parameters
        self.confidence_threshold = 0.6
        self.bridge = CvBridge()
        self.current_frame = None
        self.detection_active = False
        
        # Publishers
        self.detection_pub = self.create_publisher(Point, '/vision/detection', 10)
        self.confidence_pub = self.create_publisher(Float32, '/vision/confidence', 10)
        self.status_pub = self.create_publisher(String, '/vision/status', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/front/image_raw', self.image_callback, 10)
        self.control_sub = self.create_subscription(
            String, '/vision/control', self.control_callback, 10)
        
        # Initialize YOLO model
        self.setup_yolo_model()
        
        # Timer for detection processing
        self.detection_timer = self.create_timer(0.1, self.process_detection)
        
        self.get_logger().info("✅ YOLO Object Detector initialized")
    
    def setup_yolo_model(self):
        """Setup YOLO model for object detection"""
        model_path = Path(__file__).parent.parent / "models" / "best.pt"
        
        if YOLO_AVAILABLE and model_path.exists():
            try:
                self.yolo_model = YOLO(str(model_path))
                self.use_yolo = True
                self.get_logger().info(f"✅ YOLO model loaded: {model_path}")
            except Exception as e:
                self.get_logger().warn(f"⚠️  YOLO model failed to load: {e}")
                self.use_yolo = False
        else:
            self.use_yolo = False
            self.get_logger().warn("⚠️  Using fallback color detection")
    
    def control_callback(self, msg):
        """Handle detection control commands"""
        command = msg.data.lower()
        
        if command == "start":
            self.detection_active = True
            self.publish_status("DETECTION_ACTIVE")
            self.get_logger().info("🎯 Object detection STARTED")
            
        elif command == "stop":
            self.detection_active = False
            self.publish_status("DETECTION_STOPPED")
            self.get_logger().info("⏹️  Object detection STOPPED")
            
        elif command.startswith("confidence:"):
            try:
                self.confidence_threshold = float(command.split(':')[1])
                self.get_logger().info(f"🎯 Confidence threshold: {self.confidence_threshold}")
            except ValueError:
                self.get_logger().error(f"❌ Invalid confidence value: {command}")
    
    def image_callback(self, msg):
        """Store latest camera frame"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Image conversion failed: {e}")
    
    def process_detection(self):
        """Main detection processing loop"""
        if not self.detection_active or self.current_frame is None:
            return
        
        try:
            if self.use_yolo:
                detection_result = self.yolo_detection()
            else:
                detection_result = self.color_detection_fallback()
            
            if detection_result:
                center_x, center_y, confidence = detection_result
                
                # Publish detection result
                point = Point()
                point.x = float(center_x)
                point.y = float(center_y)
                point.z = 0.0
                self.detection_pub.publish(point)
                
                # Publish confidence
                conf_msg = Float32()
                conf_msg.data = confidence
                self.confidence_pub.publish(conf_msg)
                
                self.publish_status(f"OBJECT_DETECTED_CONF_{confidence:.2f}")
                self.get_logger().info(f"🎯 Object detected at ({center_x}, {center_y}) conf={confidence:.2f}")
            else:
                self.publish_status("SEARCHING")
                
        except Exception as e:
            self.get_logger().error(f"❌ Detection processing error: {e}")
            self.publish_status("DETECTION_ERROR")
    
    def yolo_detection(self):
        """YOLO-based object detection"""
        results = self.yolo_model(self.current_frame, verbose=False)
        
        best_detection = None
        highest_confidence = 0.0
        
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    confidence = float(box.conf[0])
                    
                    if confidence > self.confidence_threshold and confidence > highest_confidence:
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        best_detection = (center_x, center_y, confidence)
                        highest_confidence = confidence
        
        return best_detection
    
    def color_detection_fallback(self):
        """Fallback color-based detection (red objects)"""
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        
        # Red color range (HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 1000:  # Minimum area threshold
                # Get center point
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    confidence = min(area / 10000.0, 0.95)  # Fake confidence based on area
                    
                    return (center_x, center_y, confidence)
        
        return None
    
    def publish_status(self, status):
        """Publish current detection status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = YOLOObjectDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
