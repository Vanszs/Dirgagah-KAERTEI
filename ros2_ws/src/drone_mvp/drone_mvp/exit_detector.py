#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ExitDetectorNode(Node):
    def __init__(self):
        super().__init__('exit_detector')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.6),
                ('min_contour_area', 5000),
                ('detection_timeout', 3.0),
                ('exit_aspect_ratio_min', 0.5),
                ('exit_aspect_ratio_max', 2.0)
            ]
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.exit_detected_pub = self.create_publisher(Bool, '/vision/exit_detected', qos_profile)
        self.exit_point_pub = self.create_publisher(Point, '/vision/exit_point', qos_profile)
        self.debug_image_pub = self.create_publisher(Image, '/vision/exit_debug_image', qos_profile)
        self.status_pub = self.create_publisher(String, '/vision/exit_status', qos_profile)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/vision/top/image', self.image_callback, qos_profile)
        self.camera_enable_sub = self.create_subscription(
            String, '/vision/camera_enable', self.camera_enable_callback, qos_profile)
        
        # Detection state
        self.detection_enabled = False
        self.last_detection_time = None
        self.exit_detected = False
        self.exit_center_point = None
        
        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Exit Detector Node initialized")
    
    def camera_enable_callback(self, msg):
        """Handle camera enable/disable commands"""
        if msg.data == "top" or msg.data == "all":
            self.detection_enabled = True
            self.get_logger().info("Exit detection enabled")
        else:
            self.detection_enabled = False
            self.exit_detected = False
            self.get_logger().info("Exit detection disabled")
    
    def image_callback(self, msg):
        """Process images from top camera for exit detection"""
        if not self.detection_enabled:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect exit gate
            exit_found, exit_point, debug_image = self.detect_exit_gate(cv_image)
            
            # Update detection state
            if exit_found:
                self.exit_detected = True
                self.exit_center_point = exit_point
                self.last_detection_time = self.get_clock().now()
                
                # Publish detection result
                detection_msg = Bool()
                detection_msg.data = True
                self.exit_detected_pub.publish(detection_msg)
                
                # Publish exit point
                if exit_point:
                    self.exit_point_pub.publish(exit_point)
                
                self.get_logger().info(f"Exit gate detected at ({exit_point.x:.1f}, {exit_point.y:.1f})")
            else:
                # Reset detection if timeout
                current_time = self.get_clock().now()
                timeout_duration = rclpy.duration.Duration(
                    seconds=self.get_parameter('detection_timeout').value)
                
                if (self.last_detection_time is None or 
                    (current_time - self.last_detection_time) > timeout_duration):
                    if self.exit_detected:
                        self.exit_detected = False
                        detection_msg = Bool()
                        detection_msg.data = False
                        self.exit_detected_pub.publish(detection_msg)
            
            # Publish debug image
            if debug_image is not None:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                    self.debug_image_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish debug image: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def detect_exit_gate(self, image):
        """Detect exit gate in the image"""
        try:
            # Create debug image
            debug_image = image.copy()
            height, width = image.shape[:2]
            
            # Convert to different color spaces for robust detection
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Method 1: Edge detection for rectangular openings
            exit_found, exit_point = self.detect_by_edges(gray, debug_image)
            
            if not exit_found:
                # Method 2: Color-based detection (looking for specific exit colors)
                exit_found, exit_point = self.detect_by_color(hsv, debug_image)
            
            if not exit_found:
                # Method 3: Brightness-based detection (exit might be brighter)
                exit_found, exit_point = self.detect_by_brightness(gray, debug_image)
            
            return exit_found, exit_point, debug_image
            
        except Exception as e:
            self.get_logger().error(f"Error in exit detection: {e}")
            return False, None, image
    
    def detect_by_edges(self, gray_image, debug_image):
        """Detect exit by edge detection"""
        try:
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
            
            # Edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter contours for rectangular exit-like shapes
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.get_parameter('min_contour_area').value:
                    continue
                
                # Approximate contour to polygon
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # Check if it's roughly rectangular (4 vertices)
                if len(approx) >= 4:
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h
                    
                    # Check aspect ratio (exit gates are usually rectangular)
                    min_ratio = self.get_parameter('exit_aspect_ratio_min').value
                    max_ratio = self.get_parameter('exit_aspect_ratio_max').value
                    
                    if min_ratio <= aspect_ratio <= max_ratio:
                        # Calculate center point
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Draw detection on debug image
                        cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(debug_image, (center_x, center_y), 5, (0, 255, 0), -1)
                        cv2.putText(debug_image, "EXIT", (x, y - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # Create point message
                        exit_point = Point()
                        exit_point.x = float(center_x)
                        exit_point.y = float(center_y)
                        exit_point.z = float(area)  # Use area as confidence
                        
                        return True, exit_point
            
            return False, None
            
        except Exception as e:
            self.get_logger().error(f"Error in edge detection: {e}")
            return False, None
    
    def detect_by_color(self, hsv_image, debug_image):
        """Detect exit by color characteristics"""
        try:
            # Define color ranges for typical exit gate colors
            # Green exit signs
            green_lower = np.array([40, 50, 50])
            green_upper = np.array([80, 255, 255])
            green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
            
            # Red exit signs
            red_lower1 = np.array([0, 50, 50])
            red_upper1 = np.array([10, 255, 255])
            red_lower2 = np.array([170, 50, 50])
            red_upper2 = np.array([180, 255, 255])
            red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
            red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            
            # Combine masks
            combined_mask = cv2.bitwise_or(green_mask, red_mask)
            
            # Morphological operations to clean up
            kernel = np.ones((5, 5), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Find largest contour
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > self.get_parameter('min_contour_area').value:
                    # Get center
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                        
                        # Draw detection
                        cv2.drawContours(debug_image, [largest_contour], -1, (255, 0, 0), 2)
                        cv2.circle(debug_image, (center_x, center_y), 5, (255, 0, 0), -1)
                        cv2.putText(debug_image, "EXIT (Color)", (center_x - 50, center_y - 20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        
                        # Create point message
                        exit_point = Point()
                        exit_point.x = float(center_x)
                        exit_point.y = float(center_y)
                        exit_point.z = float(area)
                        
                        return True, exit_point
            
            return False, None
            
        except Exception as e:
            self.get_logger().error(f"Error in color detection: {e}")
            return False, None
    
    def detect_by_brightness(self, gray_image, debug_image):
        """Detect exit by brightness (exit might be brighter than surroundings)"""
        try:
            # Apply threshold to find bright areas
            _, thresh = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)
            
            # Morphological operations
            kernel = np.ones((10, 10), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Look for appropriately sized bright areas
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.get_parameter('min_contour_area').value:
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h
                    
                    # Check aspect ratio
                    min_ratio = self.get_parameter('exit_aspect_ratio_min').value
                    max_ratio = self.get_parameter('exit_aspect_ratio_max').value
                    
                    if min_ratio <= aspect_ratio <= max_ratio:
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Draw detection
                        cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        cv2.circle(debug_image, (center_x, center_y), 5, (0, 255, 255), -1)
                        cv2.putText(debug_image, "EXIT (Bright)", (x, y - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        # Create point message
                        exit_point = Point()
                        exit_point.x = float(center_x)
                        exit_point.y = float(center_y)
                        exit_point.z = float(area)
                        
                        return True, exit_point
            
            return False, None
            
        except Exception as e:
            self.get_logger().error(f"Error in brightness detection: {e}")
            return False, None
    
    def publish_status(self):
        """Publish current detection status"""
        status_msg = String()
        if not self.detection_enabled:
            status_msg.data = "DISABLED"
        elif self.exit_detected:
            status_msg.data = "EXIT_DETECTED"
        else:
            status_msg.data = "SEARCHING"
        
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        exit_detector = ExitDetectorNode()
        rclpy.spin(exit_detector)
    except KeyboardInterrupt:
        pass
    finally:
        if 'exit_detector' in locals():
            exit_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
