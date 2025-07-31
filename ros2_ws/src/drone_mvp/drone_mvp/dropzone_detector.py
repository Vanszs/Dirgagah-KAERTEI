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


class DropzoneDetectorNode(Node):
    def __init__(self):
        super().__init__('dropzone_detector')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.6),
                ('min_contour_area', 3000),
                ('detection_timeout', 3.0),
                ('basket_aspect_ratio_min', 0.7),
                ('basket_aspect_ratio_max', 1.3),
                ('min_baskets_for_dropzone', 2)
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
        self.dropzone_detected_pub = self.create_publisher(String, '/vision/detection', qos_profile)
        self.dropzone_point_pub = self.create_publisher(Point, '/vision/dropzone_point', qos_profile)
        self.debug_image_pub = self.create_publisher(Image, '/vision/dropzone_debug_image', qos_profile)
        self.status_pub = self.create_publisher(String, '/vision/dropzone_status', qos_profile)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/vision/front/image', self.image_callback, qos_profile)
        self.camera_enable_sub = self.create_subscription(
            String, '/vision/camera_enable', self.camera_enable_callback, qos_profile)
        
        # Detection state
        self.detection_enabled = False
        self.last_detection_time = None
        self.dropzone_detected = False
        self.dropzone_center_point = None
        self.detected_baskets = []
        
        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Dropzone Detector Node initialized")
    
    def camera_enable_callback(self, msg):
        """Handle camera enable/disable commands"""
        if msg.data == "front" or msg.data == "all":
            self.detection_enabled = True
            self.get_logger().info("Dropzone detection enabled")
        else:
            self.detection_enabled = False
            self.dropzone_detected = False
            self.detected_baskets = []
            self.get_logger().info("Dropzone detection disabled")
    
    def image_callback(self, msg):
        """Process images from front camera for dropzone detection"""
        if not self.detection_enabled:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect dropzone baskets
            baskets_found, dropzone_point, debug_image = self.detect_dropzone_baskets(cv_image)
            
            # Update detection state
            if baskets_found >= self.get_parameter('min_baskets_for_dropzone').value:
                self.dropzone_detected = True
                self.dropzone_center_point = dropzone_point
                self.last_detection_time = self.get_clock().now()
                
                # Publish detection result
                detection_msg = String()
                detection_msg.data = "DROPZONE_DETECTED"
                self.dropzone_detected_pub.publish(detection_msg)
                
                # Publish dropzone point
                if dropzone_point:
                    self.dropzone_point_pub.publish(dropzone_point)
                
                self.get_logger().info(f"Dropzone detected with {baskets_found} baskets at ({dropzone_point.x:.1f}, {dropzone_point.y:.1f})")
            else:
                # Reset detection if timeout
                current_time = self.get_clock().now()
                timeout_duration = rclpy.duration.Duration(
                    seconds=self.get_parameter('detection_timeout').value)
                
                if (self.last_detection_time is None or 
                    (current_time - self.last_detection_time) > timeout_duration):
                    if self.dropzone_detected:
                        self.dropzone_detected = False
                        detection_msg = String()
                        detection_msg.data = "NONE"
                        self.dropzone_detected_pub.publish(detection_msg)
            
            # Publish debug image
            if debug_image is not None:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                    self.debug_image_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish debug image: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def detect_dropzone_baskets(self, image):
        """Detect dropzone baskets in the image"""
        try:
            # Create debug image
            debug_image = image.copy()
            height, width = image.shape[:2]
            
            # Convert to different color spaces
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Reset detected baskets
            self.detected_baskets = []
            
            # Method 1: Color-based detection for baskets
            baskets_found = self.detect_baskets_by_color(hsv, debug_image)
            
            if baskets_found < self.get_parameter('min_baskets_for_dropzone').value:
                # Method 2: Shape-based detection
                additional_baskets = self.detect_baskets_by_shape(gray, debug_image)
                baskets_found += additional_baskets
            
            # Calculate dropzone center if enough baskets found
            dropzone_point = None
            if baskets_found >= self.get_parameter('min_baskets_for_dropzone').value:
                dropzone_point = self.calculate_dropzone_center()
                
                # Draw dropzone center on debug image
                if dropzone_point:
                    center_x = int(dropzone_point.x)
                    center_y = int(dropzone_point.y)
                    cv2.circle(debug_image, (center_x, center_y), 10, (0, 0, 255), -1)
                    cv2.putText(debug_image, "DROPZONE CENTER", (center_x - 70, center_y - 20), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            return baskets_found, dropzone_point, debug_image
            
        except Exception as e:
            self.get_logger().error(f"Error in dropzone detection: {e}")
            return 0, None, image
    
    def detect_baskets_by_color(self, hsv_image, debug_image):
        """Detect baskets by color characteristics"""
        baskets_found = 0
        
        try:
            # Define color ranges for typical basket colors
            # Brown/wooden baskets
            brown_lower = np.array([10, 50, 50])
            brown_upper = np.array([20, 255, 255])
            brown_mask = cv2.inRange(hsv_image, brown_lower, brown_upper)
            
            # Blue plastic baskets
            blue_lower = np.array([100, 100, 100])
            blue_upper = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
            
            # Red baskets
            red_lower1 = np.array([0, 100, 100])
            red_upper1 = np.array([10, 255, 255])
            red_lower2 = np.array([170, 100, 100])
            red_upper2 = np.array([180, 255, 255])
            red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
            red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            
            # Green baskets
            green_lower = np.array([40, 100, 100])
            green_upper = np.array([80, 255, 255])
            green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
            
            # Combine all color masks
            combined_mask = cv2.bitwise_or(brown_mask, blue_mask)
            combined_mask = cv2.bitwise_or(combined_mask, red_mask)
            combined_mask = cv2.bitwise_or(combined_mask, green_mask)
            
            # Morphological operations to clean up
            kernel = np.ones((5, 5), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Analyze contours for basket-like shapes
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.get_parameter('min_contour_area').value:
                    continue
                
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                
                # Check aspect ratio (baskets are usually round/square when viewed from front)
                min_ratio = self.get_parameter('basket_aspect_ratio_min').value
                max_ratio = self.get_parameter('basket_aspect_ratio_max').value
                
                if min_ratio <= aspect_ratio <= max_ratio:
                    # Calculate center
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Add to detected baskets
                    basket_info = {
                        'center': (center_x, center_y),
                        'area': area,
                        'bbox': (x, y, w, h),
                        'method': 'color'
                    }
                    self.detected_baskets.append(basket_info)
                    baskets_found += 1
                    
                    # Draw detection on debug image
                    cv2.rectangle(debug_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.circle(debug_image, (center_x, center_y), 5, (255, 0, 0), -1)
                    cv2.putText(debug_image, f"BASKET{baskets_found}", (x, y - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            return baskets_found
            
        except Exception as e:
            self.get_logger().error(f"Error in color-based basket detection: {e}")
            return 0
    
    def detect_baskets_by_shape(self, gray_image, debug_image):
        """Detect baskets by circular/elliptical shapes"""
        baskets_found = 0
        
        try:
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray_image, (9, 9), 2)
            
            # Use HoughCircles to detect circular baskets
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=50,
                param1=50,
                param2=30,
                minRadius=20,
                maxRadius=100
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                
                for (x, y, r) in circles:
                    # Check if this circle is not too close to already detected baskets
                    too_close = False
                    for basket in self.detected_baskets:
                        bx, by = basket['center']
                        distance = np.sqrt((x - bx)**2 + (y - by)**2)
                        if distance < 60:  # Minimum distance between baskets
                            too_close = True
                            break
                    
                    if not too_close:
                        # Add to detected baskets
                        basket_info = {
                            'center': (x, y),
                            'area': np.pi * r * r,
                            'bbox': (x - r, y - r, 2 * r, 2 * r),
                            'method': 'shape'
                        }
                        self.detected_baskets.append(basket_info)
                        baskets_found += 1
                        
                        # Draw detection on debug image
                        cv2.circle(debug_image, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(debug_image, (x, y), 5, (0, 255, 0), -1)
                        cv2.putText(debug_image, f"CIRCLE{len(self.detected_baskets)}", 
                                  (x - 30, y - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            return baskets_found
            
        except Exception as e:
            self.get_logger().error(f"Error in shape-based basket detection: {e}")
            return 0
    
    def calculate_dropzone_center(self):
        """Calculate the center point of the dropzone based on detected baskets"""
        if len(self.detected_baskets) < 2:
            return None
        
        # Calculate weighted center based on basket positions and areas
        total_weight = 0
        weighted_x = 0
        weighted_y = 0
        
        for basket in self.detected_baskets:
            center_x, center_y = basket['center']
            weight = basket['area']  # Use area as weight
            
            weighted_x += center_x * weight
            weighted_y += center_y * weight
            total_weight += weight
        
        if total_weight > 0:
            center_x = weighted_x / total_weight
            center_y = weighted_y / total_weight
            
            # Create point message
            dropzone_point = Point()
            dropzone_point.x = float(center_x)
            dropzone_point.y = float(center_y)
            dropzone_point.z = float(len(self.detected_baskets))  # Number of baskets as confidence
            
            return dropzone_point
        
        return None
    
    def publish_status(self):
        """Publish current detection status"""
        status_msg = String()
        if not self.detection_enabled:
            status_msg.data = "DISABLED"
        elif self.dropzone_detected:
            status_msg.data = f"DROPZONE_DETECTED_({len(self.detected_baskets)}_BASKETS)"
        else:
            status_msg.data = f"SEARCHING_({len(self.detected_baskets)}_BASKETS_FOUND)"
        
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        dropzone_detector = DropzoneDetectorNode()
        rclpy.spin(dropzone_detector)
    except KeyboardInterrupt:
        pass
    finally:
        if 'dropzone_detector' in locals():
            dropzone_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
