#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - Unified YOLO Vision System
Semua logic computer vision menggunakan YOLOv8 dari ultralytics
Terorganisir per kebutuhan untuk memudahkan debugging
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import os
from pathlib import Path

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# Import hardware configuration for centralized settings
from ..hardware.hardware_config import HardwareConfig

# Import YOLOv8 dari ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("❌ ultralytics not installed. Run: pip install ultralytics")

class VisionSystem(Node):
    """
    Unified Vision System untuk semua kebutuhan computer vision:
    1. Exit Gate Detection - keluar di tengah portal
    2. Object Detection - pencarian barang dengan alignment
    3. Dropzone Detection - lokasi drop barang
    """
    
    def __init__(self):
        super().__init__('vision_system')
        
        # Load hardware configuration
        self.hw_config = HardwareConfig()
        
        # Load YOLO models from central configuration
        models_dir = self.hw_config.config.get('vision', 'models_directory', 
                                             fallback='/home/vanszs/ros/Dirgagah-KAERTEI/kaertei_drone/models')
        self.models_path = Path(models_dir)
        self.models_path.mkdir(exist_ok=True)
        
        self.models = {}
        self.load_yolo_models()
        
        # Camera configuration from hardware config
        self.camera_type = "front"  # Default
        self.camera_index = self.hw_config.get_front_camera_index()
        self.camera_enabled = True
        
        # Initialize camera with settings from config
        self.cap = cv2.VideoCapture(self.camera_index)
        width, height = self.hw_config.get_camera_resolution()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.hw_config.get_camera_fps())
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Detection parameters from hardware config
        self.confidence_threshold = self.hw_config.get_detection_confidence()
        self.alignment_tolerance = self.hw_config.get_alignment_tolerance()
        
        # Calculate screen center from camera resolution
        width, height = self.hw_config.get_camera_resolution()
        self.screen_center_x = width // 2
        self.screen_center_y = height // 2
        
        # Current detection results
        self.current_detections = []
        self.current_target = None
        self.is_aligned = False
        
        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers - organized per function
        self.setup_publishers(qos)
        
        # Subscribers
        self.setup_subscribers(qos)
        
        # Processing timer
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 FPS
        
        self.get_logger().info("🎯 Vision System initialized")
        
    def load_yolo_models(self):
        """Load YOLO models untuk berbagai kebutuhan"""
        if not YOLO_AVAILABLE:
            self.get_logger().error("❌ YOLOv8 not available!")
            return
            
        try:
            # Get model paths from hardware config
            model_configs = {
                'general': self.hw_config.config.get('vision', 'general_model', fallback='yolov8n.pt'),
                'exit_gate': self.hw_config.config.get('vision', 'exit_gate_model', fallback='yolov8n.pt'),
                'objects': self.hw_config.config.get('vision', 'objects_model', fallback='yolov8n.pt'),
                'dropzone': self.hw_config.config.get('vision', 'dropzone_model', fallback='yolov8n.pt')
            }
            
            self.get_logger().info(f"📦 Using models directory: {self.models_path}")
            
            for model_name, model_file in model_configs.items():
                model_path = self.models_path / model_file
                
                # Cek jika custom model exist, jika tidak gunakan general
                if not model_path.exists():
                    if model_name != 'general':
                        self.get_logger().warn(f"⚠️ Custom model {model_file} not found, using general model")
                        self.models[model_name] = self.models.get('general')
                        continue
                    else:
                        # Download general model jika tidak ada
                        self.get_logger().info(f"📥 Downloading {model_file}...")
                
                try:
                    model = YOLO(str(model_path) if model_path.exists() else model_file)
                    self.models[model_name] = model
                    self.get_logger().info(f"✅ Loaded {model_name} model: {model_file}")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to load {model_name} model: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error loading YOLO models: {e}")
            
    def setup_publishers(self, qos):
        """Setup publishers terorganisir per fungsi"""
        
        # Exit Gate Detection
        self.exit_detected_pub = self.create_publisher(Bool, '/vision/exit_detected', qos)
        self.exit_center_pub = self.create_publisher(Point, '/vision/exit_center', qos)
        
        # Object Detection & Alignment
        self.object_detected_pub = self.create_publisher(String, '/vision/object_detected', qos)
        self.object_center_pub = self.create_publisher(Point, '/vision/object_center', qos)
        self.alignment_status_pub = self.create_publisher(Bool, '/vision/aligned', qos)
        self.alignment_error_pub = self.create_publisher(Point, '/vision/alignment_error', qos)
        
        # Dropzone Detection
        self.dropzone_detected_pub = self.create_publisher(Bool, '/vision/dropzone_detected', qos)
        self.dropzone_center_pub = self.create_publisher(Point, '/vision/dropzone_center', qos)
        
        # Debug & monitoring
        self.debug_image_pub = self.create_publisher(Image, '/vision/debug_image', qos)
        self.confidence_pub = self.create_publisher(Float32, '/vision/confidence', qos)
        
    def setup_subscribers(self, qos):
        """Setup subscribers untuk kontrol"""
        self.mode_sub = self.create_subscription(
            String, '/vision/mode', self.mode_callback, qos)
        self.camera_enable_sub = self.create_subscription(
            Bool, '/vision/camera_enable', self.camera_enable_callback, qos)
            
    def mode_callback(self, msg):
        """Handle vision mode changes"""
        mode = msg.data.strip().lower()
        self.current_mode = mode
        self.get_logger().info(f"🎯 Vision mode changed to: {mode}")
        
    def camera_enable_callback(self, msg):
        """Handle camera enable/disable"""
        self.camera_enabled = msg.data
        self.get_logger().info(f"📷 Camera {'enabled' if msg.data else 'disabled'}")
        
    def process_frame(self):
        """Main processing loop"""
        if not self.camera_enabled:
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("📷 Failed to read camera frame")
            return
            
        # Process berdasarkan mode current
        mode = getattr(self, 'current_mode', 'object_detection')
        
        if mode == 'exit_gate':
            self.process_exit_gate(frame)
        elif mode == 'object_detection':
            self.process_object_detection(frame)
        elif mode == 'dropzone':
            self.process_dropzone_detection(frame)
        else:
            self.process_general_detection(frame)
            
        # Publish debug image
        self.publish_debug_image(frame)
        
    def process_exit_gate(self, frame):
        """
        Deteksi exit gate dan hitung posisi tengah portal
        Untuk navigation keluar di tengah-tengah
        """
        model = self.models.get('exit_gate') or self.models.get('general')
        if not model:
            return
            
        try:
            # Deteksi dengan YOLO
            results = model(frame, conf=self.confidence_threshold)
            
            exit_detected = False
            exit_center = None
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for i in range(len(boxes)):
                        class_id = int(boxes.cls[i])
                        class_name = model.names[class_id]
                        conf = float(boxes.conf[i])
                        
                        # Cari class yang relevan untuk exit (door, window, gate, dll)
                        if self.is_exit_class(class_name):
                            x1, y1, x2, y2 = boxes.xyxy[i]
                            
                            # Hitung center portal
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            
                            exit_center = Point()
                            exit_center.x = float(center_x)
                            exit_center.y = float(center_y)
                            exit_center.z = conf
                            
                            exit_detected = True
                            
                            # Draw bounding box dan center point
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), -1)
                            cv2.putText(frame, f'EXIT: {conf:.2f}', (int(x1), int(y1-10)), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # Draw crosshair untuk target center
                            cv2.line(frame, (center_x-20, center_y), (center_x+20, center_y), (0, 255, 0), 2)
                            cv2.line(frame, (center_x, center_y-20), (center_x, center_y+20), (0, 255, 0), 2)
                            
                            break
                            
            # Publish results
            self.exit_detected_pub.publish(Bool(data=exit_detected))
            if exit_center:
                self.exit_center_pub.publish(exit_center)
                
        except Exception as e:
            self.get_logger().error(f"❌ Exit gate detection error: {e}")
            
    def process_object_detection(self, frame):
        """
        Deteksi object dan alignment untuk pickup
        Benda harus berada di center layar sebelum turun
        """
        model = self.models.get('objects') or self.models.get('general')
        if not model:
            return
            
        try:
            results = model(frame, conf=self.confidence_threshold)
            
            best_object = None
            best_conf = 0.0
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for i in range(len(boxes)):
                        class_id = int(boxes.cls[i])
                        class_name = model.names[class_id]
                        conf = float(boxes.conf[i])
                        
                        # Filter untuk object yang bisa dipickup
                        if self.is_pickup_object(class_name) and conf > best_conf:
                            x1, y1, x2, y2 = boxes.xyxy[i]
                            
                            # Hitung center object
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            
                            best_object = {
                                'name': class_name,
                                'confidence': conf,
                                'center_x': center_x,
                                'center_y': center_y,
                                'bbox': (int(x1), int(y1), int(x2), int(y2))
                            }
                            best_conf = conf
                            
            # Process best object
            if best_object:
                self.process_object_alignment(frame, best_object)
            else:
                # No object detected
                self.object_detected_pub.publish(String(data="NONE"))
                self.alignment_status_pub.publish(Bool(data=False))
                
        except Exception as e:
            self.get_logger().error(f"❌ Object detection error: {e}")
            
    def process_object_alignment(self, frame, obj):
        """
        Process alignment untuk object yang terdeteksi
        Hitung error dari center screen dan publish alignment status
        """
        center_x = obj['center_x']
        center_y = obj['center_y']
        
        # Hitung error dari center screen
        error_x = center_x - self.screen_center_x
        error_y = center_y - self.screen_center_y
        distance_from_center = np.sqrt(error_x**2 + error_y**2)
        
        # Check alignment
        is_aligned = distance_from_center <= self.alignment_tolerance
        
        # Draw object detection
        x1, y1, x2, y2 = obj['bbox']
        color = (0, 255, 0) if is_aligned else (0, 0, 255)
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.circle(frame, (center_x, center_y), 5, color, -1)
        cv2.putText(frame, f"{obj['name']}: {obj['confidence']:.2f}", 
                   (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Draw screen center dan alignment zone
        cv2.circle(frame, (self.screen_center_x, self.screen_center_y), 
                  self.alignment_tolerance, (255, 255, 0), 2)
        cv2.circle(frame, (self.screen_center_x, self.screen_center_y), 3, (255, 255, 0), -1)
        
        # Draw alignment line
        cv2.line(frame, (center_x, center_y), 
                (self.screen_center_x, self.screen_center_y), (255, 0, 255), 1)
        
        # Draw alignment info
        alignment_text = "ALIGNED" if is_aligned else f"ERROR: {distance_from_center:.1f}px"
        cv2.putText(frame, alignment_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        # Publish results
        object_center = Point()
        object_center.x = float(center_x)
        object_center.y = float(center_y)
        object_center.z = obj['confidence']
        
        alignment_error = Point()
        alignment_error.x = float(error_x)
        alignment_error.y = float(error_y)
        alignment_error.z = float(distance_from_center)
        
        self.object_detected_pub.publish(String(data=obj['name']))
        self.object_center_pub.publish(object_center)
        self.alignment_status_pub.publish(Bool(data=is_aligned))
        self.alignment_error_pub.publish(alignment_error)
        self.confidence_pub.publish(Float32(data=obj['confidence']))
        
    def process_dropzone_detection(self, frame):
        """
        Deteksi dropzone untuk drop object
        """
        model = self.models.get('dropzone') or self.models.get('general')
        if not model:
            return
            
        try:
            results = model(frame, conf=self.confidence_threshold)
            
            dropzone_detected = False
            dropzone_center = None
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for i in range(len(boxes)):
                        class_id = int(boxes.cls[i])
                        class_name = model.names[class_id]
                        conf = float(boxes.conf[i])
                        
                        if self.is_dropzone_class(class_name):
                            x1, y1, x2, y2 = boxes.xyxy[i]
                            
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            
                            dropzone_center = Point()
                            dropzone_center.x = float(center_x)
                            dropzone_center.y = float(center_y)
                            dropzone_center.z = conf
                            
                            dropzone_detected = True
                            
                            # Draw detection
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                            cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), -1)
                            cv2.putText(frame, f'DROPZONE: {conf:.2f}', (int(x1), int(y1-10)), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                            break
                            
            # Publish results
            self.dropzone_detected_pub.publish(Bool(data=dropzone_detected))
            if dropzone_center:
                self.dropzone_center_pub.publish(dropzone_center)
                
        except Exception as e:
            self.get_logger().error(f"❌ Dropzone detection error: {e}")
            
    def process_general_detection(self, frame):
        """General detection untuk debugging"""
        model = self.models.get('general')
        if not model:
            return
            
        try:
            results = model(frame, conf=self.confidence_threshold)
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for i in range(len(boxes)):
                        class_id = int(boxes.cls[i])
                        class_name = model.names[class_id]
                        conf = float(boxes.conf[i])
                        
                        x1, y1, x2, y2 = boxes.xyxy[i]
                        
                        # Draw all detections
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 1)
                        cv2.putText(frame, f'{class_name}: {conf:.2f}', (int(x1), int(y1-5)), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                                  
        except Exception as e:
            self.get_logger().error(f"❌ General detection error: {e}")
            
    def is_exit_class(self, class_name):
        """Check if detected class adalah exit gate"""
        exit_classes = ['door', 'window', 'gate', 'opening', 'entrance', 'exit']
        return any(exit_class in class_name.lower() for exit_class in exit_classes)
        
    def is_pickup_object(self, class_name):
        """Check if detected class adalah object yang bisa dipickup"""
        pickup_classes = ['bottle', 'cup', 'bowl', 'book', 'cell phone', 'remote', 
                         'mouse', 'keyboard', 'scissors', 'teddy bear', 'hair drier', 
                         'toothbrush', 'apple', 'orange', 'banana']
        return any(pickup_class in class_name.lower() for pickup_class in pickup_classes)
        
    def is_dropzone_class(self, class_name):
        """Check if detected class adalah dropzone"""
        dropzone_classes = ['bowl', 'sink', 'basket', 'container', 'box', 'bin']
        return any(dropzone_class in class_name.lower() for dropzone_class in dropzone_classes)
        
    def publish_debug_image(self, frame):
        """Publish debug image dengan annotations"""
        try:
            # Add overlay info
            cv2.putText(frame, f"Mode: {getattr(self, 'current_mode', 'general')}", 
                       (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.debug_image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish debug image: {e}")
            
    def destroy_node(self):
        """Cleanup"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        vision_system = VisionSystem()
        rclpy.spin(vision_system)
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_system' in locals():
            vision_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
