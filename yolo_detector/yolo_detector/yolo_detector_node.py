#!/usr/bin/env python3
"""
Enhanced YOLOv8 Detector Node for ROS 2
Publishes additional diagnostic and visualization topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import String, Float32, Int32
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import json


class YoloDetectorEnhancedNode(Node):
    """Enhanced ROS 2 node for YOLOv8 object detection with diagnostics."""

    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_topic', '/kitti/camera/image')
        self.declare_parameter('max_det', 300)
        self.declare_parameter('device', '')
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        image_topic = self.get_parameter('image_topic').value
        self.max_det = self.get_parameter('max_det').value
        device = self.get_parameter('device').value
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'IoU threshold: {self.iou_threshold}')
        
        # Load YOLO model
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            if device:
                self.model.to(device)
            self.get_logger().info(f'Model loaded successfully on device: {self.model.device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Create subscriber
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # Create publishers - Original
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.viz_pub = self.create_publisher(Image, '/detections/image', 10)
        
        # Create publishers - Diagnostics
        self.fps_pub = self.create_publisher(Float32, '/detections/fps', 10)
        self.count_pub = self.create_publisher(Int32, '/detections/count', 10)
        self.inference_time_pub = self.create_publisher(Float32, '/detections/inference_time', 10)
        self.stats_pub = self.create_publisher(String, '/detections/stats', 10)
        
        # Statistics
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.class_counts = {}  # Track detections per class
        
        self.get_logger().info('Enhanced YOLO Detector Node initialized')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /detections (Detection2DArray)')
        self.get_logger().info('  - /detections/image (Image)')
        self.get_logger().info('  - /detections/fps (Float32)')
        self.get_logger().info('  - /detections/count (Int32)')
        self.get_logger().info('  - /detections/inference_time (Float32)')
        self.get_logger().info('  - /detections/stats (String/JSON)')

    def image_callback(self, msg):
        """Process incoming images and publish detections."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Run YOLO inference
        start_time = time.time()
        try:
            results = self.model(
                cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=self.max_det,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return
        
        inference_time = (time.time() - start_time) * 1000  # ms
        
        # Update statistics
        self.frame_count += 1
        self.total_inference_time += inference_time
        avg_inference_time = self.total_inference_time / self.frame_count
        fps = 1000.0 / avg_inference_time if avg_inference_time > 0 else 0
        
        # Parse detection results
        detections = []
        class_counts_frame = {}
        
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
                
            for box in boxes:
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue
                
                cls = int(box.cls[0])
                xyxy = box.xyxy[0].cpu().numpy()
                
                # Update class counts
                class_name = self.model.names[cls]
                class_counts_frame[class_name] = class_counts_frame.get(class_name, 0) + 1
                self.class_counts[class_name] = self.class_counts.get(class_name, 0) + 1
                
                # Create Detection2D message
                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float((xyxy[0] + xyxy[2]) / 2.0)
                det.bbox.center.position.y = float((xyxy[1] + xyxy[3]) / 2.0)
                det.bbox.size_x = float(xyxy[2] - xyxy[0])
                det.bbox.size_y = float(xyxy[3] - xyxy[1])
                
                obj_hyp = ObjectHypothesisWithPose()
                obj_hyp.hypothesis.class_id = str(cls)
                obj_hyp.hypothesis.score = conf
                det.results.append(obj_hyp)
                
                detections.append(det)
        
        # Publish Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.detections = detections
        self.det_pub.publish(det_array)
        
        # Publish annotated image
        try:
            annotated = results[0].plot()
            viz_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            viz_msg.header = msg.header
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to create visualization: {e}')
        
        # Publish diagnostics
        fps_msg = Float32()
        fps_msg.data = fps
        self.fps_pub.publish(fps_msg)
        
        count_msg = Int32()
        count_msg.data = len(detections)
        self.count_pub.publish(count_msg)
        
        inference_msg = Float32()
        inference_msg.data = inference_time
        self.inference_time_pub.publish(inference_msg)
        
        # Publish detailed stats as JSON
        stats = {
            'frame': self.frame_count,
            'detections': len(detections),
            'inference_time_ms': round(inference_time, 2),
            'avg_inference_time_ms': round(avg_inference_time, 2),
            'fps': round(fps, 2),
            'detections_by_class': class_counts_frame,
            'total_detections_by_class': self.class_counts
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)
        
        # Log performance info (every 30 frames)
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {len(detections)} objects | '
                f'Inference: {inference_time:.1f}ms | '
                f'Avg: {avg_inference_time:.1f}ms ({fps:.1f} FPS)'
            )


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    try:
        node = YoloDetectorEnhancedNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()