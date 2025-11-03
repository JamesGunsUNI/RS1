#!/usr/bin/env python3
# ROS2 YOLO Object Detection Node with Multiple Output Options

import rclpy
import numpy as np
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

class ImageRecognition(Node):
    def __init__(self):
        super().__init__('image_recognition_node')
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image, "/camera/image", self.image_callback, 10
        )

        # Publisher for detection annotated image
        self.det_image_pub = self.create_publisher(
            Image, "/ultralytics/detection/image", 10
        )

        # Publisher for segmentation annotated image
        self.seg_image_pub = self.create_publisher(
            Image, "/ultralytics/segmentation/image", 10
        )

        # Publisher for detected class names
        self.classes_pub = self.create_publisher(
            String, "/ultralytics/detection/classes", 10
        )
        
        # Publisher for structured detection messages
        self.detections_pub = self.create_publisher(
            Detection2DArray, "/ultralytics/detections", 10
        )
        
        # Load YOLO models
        self.detection_model = YOLO('/home/marcus/41068_ws/src/RS1/src/best.pt')
        self.segmentation_model = YOLO('yolo11m-seg.pt')
        
        # Declare parameters for optional features
        self.declare_parameter('publish_detections_msg', True)
        self.declare_parameter('log_detections', True)
        self.declare_parameter('confidence_threshold', 0.1)
        
        self.get_logger().info('Image Recognition Node Started')

    def image_callback(self, data):
        # Convert ROS Image to numpy array
        array = np.array(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        # Get parameters
        conf_threshold = self.get_parameter('confidence_threshold').value
        log_detections = self.get_parameter('log_detections').value
        publish_det_msg = self.get_parameter('publish_detections_msg').value
        
        # Run detection model
        det_result = self.detection_model(array, conf=conf_threshold)
        result = det_result[0]
        
        # Publish annotated detection image (if there are subscribers)
        if self.det_image_pub.get_subscription_count() > 0:
            det_annotated = result.plot(show=False)
            det_msg = self._create_image_msg(det_annotated)
            self.det_image_pub.publish(det_msg)
        
        # Publish segmentation image (if there are subscribers)
        if self.seg_image_pub.get_subscription_count() > 0:
            seg_result = self.segmentation_model(array, conf=conf_threshold)
            seg_annotated = seg_result[0].plot(show=False)
            seg_msg = self._create_image_msg(seg_annotated)
            self.seg_image_pub.publish(seg_msg)
        
        # Get detection information
        boxes = result.boxes
        if len(boxes) > 0:
            class_ids = boxes.cls.cpu().numpy().astype(int)
            confidences = boxes.conf.cpu().numpy()
            coordinates = boxes.xyxy.cpu().numpy()  # [x1, y1, x2, y2]
            xywh = boxes.xywh.cpu().numpy()  # [center_x, center_y, width, height]
            names = [result.names[i] for i in class_ids]
            
            # Publish class names as string
            if self.classes_pub.get_subscription_count() > 0:
                self.classes_pub.publish(String(data=str(names)))
            
            # Publish structured detection messages
            if publish_det_msg and self.detections_pub.get_subscription_count() > 0:
                self._publish_detections(data.header, class_ids, confidences, 
                                        xywh, result.names)
            
            # Log detections to console
            if log_detections:
                self.get_logger().info(f"Detected {len(boxes)} objects:")
                for i in range(len(boxes)):
                    class_name = result.names[class_ids[i]]
                    confidence = confidences[i]
                    x1, y1, x2, y2 = coordinates[i]
                    self.get_logger().info(
                        f"  [{i}] {class_name} ({confidence:.2f}) at "
                        f"[{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]"
                    )
        else:
            # Publish empty detection array if no objects detected
            if publish_det_msg and self.detections_pub.get_subscription_count() > 0:
                empty_msg = Detection2DArray()
                empty_msg.header = data.header
                self.detections_pub.publish(empty_msg)

    
    def _create_image_msg(self, cv_image):
        """Convert numpy array to ROS Image message"""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = "rgb8"
        msg.step = cv_image.shape[1] * 3
        msg.data = cv_image.tobytes()
        return msg
    
    def _publish_detections(self, header, class_ids, confidences, xywh, names_dict):
        """Publish detections as Detection2DArray message"""
        det_array = Detection2DArray()
        det_array.header = header
        
        for i in range(len(class_ids)):
            detection = Detection2D()
            
            # Set bounding box center and size
            detection.bbox.center.position.x = float(xywh[i][0])
            detection.bbox.center.position.y = float(xywh[i][1])
            detection.bbox.size_x = float(xywh[i][2])
            detection.bbox.size_y = float(xywh[i][3])
            
            # Set detection hypothesis (class and confidence)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(names_dict[class_ids[i]])
            hypothesis.hypothesis.score = float(confidences[i])
            detection.results.append(hypothesis)
            
            det_array.detections.append(detection)
        
        self.detections_pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        ultralytics = ImageRecognition()
        rclpy.spin(ultralytics)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down Image Recognition Node...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()