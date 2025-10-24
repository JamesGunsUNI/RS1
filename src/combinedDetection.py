import rclpy
import numpy as np
import math
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException, Buffer, TransformListener
import time

class TrackedObject:
    def __init__(self, obj_id, class_name, distance, angle, x, y, map_x, map_y):
        self.id = obj_id
        self.class_name = class_name
        self.distance = distance
        self.angle = angle
        self.x = x  # local x
        self.y = y  # local y
        self.map_x = map_x  # global map x
        self.map_y = map_y  # global map y
        self.last_seen = time.time()
        self.confidence = 0.0
        self.detection_count = 1
        
    def update(self, distance, angle, x, y, map_x, map_y, confidence=None):
        self.distance = distance
        self.angle = angle
        self.x = x
        self.y = y
        weight = 0.3  # Weight for new position
        self.map_x = self.map_x * (1 - weight) + map_x * weight
        self.map_y = self.map_y * (1 - weight) + map_y * weight
        self.last_seen = time.time()
        self.detection_count += 1
        if confidence is not None:
            self.confidence = confidence

class CombinedDetectionNode(Node):
    def __init__(self):
        super().__init__('combined_detection_node')
        
        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image, "/camera/image", self.image_callback, 10
        )
        
        # Subscribe to LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )

        # Publishers for detection annotated images
        self.det_image_pub = self.create_publisher(
            Image, "/ultralytics/detection/image", 10
        )
        self.seg_image_pub = self.create_publisher(
            Image, "/ultralytics/segmentation/image", 10
        )

        # Publishers for detection data
        self.classes_pub = self.create_publisher(
            String, "/ultralytics/detection/classes", 10
        )
        
        # Publisher for fused detection results
        self.fused_detections_pub = self.create_publisher(
            String, "/fused_detections", 10
        )
        
        # Publisher for tracked objects (visualization)
        self.tracked_objects_pub = self.create_publisher(
            MarkerArray, "/tracked_objects", 10
        )
        
        # Load YOLO models
        self.detection_model = YOLO('/home/marcus/41068_ws/src/RS1/src/ModelV2.pt')
        self.segmentation_model = YOLO('yolo11m-seg.pt')
        
        # Declare parameters
        self.declare_parameter('log_detections', True)
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('min_lidar_distance', 0.1)
        self.declare_parameter('max_lidar_distance', 10.0)
        self.declare_parameter('min_points_per_object', 1)
        self.declare_parameter('max_gap_distance', 0.3)
        self.declare_parameter('tracking_timeout', 0.0)  # seconds - 0 means no timeout (infinite)
        self.declare_parameter('association_distance_threshold', 1.0)  # meters - increase for re-association
        self.declare_parameter('map_frame', 'map')  # map frame name
        self.declare_parameter('base_frame', 'base_link')  # robot base frame
        self.declare_parameter('camera_hfov', 1.3962634)  # ~80 degrees in radians
        self.declare_parameter('image_width', 640)
        
        # Data storage
        self.latest_detections = []  # [(class_name, confidence, bbox)]
        self.latest_lidar_objects = []  # [(distance, angle, x, y)]
        self.tracked_objects = {}  # {id: TrackedObject}
        self.next_object_id = 0
        self.published_marker_ids = set()  # Track which marker IDs have been published
        
        self.get_logger().info('Combined Detection Node Started with persistent map markers')

    def transform_point_to_map(self, x, y):
        try:
            base_frame = self.get_parameter('base_frame').value
            map_frame = self.get_parameter('map_frame').value
            
            # Get latest transform
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),  # Time=0 means "get latest available"
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Convert quaternion to yaw angle
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # Rotate and translate
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            map_x = transform.transform.translation.x + x * cos_yaw - y * sin_yaw
            map_y = transform.transform.translation.y + x * sin_yaw + y * cos_yaw
            
            return map_x, map_y
            
        except (TransformException, Exception) as e:
            self.get_logger().warn(f'Could not transform point to map: {str(e)}', throttle_duration_sec=5.0)
            return None, None

    def image_callback(self, data):
        # Convert ROS Image to numpy array
        array = np.array(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        # Get parameters
        conf_threshold = self.get_parameter('confidence_threshold').value
        log_detections = self.get_parameter('log_detections').value
        
        # Run detection model
        det_result = self.detection_model(array, conf=conf_threshold)
        result = det_result[0]
        
        # Publish annotated detection image
        if self.det_image_pub.get_subscription_count() > 0:
            det_annotated = result.plot(show=False)
            det_msg = self._create_image_msg(det_annotated)
            self.det_image_pub.publish(det_msg)
        
        # Publish segmentation image
        if self.seg_image_pub.get_subscription_count() > 0:
            seg_result = self.segmentation_model(array, conf=conf_threshold)
            seg_annotated = seg_result[0].plot(show=False)
            seg_msg = self._create_image_msg(seg_annotated)
            self.seg_image_pub.publish(seg_msg)
        
        # Store detection information for fusion
        boxes = result.boxes
        self.latest_detections = []
        
        if len(boxes) > 0:
            class_ids = boxes.cls.cpu().numpy().astype(int)
            confidences = boxes.conf.cpu().numpy()
            xywh = boxes.xywh.cpu().numpy()
            names = [result.names[i] for i in class_ids]
            
            for i in range(len(boxes)):
                self.latest_detections.append({
                    'class_name': names[i],
                    'confidence': confidences[i],
                    'bbox_center_x': xywh[i][0],
                    'bbox_center_y': xywh[i][1],
                    'bbox_width': xywh[i][2],
                    'bbox_height': xywh[i][3]
                })
            
            # Publish class names
            if self.classes_pub.get_subscription_count() > 0:
                self.classes_pub.publish(String(data=str(names)))
            
            # Log detections
            if log_detections:
                self.get_logger().info(f"Vision detected {len(boxes)} objects: {names}")
        
        # Perform fusion with LiDAR data
        self._fuse_detections()

    def laser_callback(self, msg):
        try:
            # Get parameters
            min_distance = self.get_parameter('min_lidar_distance').value
            max_distance = self.get_parameter('max_lidar_distance').value
            max_gap = self.get_parameter('max_gap_distance').value
            min_points = self.get_parameter('min_points_per_object').value
            
            # Filter valid ranges
            ranges = np.array(msg.ranges)
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            # Create list of valid points
            valid_points = []
            for i, range_val in enumerate(ranges):
                if (range_val >= min_distance and 
                    range_val <= max_distance and 
                    range_val != float('inf') and 
                    not math.isnan(range_val)):
                    angle = angle_min + i * angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    valid_points.append((x, y, angle, range_val))
            
            # Detect objects from LiDAR
            self.latest_lidar_objects = self._count_objects_with_positions(
                valid_points, max_gap, min_points
            )
            
            # Perform fusion with vision data
            self._fuse_detections()
                
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {str(e)}')

    def _count_objects_with_positions(self, points, max_gap, min_points):
        # Group nearby points and return list of object positions
        if not points:
            return []
        
        points.sort(key=lambda p: p[2])  # Sort by angle
        
        objects = []
        current_object = [points[0]]
        
        for i in range(1, len(points)):
            current_point = points[i]
            last_point = current_object[-1]
            
            dx = current_point[0] - last_point[0]
            dy = current_point[1] - last_point[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance <= max_gap:
                current_object.append(current_point)
            else:
                if len(current_object) >= min_points:
                    objects.append(current_object)
                current_object = [current_point]
        
        if len(current_object) >= min_points:
            objects.append(current_object)
        
        # Calculate average position and distance for each object
        object_data = []
        for obj in objects:
            avg_x = sum(p[0] for p in obj) / len(obj)
            avg_y = sum(p[1] for p in obj) / len(obj)
            avg_distance = sum(p[3] for p in obj) / len(obj)
            avg_angle = math.atan2(avg_y, avg_x)
            object_data.append((avg_distance, avg_angle, avg_x, avg_y))
        
        return object_data

    def _fuse_detections(self):
        if not self.latest_lidar_objects or not self.latest_detections:
            return
        
        camera_hfov = self.get_parameter('camera_hfov').value
        image_width = self.get_parameter('image_width').value
        
        # Associate LiDAR objects with vision detections
        for lidar_obj in self.latest_lidar_objects:
            distance, angle, x, y = lidar_obj
            
            # Convert LiDAR angle to image x coordinate
            # Assuming camera is centered and aligned with LiDAR
            if abs(angle) > camera_hfov / 2:
                continue  # Object outside camera FOV
            
            # Map angle to pixel position
            normalized_pos = (angle + camera_hfov/2) / camera_hfov
            pixel_x = normalized_pos * image_width
            
            # Find closest vision detection
            best_match = None
            best_distance = float('inf')
            
            for detection in self.latest_detections:
                bbox_x = detection['bbox_center_x']
                pixel_diff = abs(pixel_x - bbox_x)
                
                if pixel_diff < best_distance:
                    best_distance = pixel_diff
                    best_match = detection
            
            # If match found, update or create tracked object
            if best_match and best_distance < 100:  # 100 pixel threshold
                # Transform to map coordinates
                map_x, map_y = self.transform_point_to_map(x, y)
                
                if map_x is not None and map_y is not None:
                    self._update_tracked_object(
                        best_match['class_name'],
                        distance, angle, x, y, map_x, map_y,
                        best_match['confidence']
                    )
        
        # Clean up old tracked objects
        self._cleanup_old_tracks()
        
        # Publish tracking results
        self._publish_tracked_objects()

    def _update_tracked_object(self, class_name, distance, angle, x, y, map_x, map_y, confidence):
        # Convert to Python floats to avoid numpy type issues
        distance = float(distance)
        angle = float(angle)
        x = float(x)
        y = float(y)
        map_x = float(map_x)
        map_y = float(map_y)
        confidence = float(confidence)
        
        threshold = self.get_parameter('association_distance_threshold').value
        
        best_match_id = None
        best_match_dist = float('inf')
        
        for obj_id, tracked_obj in self.tracked_objects.items():
            if tracked_obj.class_name == class_name:
                dx = map_x - tracked_obj.map_x
                dy = map_y - tracked_obj.map_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist < best_match_dist and dist < threshold:
                    best_match_dist = dist
                    best_match_id = obj_id
        
        if best_match_id is not None:
            # Update existing track
            self.tracked_objects[best_match_id].update(distance, angle, x, y, map_x, map_y, confidence)
        else:
            # Create new track only if no existing object found nearby
            new_id = self.next_object_id
            self.next_object_id += 1
            self.tracked_objects[new_id] = TrackedObject(
                new_id, class_name, distance, angle, x, y, map_x, map_y
            )
            self.tracked_objects[new_id].confidence = confidence

    def _cleanup_old_tracks(self):
        timeout = self.get_parameter('tracking_timeout').value
        
        # If timeout is 0, never remove tracks
        if timeout <= 0:
            return
        
        current_time = time.time()
        
        to_remove = []
        for obj_id, tracked_obj in self.tracked_objects.items():
            if current_time - tracked_obj.last_seen > timeout:
                to_remove.append(obj_id)
        
        # Delete markers for removed objects
        if to_remove:
            self._delete_markers(to_remove)
        
        for obj_id in to_remove:
            del self.tracked_objects[obj_id]
            self.published_marker_ids.discard(obj_id)
            self.published_marker_ids.discard(obj_id + 1000)  # text marker

    def _delete_markers(self, obj_ids):
        map_frame = self.get_parameter('map_frame').value
        marker_array = MarkerArray()
        
        for obj_id in obj_ids:
            # Delete sphere marker
            marker = Marker()
            marker.header.frame_id = map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_objects"
            marker.id = obj_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
            
            # Delete text marker
            text_marker = Marker()
            text_marker.header.frame_id = map_frame
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "object_labels"
            text_marker.id = obj_id + 1000
            text_marker.action = Marker.DELETE
            marker_array.markers.append(text_marker)
        
        if marker_array.markers:
            self.tracked_objects_pub.publish(marker_array)

    def _publish_tracked_objects(self):
        if not self.tracked_objects:
            return
        
        map_frame = self.get_parameter('map_frame').value
        
        # Publish text summary
        summary_lines = [f"Tracking {len(self.tracked_objects)} objects:"]
        for obj_id, obj in self.tracked_objects.items():
            summary_lines.append(
                f"  ID:{obj.id} {obj.class_name} @ {obj.distance:.2f}m "
                f"map:({obj.map_x:.2f}, {obj.map_y:.2f}) conf:{obj.confidence:.2f}"
            )
        
        summary = "\n".join(summary_lines)
        self.fused_detections_pub.publish(String(data=summary))
        self.get_logger().info(summary)
        
        # Publish visualization markers
        marker_array = MarkerArray()
        current_marker_ids = set()
        
        for obj_id, obj in self.tracked_objects.items():
            marker = Marker()
            marker.header.frame_id = map_frame 
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_objects"
            marker.id = obj.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Use global map coordinates
            marker.pose.position.x = float(obj.map_x)
            marker.pose.position.y = float(obj.map_y)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            current_marker_ids.add(obj.id)
            
            # Add text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "object_labels"
            text_marker.id = obj.id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Use global map coordinates
            text_marker.pose.position.x = float(obj.map_x)
            text_marker.pose.position.y = float(obj.map_y)
            text_marker.pose.position.z = 0.8
            
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{obj.class_name}\n{obj.distance:.1f}m"
            
            marker_array.markers.append(text_marker)
            current_marker_ids.add(obj.id + 1000)
        
        self.published_marker_ids.update(current_marker_ids)
        self.tracked_objects_pub.publish(marker_array)

    def _create_image_msg(self, cv_image):
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = "rgb8"
        msg.step = cv_image.shape[1] * 3
        msg.data = cv_image.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CombinedDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down Combined Detection Node...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()