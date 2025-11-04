import rclpy
import numpy as np
import math
import json
import struct
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException, Buffer, TransformListener
import time

class TrackedObject:
    def __init__(self, obj_id, distance, angle, x, y, map_x, map_y, source='lidar'):
        self.id = obj_id
        self.class_name = 'unknown'
        self.distance = distance
        self.angle = angle
        self.x = x
        self.y = y
        self.map_x = map_x
        self.map_y = map_y
        self.last_seen = time.time()
        self.confidence = 0.0
        self.detection_count = 1
        self.source = source
        self.confirmed = False
        self.distance_history = [distance]  # Track distance consistency
        self.is_stable = False  # Only show marker when stable
        
    def update_position(self, distance, angle, x, y, map_x, map_y):
        self.distance = distance
        self.angle = angle
        self.x = x
        self.y = y
        weight = 0.3
        self.map_x = self.map_x * (1 - weight) + map_x * weight
        self.map_y = self.map_y * (1 - weight) + map_y * weight
        self.last_seen = time.time()
        self.detection_count += 1
        
        # Track distance history (keep last 5 readings)
        self.distance_history.append(distance)
        if len(self.distance_history) > 5:
            self.distance_history.pop(0)
        
        # Check if distance is stable (not jumping around)
        if len(self.distance_history) >= 3:
            avg_distance = sum(self.distance_history) / len(self.distance_history)
            max_deviation = max(abs(d - avg_distance) for d in self.distance_history)
            
            # Object is stable if distance doesn't vary more than 30cm
            if max_deviation < 0.3:
                self.is_stable = True
            else:
                self.is_stable = False
        
    def confirm_with_camera(self, class_name, confidence):
        self.class_name = class_name
        self.confidence = confidence
        self.confirmed = True
        self.source = 'confirmed'
        self.last_seen = time.time()
    
    def to_dict(self):
        return {
            'id': self.id,
            'class_name': self.class_name,
            'map_x': float(self.map_x),
            'map_y': float(self.map_y),
            'distance': float(self.distance),
            'angle': float(self.angle),
            'local_x': float(self.x),
            'local_y': float(self.y),
            'confidence': float(self.confidence),
            'detection_count': self.detection_count,
            'last_seen': self.last_seen,
            'source': self.source,
            'confirmed': self.confirmed
        }

class CombinedDetectionNode(Node):
    def __init__(self):
        super().__init__('combined_detection_node')
        
        # TF2 setup
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

        # Publishers
        self.det_image_pub = self.create_publisher(
            Image, "/ultralytics/detection/image", 10
        )
        self.classes_pub = self.create_publisher(
            String, "/ultralytics/detection/classes", 10
        )
        self.fused_detections_pub = self.create_publisher(
            String, "/fused_detections", 10
        )
        self.obstacles_array_pub = self.create_publisher(
            String, "/obstacles_array", 10
        )
        self.tracked_objects_pub = self.create_publisher(
            MarkerArray, "/tracked_objects", 10
        )
        
        # Load YOLO model
        self.detection_model = YOLO('/home/marcus/41068_ws/src/RS1/src/ModelV2.pt')
        
        # Declare parameters with improved defaults
        self.declare_parameter('log_detections', True)
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('min_lidar_distance', 0.1)
        self.declare_parameter('max_lidar_distance', 10.0)
        self.declare_parameter('min_points_per_object', 5)  # Increased from 2
        self.declare_parameter('max_gap_distance', 0.5)  # Increased from 0.3
        self.declare_parameter('tracking_timeout', 3.0)  # Reduced from 5.0
        self.declare_parameter('confirmed_timeout', 0.0)
        self.declare_parameter('association_distance_threshold', 1.2)  # Increased to merge better
        self.declare_parameter('camera_match_distance_threshold', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_hfov', 1.3962634)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Wall filtering parameters
        self.declare_parameter('enable_wall_filter', True)
        self.declare_parameter('wall_min_points', 20)  # Walls have many consecutive points
        self.declare_parameter('wall_max_width', 5.0)  # Walls are long/wide
        self.declare_parameter('wall_straightness_threshold', 0.95)  # How straight (0-1)
        
        # Consistency filtering parameters
        self.declare_parameter('require_stable_detections', True)
        self.declare_parameter('min_stable_detections', 3)  # Need 3+ consistent readings
        self.declare_parameter('max_distance_deviation', 0.3)  # Max 30cm variation
        
        # Data storage
        self.latest_detections = []
        self.tracked_objects = {}
        self.next_object_id = 0
        self.published_marker_ids = set()
        
        self.get_logger().info('object detection node started')

    def transform_point_to_map(self, x, y, z=0.0, source_frame=None):
        try:
            if source_frame is None:
                source_frame = self.get_parameter('base_frame').value
            
            map_frame = self.get_parameter('map_frame').value
            
            transform = self.tf_buffer.lookup_transform(
                map_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            map_x = transform.transform.translation.x + x * cos_yaw - y * sin_yaw
            map_y = transform.transform.translation.y + x * sin_yaw + y * cos_yaw
            
            return map_x, map_y
            
        except (TransformException, Exception) as e:
            self.get_logger().warn(f'Could not transform point: {str(e)}', throttle_duration_sec=5.0)
            return None, None

    def laser_callback(self, msg):
        try:
            min_distance = self.get_parameter('min_lidar_distance').value
            max_distance = self.get_parameter('max_lidar_distance').value
            max_gap = self.get_parameter('max_gap_distance').value
            min_points = self.get_parameter('min_points_per_object').value
            
            ranges = np.array(msg.ranges)
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            valid_points = []
            for i, range_val in enumerate(ranges):
                if (range_val >= min_distance and range_val <= max_distance and 
                    range_val != float('inf') and not math.isnan(range_val)):
                    angle = angle_min + i * angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    valid_points.append((x, y, angle, range_val))
            
            lidar_objects = self._count_objects_with_positions(valid_points, max_gap, min_points)
            
            # Filter small objects
            filtered_objects = []
            for obj_data in lidar_objects:
                distance, angle, x, y, num_points = obj_data
                if num_points >= min_points or (distance < 3.0 and num_points >= 3):
                    filtered_objects.append(obj_data)
            
            self._update_from_lidar(filtered_objects)
            self._cleanup_old_tracks()
            self._publish_tracked_objects()
                
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {str(e)}')

    def _count_objects_with_positions(self, points, max_gap, min_points):
        if not points:
            return []
        
        points.sort(key=lambda p: p[2])
        
        objects = []
        current_object = [points[0]]
        
        for i in range(1, len(points)):
            current_point = points[i]
            last_point = current_object[-1]
            
            dx = current_point[0] - last_point[0]
            dy = current_point[1] - last_point[1]
            distance = math.sqrt(dx*dx + dy*dy)
            angle_diff = abs(current_point[2] - last_point[2])
            
            avg_range = (current_point[3] + last_point[3]) / 2
            adaptive_gap = max_gap * (1 + avg_range * 0.1)
            
            if distance <= adaptive_gap and angle_diff < 0.5:
                current_object.append(current_point)
            else:
                if len(current_object) >= min_points:
                    objects.append(current_object)
                current_object = [current_point]
        
        if len(current_object) >= min_points:
            objects.append(current_object)
        
        # Get wall filtering parameters
        enable_wall_filter = self.get_parameter('enable_wall_filter').value
        wall_min_points = self.get_parameter('wall_min_points').value
        wall_max_width = self.get_parameter('wall_max_width').value
        wall_straightness_threshold = self.get_parameter('wall_straightness_threshold').value
        
        object_data = []
        for obj in objects:
            avg_x = sum(p[0] for p in obj) / len(obj)
            avg_y = sum(p[1] for p in obj) / len(obj)
            avg_distance = sum(p[3] for p in obj) / len(obj)
            avg_angle = math.atan2(avg_y, avg_x)
            num_points = len(obj)
            
            x_coords = [p[0] for p in obj]
            y_coords = [p[1] for p in obj]
            width = math.sqrt((max(x_coords) - min(x_coords))**2 + 
                             (max(y_coords) - min(y_coords))**2)
            
            # Filter out very small objects (likely noise) unless they're close
            if width < 0.1 and avg_distance > 2.0:
                continue
            
            # Wall detection and filtering
            if enable_wall_filter and self._is_wall(obj, num_points, width, wall_min_points, 
                                                     wall_max_width, wall_straightness_threshold):
                self.get_logger().debug(
                    f"Filtered out wall: {num_points} points, width={width:.2f}m",
                    throttle_duration_sec=5.0
                )
                continue
            
            object_data.append((avg_distance, avg_angle, avg_x, avg_y, num_points))
        
        return object_data
    
    def _is_wall(self, points, num_points, width, wall_min_points, wall_max_width, straightness_threshold):
        # Walls typically have many points
        if num_points < wall_min_points:
            return False
        
        # Walls are long
        if width < wall_max_width:
            return False
        
        # Check straightness using linear regression
        x_coords = np.array([p[0] for p in points])
        y_coords = np.array([p[1] for p in points])
        
        # Fit a line to the points
        if len(x_coords) < 3:
            return False
        
        # Calculate correlation coefficient (measure of straightness)
        # R² close to 1.0 means points are very straight (like a wall)
        try:
            # Handle vertical lines
            if np.std(x_coords) < 0.01:  # Nearly vertical
                r_squared = np.corrcoef(y_coords, np.arange(len(y_coords)))[0, 1] ** 2
            else:
                r_squared = np.corrcoef(x_coords, y_coords)[0, 1] ** 2
            
            # If points form a very straight line with many points, it's likely a wall
            if r_squared > straightness_threshold:
                return True
        except:
            pass
        
        return False

    def _update_from_lidar(self, lidar_objects):
        threshold = self.get_parameter('association_distance_threshold').value
        matched_object_ids = set()
        
        for distance, angle, x, y, num_points in lidar_objects:
            map_x, map_y = self.transform_point_to_map(x, y)
            
            if map_x is None or map_y is None:
                continue
            
            distance = float(distance)
            angle = float(angle)
            x = float(x)
            y = float(y)
            map_x = float(map_x)
            map_y = float(map_y)
            
            # Find closest existing object
            best_match_id = None
            best_match_dist = float('inf')
            
            for obj_id, tracked_obj in self.tracked_objects.items():
                dx = map_x - tracked_obj.map_x
                dy = map_y - tracked_obj.map_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist < best_match_dist:
                    best_match_dist = dist
                    best_match_id = obj_id
            
            if best_match_id is not None and best_match_dist < threshold:
                # Update existing marker position
                self.tracked_objects[best_match_id].update_position(
                    distance, angle, x, y, map_x, map_y
                )
                matched_object_ids.add(best_match_id)
            else:
                # Only create new object if we don't have many unconfirmed ones already
                # This prevents marker spam
                unconfirmed_count = sum(1 for obj in self.tracked_objects.values() if not obj.confirmed)
                
                # Only allow new markers if:
                # 1. We have strong evidence (many points)
                # 2. We don't already have too many unconfirmed objects
                if num_points >= self.get_parameter('min_points_per_object').value and unconfirmed_count < 10:
                    new_id = self.next_object_id
                    self.next_object_id += 1
                    self.tracked_objects[new_id] = TrackedObject(
                        new_id, distance, angle, x, y, map_x, map_y, source='lidar'
                    )
                    matched_object_ids.add(new_id)
                    self.get_logger().debug(f"New object ID:{new_id} @ ({map_x:.2f}, {map_y:.2f})")

    def image_callback(self, data):
        array = np.array(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        conf_threshold = self.get_parameter('confidence_threshold').value
        log_detections = self.get_parameter('log_detections').value
        
        det_result = self.detection_model(array, conf=conf_threshold)
        result = det_result[0]
        
        if self.det_image_pub.get_subscription_count() > 0:
            det_annotated = result.plot(show=False)
            det_msg = self._create_image_msg(det_annotated)
            self.det_image_pub.publish(det_msg)
        
        boxes = result.boxes
        unconfirmed_count = sum(1 for obj in self.tracked_objects.values() if not obj.confirmed)
        
        if len(boxes) > 0 or unconfirmed_count > 0:
            self.get_logger().info(
                f"Camera: {len(boxes)} detections | Tracking: {len(self.tracked_objects)} "
                f"({unconfirmed_count} unconfirmed)",
                throttle_duration_sec=2.0
            )
        
        if len(boxes) > 0:
            class_ids = boxes.cls.cpu().numpy().astype(int)
            confidences = boxes.conf.cpu().numpy()
            xywh = boxes.xywh.cpu().numpy()
            names = [result.names[i] for i in class_ids]
            
            detections = []
            for i in range(len(boxes)):
                detections.append({
                    'class_name': names[i],
                    'confidence': confidences[i],
                    'bbox_center_x': xywh[i][0],
                    'bbox_center_y': xywh[i][1],
                    'bbox_width': xywh[i][2],
                    'bbox_height': xywh[i][3]
                })
            
            if self.classes_pub.get_subscription_count() > 0:
                self.classes_pub.publish(String(data=str(names)))
            
            if log_detections:
                self.get_logger().info(f"Camera: {names}")
            
            self._match_camera_to_lidar(detections)
        
        self._publish_tracked_objects()

    def _match_camera_to_lidar(self, detections):
        camera_hfov = self.get_parameter('camera_hfov').value
        image_width = self.get_parameter('image_width').value
        max_match_distance = 10.0
        
        for detection in detections:
            class_name = detection['class_name']
            confidence = detection['confidence']
            bbox_x = detection['bbox_center_x']
            bbox_width = detection['bbox_width']
            
            normalized_x = (bbox_x - image_width/2) / (image_width/2)
            camera_angle = normalized_x * (camera_hfov / 2)
            normalized_width = bbox_width / image_width
            angle_uncertainty = normalized_width * (camera_hfov / 2) * 0.5
            
            try:
                camera_frame = self.get_parameter('camera_frame').value
                base_frame = self.get_parameter('base_frame').value
                
                transform = self.tf_buffer.lookup_transform(
                    base_frame, camera_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                qx, qy, qz, qw = (transform.transform.rotation.x,
                                 transform.transform.rotation.y,
                                 transform.transform.rotation.z,
                                 transform.transform.rotation.w)
                camera_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                                       1.0 - 2.0 * (qy * qy + qz * qz))
                detection_angle_base = camera_angle + camera_yaw
            except:
                detection_angle_base = camera_angle
            
            best_match_id = None
            best_match_score = float('inf')
            
            for obj_id, tracked_obj in self.tracked_objects.items():
                if tracked_obj.confirmed:
                    continue
                
                angle_diff = tracked_obj.angle - detection_angle_base
                while angle_diff > math.pi:
                    angle_diff -= 2*math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2*math.pi
                angle_diff_abs = abs(angle_diff)
                
                distance = math.sqrt(tracked_obj.x**2 + tracked_obj.y**2)
                
                base_fov_margin = camera_hfov / 2 + 0.3
                distance_tolerance = min(distance * 0.1, 0.5)
                total_tolerance = base_fov_margin + distance_tolerance + angle_uncertainty
                
                if angle_diff_abs > total_tolerance or distance > max_match_distance or distance < 0.1:
                    continue
                
                if distance < 3.0:
                    angle_weight, distance_weight = 2.0, 0.5
                elif distance < 6.0:
                    angle_weight, distance_weight = 1.5, 0.3
                else:
                    angle_weight, distance_weight = 1.0, 0.1
                
                score = angle_diff_abs * angle_weight + (distance * distance_weight)
                
                if score < best_match_score:
                    best_match_score = score
                    best_match_id = obj_id
            
            if best_match_id is not None:
                best_obj = self.tracked_objects[best_match_id]
                best_distance = math.sqrt(best_obj.x**2 + best_obj.y**2)
                
                if best_distance < 3.0:
                    score_threshold = 1.5
                elif best_distance < 6.0:
                    score_threshold = 2.5
                else:
                    score_threshold = 4.0
                
                if best_match_score < score_threshold:
                    best_obj.confirm_with_camera(class_name, confidence)
                    self.get_logger().info(
                        f"✓ ID:{best_match_id} → {class_name} @ {best_distance:.1f}m"
                    )

    def _cleanup_old_tracks(self):
        tracking_timeout = self.get_parameter('tracking_timeout').value
        confirmed_timeout = self.get_parameter('confirmed_timeout').value
        current_time = time.time()
        
        to_remove = []
        
        for obj_id, tracked_obj in self.tracked_objects.items():
            time_since_seen = current_time - tracked_obj.last_seen
            
            if not tracked_obj.confirmed:
                if time_since_seen > tracking_timeout:
                    to_remove.append(obj_id)
                elif tracked_obj.detection_count < 3 and time_since_seen > 1.0:
                    to_remove.append(obj_id)
            elif tracked_obj.confirmed and confirmed_timeout > 0:
                if time_since_seen > confirmed_timeout:
                    to_remove.append(obj_id)
        
        if to_remove:
            self._delete_markers(to_remove)
        
        for obj_id in to_remove:
            del self.tracked_objects[obj_id]
            self.published_marker_ids.discard(obj_id)
            self.published_marker_ids.discard(obj_id + 1000)

    def _delete_markers(self, obj_ids):
        map_frame = self.get_parameter('map_frame').value
        marker_array = MarkerArray()
        
        for obj_id in obj_ids:
            marker = Marker()
            marker.header.frame_id = map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_objects"
            marker.id = obj_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
            
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
            self.obstacles_array_pub.publish(String(data=json.dumps([])))
            return
        
        map_frame = self.get_parameter('map_frame').value
        require_stable = self.get_parameter('require_stable_detections').value
        
        # Filter objects: only show confirmed OR stable unconfirmed
        visible_objects = {}
        for obj_id, obj in self.tracked_objects.items():
            if obj.confirmed or (not require_stable) or obj.is_stable:
                visible_objects[obj_id] = obj
        
        confirmed_count = sum(1 for obj in visible_objects.values() if obj.confirmed)
        
        summary_lines = [f"{confirmed_count} confirmed, {len(visible_objects)-confirmed_count} waiting"]
        for obj in visible_objects.values():
            if obj.confirmed:
                summary_lines.append(
                    f"  ✓ {obj.class_name} @ {obj.distance:.1f}m [{obj.map_x:.1f}, {obj.map_y:.1f}]"
                )
        
        self.fused_detections_pub.publish(String(data="\n".join(summary_lines)))
        
        # Publish JSON array (only visible objects)
        obstacles_array = [obj.to_dict() for obj in visible_objects.values()]
        self.obstacles_array_pub.publish(String(data=json.dumps(obstacles_array, indent=2)))
        
        # Publish markers (only for visible objects)
        marker_array = MarkerArray()
        for obj in visible_objects.values():
            marker = Marker()
            marker.header.frame_id = map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_objects"
            marker.id = obj.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(obj.map_x)
            marker.pose.position.y = float(obj.map_y)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            if obj.confirmed:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.5
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.3
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.6, 0.6, 0.6, 0.7
            
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(marker)
            
            # Text
            text = Marker()
            text.header = marker.header
            text.ns = "object_labels"
            text.id = obj.id + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(obj.map_x)
            text.pose.position.y = float(obj.map_y)
            text.pose.position.z = 1.0
            text.scale.z = 0.3
            
            if obj.confirmed:
                text.color.r, text.color.g, text.color.b, text.color.a = 0.0, 1.0, 0.0, 1.0
                text.text = f"{obj.class_name}\n{obj.distance:.1f}m"
            else:
                text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 0.0, 1.0
                text.text = f"?\n{obj.distance:.1f}m"
            
            text.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(text)
        
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
        print("\nShutting down Two-Stage Detection Node...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()