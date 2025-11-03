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
        self.class_name = 'unknown'  # Initially unknown until camera confirms
        self.distance = distance
        self.angle = angle
        self.x = x  # local x
        self.y = y  # local y
        self.map_x = map_x  # global map x
        self.map_y = map_y  # global map y
        self.last_seen = time.time()
        self.confidence = 0.0
        self.detection_count = 1
        self.source = source  # 'lidar' = unconfirmed, 'confirmed' = camera confirmed
        self.confirmed = False  # Whether camera has identified this object
        
    def update_position(self, distance, angle, x, y, map_x, map_y):
        """Update position from LiDAR"""
        self.distance = distance
        self.angle = angle
        self.x = x
        self.y = y
        weight = 0.3  # Weight for new position
        self.map_x = self.map_x * (1 - weight) + map_x * weight
        self.map_y = self.map_y * (1 - weight) + map_y * weight
        self.last_seen = time.time()
        self.detection_count += 1
        
    def confirm_with_camera(self, class_name, confidence):
        """Confirm and identify object with camera"""
        self.class_name = class_name
        self.confidence = confidence
        self.confirmed = True
        self.source = 'confirmed'
        self.last_seen = time.time()
    
    def to_dict(self):
        """Convert to dictionary for JSON serialization"""
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

        # Publishers for detection data
        self.classes_pub = self.create_publisher(
            String, "/ultralytics/detection/classes", 10
        )
        
        # Publisher for fused detection results (human-readable)
        self.fused_detections_pub = self.create_publisher(
            String, "/fused_detections", 10
        )
        
        # Publisher for obstacles array (for path planning)
        self.obstacles_array_pub = self.create_publisher(
            String, "/obstacles_array", 10
        )
        
        # Publisher for tracked objects (visualization)
        self.tracked_objects_pub = self.create_publisher(
            MarkerArray, "/tracked_objects", 10
        )
        
        # Load YOLO model
        self.detection_model = YOLO('/home/marcus/41068_ws/src/RS1/src/ModelV2.pt')
        
        # Declare parameters
        self.declare_parameter('log_detections', True)
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('min_lidar_distance', 0.1)
        self.declare_parameter('max_lidar_distance', 10.0)
        self.declare_parameter('min_points_per_object', 2)  
        self.declare_parameter('max_gap_distance', 0.3)
        self.declare_parameter('tracking_timeout', 5.0)  # seconds to keep unconfirmed objects
        self.declare_parameter('confirmed_timeout', 0.0)  # 0 = infinite for confirmed objects
        self.declare_parameter('association_distance_threshold', 1.5)  # meters for LiDAR object merging
        self.declare_parameter('camera_match_distance_threshold', 2.0)  # meters for camera-LiDAR matching
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_hfov', 1.3962634)  # ~80 degrees in radians
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Data storage
        self.latest_detections = []  # [(class_name, confidence, bbox)]
        self.tracked_objects = {}  # {id: TrackedObject}
        self.next_object_id = 0
        self.published_marker_ids = set()
        
        self.get_logger().info('Two-Stage Detection Node Started: LiDAR scans → Camera confirms')

    def transform_point_to_map(self, x, y, z=0.0, source_frame=None):
        """Transform a point from source frame to map frame"""
        try:
            if source_frame is None:
                source_frame = self.get_parameter('base_frame').value
            
            map_frame = self.get_parameter('map_frame').value
            
            # Get latest transform
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Convert quaternion to yaw
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

    def laser_callback(self, msg):
        """Stage 1: LiDAR detects potential objects and creates grey markers"""
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
            lidar_objects = self._count_objects_with_positions(
                valid_points, max_gap, min_points
            )
            
            # Update or create tracked objects from LiDAR detections
            self._update_from_lidar(lidar_objects)
            
            # Clean up old tracks
            self._cleanup_old_tracks()
            
            # Publish visualization
            self._publish_tracked_objects()
                
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {str(e)}')

    def _count_objects_with_positions(self, points, max_gap, min_points):
        """Group nearby LiDAR points into objects"""
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

    def _update_from_lidar(self, lidar_objects):
        """Update tracked objects from LiDAR detections"""
        threshold = self.get_parameter('association_distance_threshold').value
        matched_object_ids = set()
        
        for distance, angle, x, y in lidar_objects:
            # Transform to map coordinates
            map_x, map_y = self.transform_point_to_map(x, y)
            
            if map_x is None or map_y is None:
                continue
            
            # Convert to Python floats
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
            
            # Update existing or create new
            if best_match_id is not None and best_match_dist < threshold:
                # Update existing track
                self.tracked_objects[best_match_id].update_position(
                    distance, angle, x, y, map_x, map_y
                )
                matched_object_ids.add(best_match_id)
            else:
                # Create new unconfirmed object (grey marker)
                new_id = self.next_object_id
                self.next_object_id += 1
                self.tracked_objects[new_id] = TrackedObject(
                    new_id, distance, angle, x, y, map_x, map_y, source='lidar'
                )
                matched_object_ids.add(new_id)
                self.get_logger().info(
                    f"LiDAR detected new object ID:{new_id} at ({map_x:.2f}, {map_y:.2f}) - waiting for camera confirmation"
                )

    def image_callback(self, data):
        """Stage 2: Camera confirms and identifies objects"""
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
        
        # Process detections
        boxes = result.boxes
        
        # DEBUG: Log camera status periodically
        unconfirmed_count = sum(1 for obj in self.tracked_objects.values() if not obj.confirmed)
        if len(boxes) > 0 or unconfirmed_count > 0:
            self.get_logger().info(
                f"Camera: {len(boxes)} detections, {unconfirmed_count} unconfirmed objects waiting",
                throttle_duration_sec=2.0
            )
        
        if len(boxes) > 0:
            class_ids = boxes.cls.cpu().numpy().astype(int)
            confidences = boxes.conf.cpu().numpy()
            xywh = boxes.xywh.cpu().numpy()
            names = [result.names[i] for i in class_ids]
            
            # Store detections
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
            
            # Publish class names
            if self.classes_pub.get_subscription_count() > 0:
                self.classes_pub.publish(String(data=str(names)))
            
            # Log detections
            if log_detections:
                self.get_logger().info(f"Camera detected {len(boxes)} objects: {names}")
            
            # Match camera detections to LiDAR-detected objects
            self._match_camera_to_lidar(detections)
        
        # Publish updated tracking results
        self._publish_tracked_objects()

    def _match_camera_to_lidar(self, detections):
        """Match camera detections to existing LiDAR-detected objects with improved long-range matching"""
        camera_hfov = self.get_parameter('camera_hfov').value
        image_width = self.get_parameter('image_width').value
        
        # More generous distance threshold for matching (was 2.0m)
        max_match_distance = 10.0  # Allow matching up to 10 meters
        
        self.get_logger().info(f"=== Matching {len(detections)} camera detection(s) ===")
        
        for idx, detection in enumerate(detections):
            class_name = detection['class_name']
            confidence = detection['confidence']
            bbox_x = detection['bbox_center_x']
            bbox_width = detection['bbox_width']
            
            # Calculate angle from camera center (in camera frame)
            normalized_x = (bbox_x - image_width/2) / (image_width/2)
            camera_angle = normalized_x * (camera_hfov / 2)
            
            # Estimate angular uncertainty based on bbox width
            # Larger objects = more uncertainty about exact center
            normalized_width = bbox_width / image_width
            angle_uncertainty = normalized_width * (camera_hfov / 2) * 0.5  # Half bbox width as uncertainty
            
            self.get_logger().info(
                f"  {class_name}: bbox_x={bbox_x:.0f}, angle={math.degrees(camera_angle):.1f}°, "
                f"uncertainty=±{math.degrees(angle_uncertainty):.1f}°"
            )
            
            # Get camera transform to base_link
            try:
                camera_frame = self.get_parameter('camera_frame').value
                base_frame = self.get_parameter('base_frame').value
                
                transform = self.tf_buffer.lookup_transform(
                    base_frame,
                    camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                # Get camera yaw relative to base_link
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                camera_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                                    1.0 - 2.0 * (qy * qy + qz * qz))
                
                # Transform detection angle to base_link frame
                detection_angle_base = camera_angle + camera_yaw
                
            except (TransformException, Exception) as e:
                self.get_logger().warn(
                    f'Could not get camera transform, assuming aligned: {str(e)}',
                    throttle_duration_sec=5.0
                )
                detection_angle_base = camera_angle
            
            # Find best matching unconfirmed object
            best_match_id = None
            best_match_score = float('inf')
            candidates = []
            
            for obj_id, tracked_obj in self.tracked_objects.items():
                # Skip already confirmed objects
                if tracked_obj.confirmed:
                    continue
                
                # Calculate angle difference
                angle_diff = tracked_obj.angle - detection_angle_base
                
                # Normalize angle difference to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2*math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2*math.pi
                
                angle_diff_abs = abs(angle_diff)
                
                # Calculate distance from robot
                distance = math.sqrt(tracked_obj.x**2 + tracked_obj.y**2)
                
                # Dynamic FOV check based on distance and bbox size
                # Further objects need more lenient angle matching
                # Base FOV margin + distance-based tolerance + bbox uncertainty
                base_fov_margin = camera_hfov / 2 + 0.3  # Base margin (half FOV + 17°)
                distance_tolerance = min(distance * 0.1, 0.5)  # Up to 0.5 rad (~28°) for distant objects
                total_tolerance = base_fov_margin + distance_tolerance + angle_uncertainty
                
                if angle_diff_abs > total_tolerance:
                    continue  # Outside FOV
                
                # Distance check - much more generous now
                if distance > max_match_distance or distance < 0.1:
                    continue
                
                # Calculate matching score with distance-aware weighting
                # For distant objects, rely more on angle, less on exact distance
                if distance < 3.0:
                    # Close range: angle is very important
                    angle_weight = 2.0
                    distance_weight = 0.5
                elif distance < 6.0:
                    # Medium range: balanced
                    angle_weight = 1.5
                    distance_weight = 0.3
                else:
                    # Long range: angle is most important, distance matters less
                    angle_weight = 1.0
                    distance_weight = 0.1
                
                # Score: lower is better
                # Penalize angle difference more than distance for far objects
                score = angle_diff_abs * angle_weight + (distance * distance_weight)
                
                candidates.append({
                    'id': obj_id,
                    'distance': distance,
                    'angle_diff': angle_diff_abs,
                    'score': score
                })
                
                if score < best_match_score:
                    best_match_score = score
                    best_match_id = obj_id
            
            # Log all candidates for debugging
            if candidates:
                self.get_logger().info(f"    Found {len(candidates)} candidate(s):")
                for c in sorted(candidates, key=lambda x: x['score'])[:3]:  # Show top 3
                    self.get_logger().info(
                        f"      ID:{c['id']}: dist={c['distance']:.1f}m, "
                        f"angle_diff={math.degrees(c['angle_diff']):.1f}°, score={c['score']:.2f}"
                    )
            
            # More generous score threshold based on distance
            if best_match_id is not None:
                best_obj = self.tracked_objects[best_match_id]
                best_distance = math.sqrt(best_obj.x**2 + best_obj.y**2)
                
                # Dynamic threshold: more lenient for distant objects
                if best_distance < 3.0:
                    score_threshold = 1.5  # Strict for close objects
                elif best_distance < 6.0:
                    score_threshold = 2.5  # Medium for mid-range
                else:
                    score_threshold = 4.0  # Very lenient for distant objects
                
                if best_match_score < score_threshold:
                    tracked_obj = self.tracked_objects[best_match_id]
                    tracked_obj.confirm_with_camera(class_name, confidence)
                    self.get_logger().info(
                        f"    ✓ CONFIRMED ID:{best_match_id} as '{class_name}' "
                        f"at {best_distance:.1f}m, score={best_match_score:.2f}/{score_threshold:.1f}"
                    )
                else:
                    self.get_logger().info(
                        f"    ✗ ID:{best_match_id} rejected: score {best_match_score:.2f} > {score_threshold:.1f}"
                    )
            else:
                self.get_logger().info(f"    ✗ No candidates found for '{class_name}'")


    def _cleanup_old_tracks(self):
        """Remove old unconfirmed objects, keep confirmed objects indefinitely (or until timeout)"""
        tracking_timeout = self.get_parameter('tracking_timeout').value
        confirmed_timeout = self.get_parameter('confirmed_timeout').value
        current_time = time.time()
        
        to_remove = []
        
        for obj_id, tracked_obj in self.tracked_objects.items():
            time_since_seen = current_time - tracked_obj.last_seen
            
            # Remove unconfirmed objects after tracking_timeout
            if not tracked_obj.confirmed and tracking_timeout > 0:
                if time_since_seen > tracking_timeout:
                    to_remove.append(obj_id)
                    self.get_logger().debug(
                        f"Removing unconfirmed object ID:{obj_id} (timeout: {time_since_seen:.1f}s)"
                    )
            
            # Remove confirmed objects only if confirmed_timeout > 0
            elif tracked_obj.confirmed and confirmed_timeout > 0:
                if time_since_seen > confirmed_timeout:
                    to_remove.append(obj_id)
                    self.get_logger().info(
                        f"Removing confirmed {tracked_obj.class_name} ID:{obj_id} (timeout: {time_since_seen:.1f}s)"
                    )
        
        # Delete markers for removed objects
        if to_remove:
            self._delete_markers(to_remove)
        
        for obj_id in to_remove:
            del self.tracked_objects[obj_id]
            self.published_marker_ids.discard(obj_id)
            self.published_marker_ids.discard(obj_id + 1000)

    def _delete_markers(self, obj_ids):
        """Delete markers for removed objects"""
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
        """Publish tracked objects with grey markers for unconfirmed, green for confirmed"""
        if not self.tracked_objects:
            self.obstacles_array_pub.publish(String(data=json.dumps([])))
            return
        
        map_frame = self.get_parameter('map_frame').value
        
        # Count confirmed vs unconfirmed
        confirmed_count = sum(1 for obj in self.tracked_objects.values() if obj.confirmed)
        unconfirmed_count = len(self.tracked_objects) - confirmed_count
        
        # Publish text summary
        summary_lines = [
            f"Tracking {len(self.tracked_objects)} objects: "
            f"{confirmed_count} confirmed (green), {unconfirmed_count} unconfirmed (grey)"
        ]
        
        for obj_id, obj in self.tracked_objects.items():
            if obj.confirmed:
                summary_lines.append(
                    f"  ✓ ID:{obj.id} {obj.class_name} @ {obj.distance:.2f}m "
                    f"map:({obj.map_x:.2f}, {obj.map_y:.2f}) conf:{obj.confidence:.2f}"
                )
            else:
                summary_lines.append(
                    f"  ? ID:{obj.id} unconfirmed @ {obj.distance:.2f}m "
                    f"map:({obj.map_x:.2f}, {obj.map_y:.2f}) angle:{math.degrees(obj.angle):.1f}°"
                )
        
        summary = "\n".join(summary_lines)
        self.fused_detections_pub.publish(String(data=summary))
        
        # Publish obstacles array as JSON (all objects)
        obstacles_array = [obj.to_dict() for obj in self.tracked_objects.values()]
        json_string = json.dumps(obstacles_array, indent=2)
        self.obstacles_array_pub.publish(String(data=json_string))
        
        # Publish visualization markers
        marker_array = MarkerArray()
        
        for obj_id, obj in self.tracked_objects.items():
            # Sphere marker
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
            
            # Larger markers for confirmed objects
            if obj.confirmed:
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            else:
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
            
            # Color: Grey for unconfirmed, Bright Green for confirmed
            if obj.confirmed:
                marker.color.r = 0.0
                marker.color.g = 1.0  # Bright green = confirmed
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.6
                marker.color.g = 0.6  # Grey = unconfirmed
                marker.color.b = 0.6
                marker.color.a = 0.7
            
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "object_labels"
            text_marker.id = obj.id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(obj.map_x)
            text_marker.pose.position.y = float(obj.map_y)
            text_marker.pose.position.z = 1.0
            
            text_marker.scale.z = 0.3
            
            if obj.confirmed:
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                text_marker.text = f"{obj.class_name}\n{obj.distance:.1f}m\nID:{obj.id}"
            else:
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                text_marker.text = f"Unconfirmed\n{obj.distance:.1f}m\nID:{obj.id}"
            
            text_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(text_marker)
        
        self.tracked_objects_pub.publish(marker_array)



    def _create_image_msg(self, cv_image):
        """Convert OpenCV image to ROS Image message"""
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