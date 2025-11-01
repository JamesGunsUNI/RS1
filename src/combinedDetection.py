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
    def __init__(self, obj_id, class_name, distance, angle, x, y, map_x, map_y, source='unknown'):
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
        self.source = source  # 'lidar', 'depth', or 'fused'
        
    def update(self, distance, angle, x, y, map_x, map_y, confidence=None, source='unknown'):
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
        self.source = source
    
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
            'source': self.source
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
        
        # Subscribe to depth point cloud
        self.depth_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.depth_callback, 10
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
        self.declare_parameter('association_distance_threshold', 2.5)  # meters - distance to merge detections
        self.declare_parameter('map_frame', 'map')  # map frame name
        self.declare_parameter('base_frame', 'base_link')  # robot base frame
        self.declare_parameter('camera_frame', 'camera_link')  # camera frame
        self.declare_parameter('camera_hfov', 1.3962634)  # ~80 degrees in radians
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('use_depth_camera', True)  # Enable/disable depth camera
        self.declare_parameter('min_depth', 0.1)  # Minimum valid depth (meters)
        self.declare_parameter('max_depth', 10.0)  # Maximum valid depth (meters)
        self.declare_parameter('depth_sample_percentage', 0.3)  # Sample 30% of bbox pixels
        self.declare_parameter('lidar_match_pixel_threshold', 100)  # Pixel threshold for LiDAR matching
        
        # Data storage
        self.latest_detections = []  # [(class_name, confidence, bbox)]
        self.latest_lidar_objects = []  # [(distance, angle, x, y)]
        self.latest_depth_cloud = None  # Store latest point cloud
        self.tracked_objects = {}  # {id: TrackedObject}
        self.next_object_id = 0
        self.published_marker_ids = set()  # Track which marker IDs have been published
        
        self.get_logger().info('Combined Detection Node Started with depth camera integration')

    def depth_callback(self, msg):
        """Store the latest depth point cloud for fusion"""
        self.latest_depth_cloud = msg

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
                rclpy.time.Time(),  # Time=0 means "get latest available"
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Convert quaternion to rotation matrix (simplified for 2D case)
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
            self.get_logger().warn(f'Could not transform point to map from {source_frame}: {str(e)}', throttle_duration_sec=5.0)
            return None, None

    def extract_depth_for_bbox(self, bbox_data, point_cloud_msg):
        # Extract median depth from point cloud within bounding box
        if point_cloud_msg is None:
            return None, None, None
        
        try:
            # Get bbox parameters
            center_x = int(bbox_data['bbox_center_x'])
            center_y = int(bbox_data['bbox_center_y'])
            width = int(bbox_data['bbox_width'])
            height = int(bbox_data['bbox_height'])
            
            # Calculate bbox boundaries
            x_min = max(0, int(center_x - width/2))
            x_max = min(point_cloud_msg.width, int(center_x + width/2))
            y_min = max(0, int(center_y - height/2))
            y_max = min(point_cloud_msg.height, int(center_y + height/2))
            
            # Sample points from the bounding box
            sample_rate = self.get_parameter('depth_sample_percentage').value
            min_depth = self.get_parameter('min_depth').value
            max_depth = self.get_parameter('max_depth').value
            
            valid_points = []
            
            # Read point cloud data
            for point in point_cloud2.read_points(
                point_cloud_msg, 
                field_names=("x", "y", "z"),
                skip_nans=True,
                uvs=[(x, y) for x in range(x_min, x_max, max(1, int(1/sample_rate)))
                     for y in range(y_min, y_max, max(1, int(1/sample_rate)))]
            ):
                x, y, z = point
                distance = math.sqrt(x*x + y*y + z*z)
                
                # Filter valid depths
                if min_depth <= distance <= max_depth and not math.isnan(distance):
                    valid_points.append((x, y, z, distance))
            
            if not valid_points:
                return None, None, None
            
            # Use median to avoid outliers
            valid_points.sort(key=lambda p: p[3])
            median_idx = len(valid_points) // 2
            median_point = valid_points[median_idx]
            
            x, y, z, distance = median_point
            
            # Calculate angle in robot frame (camera typically points forward)
            angle = math.atan2(y, x)
            
            return distance, angle, (x, y, z)
            
        except Exception as e:
            self.get_logger().warn(f'Error extracting depth: {str(e)}', throttle_duration_sec=5.0)
            return None, None, None

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
        
        # Perform fusion with depth and LiDAR data
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
        if not self.latest_detections:
            return
        
        use_depth = self.get_parameter('use_depth_camera').value
        camera_frame = self.get_parameter('camera_frame').value
        pixel_threshold = self.get_parameter('lidar_match_pixel_threshold').value
        
        # Track which detections got valid sensor data
        successful_fusions = []
        
        # Track which LiDAR objects have been matched to prevent double-matching
        matched_lidar_indices = set()
        
        # Process each vision detection
        for detection in self.latest_detections:
            class_name = detection['class_name']
            best_source = None
            best_data = None
            
            if class_name.lower() == 'tree':
                # Trees: Use LiDAR only - must have valid LiDAR match
                if self.latest_lidar_objects:
                    camera_hfov = self.get_parameter('camera_hfov').value
                    image_width = self.get_parameter('image_width').value
                    bbox_x = detection['bbox_center_x']
                    
                    best_lidar_match = None
                    best_pixel_diff = float('inf')
                    best_lidar_idx = None
                    
                    # Find best matching LiDAR object
                    for idx, lidar_obj in enumerate(self.latest_lidar_objects):
                        # Skip if already matched
                        if idx in matched_lidar_indices:
                            continue
                        
                        distance, angle, x, y = lidar_obj
                        
                        # Check if LiDAR object is in camera FOV
                        if abs(angle) > camera_hfov / 2:
                            continue
                        
                        # Map angle to pixel position
                        normalized_pos = (angle + camera_hfov/2) / camera_hfov
                        pixel_x = normalized_pos * image_width
                        
                        # Check if close to detection and find closest match
                        pixel_diff = abs(pixel_x - bbox_x)
                        if pixel_diff < pixel_threshold and pixel_diff < best_pixel_diff:
                            best_pixel_diff = pixel_diff
                            best_lidar_match = lidar_obj
                            best_lidar_idx = idx
                    
                    # Use best match if found
                    if best_lidar_match is not None:
                        distance, angle, x, y = best_lidar_match
                        
                        # Transform to map coordinates
                        map_x, map_y = self.transform_point_to_map(x, y)
                        
                        if map_x is not None and map_y is not None:
                            best_source = 'lidar'
                            best_data = (distance, angle, x, y, map_x, map_y)
                            matched_lidar_indices.add(best_lidar_idx)
                            self.get_logger().debug(
                                f"Tree matched with LiDAR at ({map_x:.2f}, {map_y:.2f}), pixel diff: {best_pixel_diff:.1f}",
                                throttle_duration_sec=2.0
                            )
                
                if best_data is None:
                    self.get_logger().debug(
                        f"Tree detected but no LiDAR match found (waiting for LiDAR data)",
                        throttle_duration_sec=2.0
                    )
            else:
                # Everything else (rocks, etc.): Use depth camera - must have valid depth
                if use_depth and self.latest_depth_cloud is not None:
                    distance, angle, xyz = self.extract_depth_for_bbox(detection, self.latest_depth_cloud)
                    
                    if distance is not None and xyz is not None:
                        x, y, z = xyz
                        # Transform from camera frame to map frame
                        map_x, map_y = self.transform_point_to_map(x, y, z, camera_frame)
                        
                        if map_x is not None and map_y is not None:
                            best_source = 'depth'
                            best_data = (distance, angle, x, y, map_x, map_y)
                            self.get_logger().debug(
                                f"{class_name} matched with depth at ({map_x:.2f}, {map_y:.2f})",
                                throttle_duration_sec=2.0
                            )
                    else:
                        self.get_logger().debug(
                            f"{class_name} detected but no valid depth data",
                            throttle_duration_sec=2.0
                        )
                else:
                    self.get_logger().debug(
                        f"{class_name} detected but depth camera disabled or no data",
                        throttle_duration_sec=2.0
                    )
            
            # Only update tracked object if we have valid data from the correct sensor
            if best_data is not None:
                distance, angle, x, y, map_x, map_y = best_data
                self._update_tracked_object(
                    detection['class_name'],
                    distance, angle, x, y, map_x, map_y,
                    detection['confidence'],
                    best_source
                )
                successful_fusions.append(class_name)
        
        # Log fusion summary
        if self.latest_detections:
            self.get_logger().debug(
                f"Fusion: {len(successful_fusions)}/{len(self.latest_detections)} detections got valid position data",
                throttle_duration_sec=2.0
            )
        
        # Clean up old tracked objects
        self._cleanup_old_tracks()
        
        # Publish tracking results (only for objects with valid positions)
        self._publish_tracked_objects()

    def _update_tracked_object(self, class_name, distance, angle, x, y, map_x, map_y, confidence, source='unknown'):
        # Convert to Python floats to avoid numpy type issues
        distance = float(distance)
        angle = float(angle)
        x = float(x)
        y = float(y)
        map_x = float(map_x)
        map_y = float(map_y)
        confidence = float(confidence)
        
        threshold = self.get_parameter('association_distance_threshold').value
        
        # For trees, use a larger threshold since they're static and position estimates can vary
        if class_name.lower() == 'tree':
            threshold = max(threshold, 2.5)  # At least 2.5m for trees
        
        best_match_id = None
        best_match_dist = float('inf')
        
        # Find closest existing object of the same class
        for obj_id, tracked_obj in self.tracked_objects.items():
            if tracked_obj.class_name == class_name:
                dx = map_x - tracked_obj.map_x
                dy = map_y - tracked_obj.map_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist < best_match_dist:
                    best_match_dist = dist
                    best_match_id = obj_id
        
        # Only create new track if no match within threshold OR no existing tracks of this class
        if best_match_id is not None and best_match_dist < threshold:
            # Update existing track
            self.tracked_objects[best_match_id].update(distance, angle, x, y, map_x, map_y, confidence, source)
            self.get_logger().debug(
                f"Updated existing {class_name} track ID:{best_match_id} (dist: {best_match_dist:.2f}m)",
                throttle_duration_sec=2.0
            )
        else:
            # Check if we should really create a new track
            if best_match_id is not None:
                # If closest match is relatively close but outside threshold, log a warning
                if best_match_dist < threshold * 2:
                    self.get_logger().warn(
                        f"Creating new {class_name} despite nearby track at {best_match_dist:.2f}m "
                        f"(threshold: {threshold:.2f}m). Consider increasing association_distance_threshold.",
                        throttle_duration_sec=5.0
                    )
                else:
                    self.get_logger().info(
                        f"Creating new {class_name} track - nearest existing is {best_match_dist:.2f}m away"
                    )
            
            new_id = self.next_object_id
            self.next_object_id += 1
            self.tracked_objects[new_id] = TrackedObject(
                new_id, class_name, distance, angle, x, y, map_x, map_y, source
            )
            self.tracked_objects[new_id].confidence = confidence
            self.get_logger().info(
                f"Created new {class_name} track ID:{new_id} at ({map_x:.2f}, {map_y:.2f}) using {source}"
            )

    def _cleanup_old_tracks(self):
        timeout = self.get_parameter('tracking_timeout').value
        current_time = time.time()
        
        to_remove = []
        
        # Remove tracks that haven't been seen recently (only if timeout > 0)
        if timeout > 0:
            for obj_id, tracked_obj in self.tracked_objects.items():
                if current_time - tracked_obj.last_seen > timeout:
                    to_remove.append(obj_id)
        
        # Merge duplicate tracks that are too close together
        self._merge_duplicate_tracks()
        
        # Delete markers for removed objects
        if to_remove:
            self._delete_markers(to_remove)
        
        for obj_id in to_remove:
            del self.tracked_objects[obj_id]
            self.published_marker_ids.discard(obj_id)
            self.published_marker_ids.discard(obj_id + 1000)  # text marker
    
    def _merge_duplicate_tracks(self):
        """Merge tracks that are suspiciously close to each other"""
        merge_threshold = 1.0  # Merge tracks within 1 meter
        
        to_remove = []
        
        obj_list = list(self.tracked_objects.items())
        
        for i in range(len(obj_list)):
            obj_id_i, obj_i = obj_list[i]
            
            if obj_id_i in to_remove:
                continue
            
            for j in range(i + 1, len(obj_list)):
                obj_id_j, obj_j = obj_list[j]
                
                if obj_id_j in to_remove:
                    continue
                
                # Only merge objects of the same class
                if obj_i.class_name != obj_j.class_name:
                    continue
                
                # Calculate distance between tracks
                dx = obj_i.map_x - obj_j.map_x
                dy = obj_i.map_y - obj_j.map_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                # If tracks are very close, merge them
                if dist < merge_threshold:
                    # Keep the one with more detections, merge into it
                    if obj_i.detection_count >= obj_j.detection_count:
                        # Merge j into i (weighted average)
                        weight = 0.5
                        obj_i.map_x = obj_i.map_x * (1 - weight) + obj_j.map_x * weight
                        obj_i.map_y = obj_i.map_y * (1 - weight) + obj_j.map_y * weight
                        obj_i.detection_count += obj_j.detection_count
                        to_remove.append(obj_id_j)
                        self.get_logger().info(
                            f"Merged duplicate {obj_i.class_name} tracks: ID:{obj_id_j} -> ID:{obj_id_i} (dist: {dist:.2f}m)"
                        )
                    else:
                        # Merge i into j
                        weight = 0.5
                        obj_j.map_x = obj_j.map_x * (1 - weight) + obj_i.map_x * weight
                        obj_j.map_y = obj_j.map_y * (1 - weight) + obj_i.map_y * weight
                        obj_j.detection_count += obj_i.detection_count
                        to_remove.append(obj_id_i)
                        self.get_logger().info(
                            f"Merged duplicate {obj_j.class_name} tracks: ID:{obj_id_i} -> ID:{obj_id_j} (dist: {dist:.2f}m)"
                        )
                        break  # obj_i is removed, move to next i
        
        # Remove merged tracks
        if to_remove:
            self._delete_markers(to_remove)
            for obj_id in to_remove:
                if obj_id in self.tracked_objects:
                    del self.tracked_objects[obj_id]
                self.published_marker_ids.discard(obj_id)
                self.published_marker_ids.discard(obj_id + 1000)

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
            # Publish empty array when no objects
            self.obstacles_array_pub.publish(String(data=json.dumps([])))
            return
        
        map_frame = self.get_parameter('map_frame').value
        
        # Publish text summary (existing behavior)
        summary_lines = [f"Tracking {len(self.tracked_objects)} objects:"]
        for obj_id, obj in self.tracked_objects.items():
            summary_lines.append(
                f"  ID:{obj.id} {obj.class_name} @ {obj.distance:.2f}m "
                f"map:({obj.map_x:.2f}, {obj.map_y:.2f}) conf:{obj.confidence:.2f} src:{obj.source}"
            )
        
        summary = "\n".join(summary_lines)
        self.fused_detections_pub.publish(String(data=summary))
        self.get_logger().info(summary)
        
        # Publish obstacles array as JSON
        obstacles_array = [obj.to_dict() for obj in self.tracked_objects.values()]
        json_string = json.dumps(obstacles_array)
        self.obstacles_array_pub.publish(String(data=json_string))
        
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
            
            # Color based on source
            if obj.source == 'depth':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0  # Blue for depth camera
            elif obj.source == 'lidar':
                marker.color.r = 0.0
                marker.color.g = 1.0  # Green for LiDAR
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0  # Red for unknown
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
            
            text_marker.text = f"{obj.class_name}\n{obj.distance:.1f}m\n({obj.source})"
            
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