#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class ObjectCounter(Node):
    def __init__(self):
        super().__init__('object_counter')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        
        # Parameters for object detection
        self.min_distance = 0.1  # Minimum distance to consider (meters)
        self.max_distance = 10.0  # Maximum distance to consider (meters)
        self.min_points_per_object = 1  # Minimum points to consider as an object
        self.max_gap_distance = 0.3  # Maximum gap between points in same object (meters)
        self.get_logger().info('Object Counter Node Started')

    def laser_callback(self, msg):
        try:
            # Filter valid ranges
            ranges = np.array(msg.ranges)
            valid_ranges = ranges[(ranges >= self.min_distance) & 
                                 (ranges <= self.max_distance) & 
                                 (ranges != float('inf')) & 
                                 (~np.isnan(ranges))]
            
            if len(valid_ranges) == 0:
                print("No objects detected")
                return
            
            # Get angles for valid points
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            # Create list of valid points with their angles and distances
            valid_points = []
            for i, range_val in enumerate(ranges):
                if (range_val >= self.min_distance and 
                    range_val <= self.max_distance and 
                    range_val != float('inf') and 
                    not math.isnan(range_val)):
                    angle = angle_min + i * angle_increment
                    
                    # Convert to Cartesian coordinates
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    valid_points.append((x, y, angle, range_val))
            
            # Count objects and get their distances
            objects_with_distances = self.count_objects_with_distances(valid_points)
            
            # Print results to terminal
            if objects_with_distances:
                for i, distance in enumerate(objects_with_distances, 1):
                    print(f"Object {i} is {distance:.2f}m away")
            else:
                print("No objects detected")
                
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {str(e)}')
            print("No objects detected due to error")

    def count_objects_with_distances(self, points):
        #Group nearby points and return list of object distances
        if not points:
            return []
        
        # Sort points by angle 
        points.sort(key=lambda p: p[2]) 
        
        objects = []
        current_object = [points[0]]
        
        for i in range(1, len(points)):
            current_point = points[i]
            last_point = current_object[-1]
            
            # Calculate distance between consecutive points
            dx = current_point[0] - last_point[0]
            dy = current_point[1] - last_point[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If points are close enough, add to current object
            if distance <= self.max_gap_distance:
                current_object.append(current_point)
            else:
                # Start new object if current object has enough points
                if len(current_object) >= self.min_points_per_object:
                    objects.append(current_object)
                current_object = [current_point]
        
        # Last object
        if len(current_object) >= self.min_points_per_object:
            objects.append(current_object)
        
        # Calculate average distance for each object
        object_distances = []
        for obj in objects:
            # Get all distances for points in this object
            distances = [point[3] for point in obj]  # point[3] is the range value
            avg_distance = sum(distances) / len(distances)
            object_distances.append(avg_distance)
        
        return object_distances

def main(args=None):
    rclpy.init(args=args)
    object_counter = ObjectCounter()
    rclpy.spin(object_counter)
    print("\nShutting down Object Counter Node...")
    rclpy.shutdown()

if __name__ == '__main__':
    main()