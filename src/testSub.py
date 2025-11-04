#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String


class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        
        self.detected_objects = []
        
        self.obstacles_sub = self.create_subscription(
            String,
            '/obstacles_array',
            self.obstacles_callback,
            10
        )
        
        self.get_logger().info('Detection Subscriber Node Started')
        self.get_logger().info('Storing coordinates and printing detections...')
    
    def obstacles_callback(self, msg):
        try:
            obstacles = json.loads(msg.data)
            
            # Clear the array and update with latest data
            self.detected_objects = []
            
            # Store each object with its coordinates
            for obj in obstacles:
                object_data = {
                    'id': obj.get('id', 0),
                    'class_name': obj.get('class_name', 'unknown'),
                    'map_x': obj.get('map_x', 0.0),
                    'map_y': obj.get('map_y', 0.0)
                }
                self.detected_objects.append(object_data)
            
            # Print
            if self.detected_objects:
                for obj in self.detected_objects:
                    self.get_logger().info(
                        f"ID: {obj['id']}, Object: {obj['class_name']}, X: {obj['map_x']:.2f}, Y: {obj['map_y']:.2f}"
                    )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse obstacles JSON: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DetectionSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down Detection Subscriber Node...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()