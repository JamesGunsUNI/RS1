import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from collections import deque
import math

class NavNode(Node):
    def __init__(self, name='nav_to_pose_client'):
        super().__init__(name)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.isMoving = False
        self.goal_queue = deque()
        self.post_goal_callback = None

    def set_post_goal_callback(self, callback):
        self.post_goal_callback = callback

    def add_goal(self, x, y, yaw=0.0):
        self.get_logger().info(f'Added goal to queue: ({x}, {y}, {yaw})')
        self.goal_queue.append((x, y, yaw))
        if not self.isMoving:
            self.try_next_goal()

    def send_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.isMoving = False
            self.try_next_goal()
            return

        self.get_logger().info('Goal accepted :)')
        self.isMoving = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.isMoving = False

        # Call post-goal callback (e.g., soil sampling)
        if self.post_goal_callback:
            self.post_goal_callback()

        self.try_next_goal()

    def try_next_goal(self):
        if self.goal_queue:
            x, y, yaw = self.goal_queue.popleft()
            self.get_logger().info(f'Sending next goal: x={x}, y={y}, yaw={yaw}')
            self.send_goal(x, y, yaw)
        else:
            self.get_logger().info('No more goals queued')
