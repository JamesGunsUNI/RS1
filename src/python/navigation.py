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

        # Wait for Nav2 action server once at startup
        self.get_logger().info('Waiting for NavigateToPose action server...')
        self._client.wait_for_server()
        self.get_logger().info('Action server ready!')

    def set_post_goal_callback(self, callback):
        """Set a callback to be called after each goal finishes."""
        self.post_goal_callback = callback

    def add_goal(self, x, y, yaw=0.0):
        """Add a navigation goal to the queue."""
        self.get_logger().info(f'Added goal to queue: ({x}, {y}, {yaw})')
        self.goal_queue.append((x, y, yaw))

        # If not currently moving, immediately try the first goal
        if not self.isMoving:
            self.try_next_goal()

    def send_goal(self, x, y, yaw=0.0):
        """Send a goal to Nav2 NavigateToPose action server."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation (yaw -> quaternion)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')

        # ✅ Set moving flag immediately to prevent other goals firing
        self.isMoving = True  

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called once the action server responds to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            self.isMoving = False
            self.try_next_goal()
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called when the action server reports the goal result."""
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}, result: {result}')
        self.isMoving = False

        # Optional: perform a task after reaching each goal
        if self.post_goal_callback:
            self.post_goal_callback()

        # Now move on to the next queued goal
        self.try_next_goal()

    def try_next_goal(self):
        """Send the next goal in the queue if available."""
        if self.goal_queue:
            x, y, yaw = self.goal_queue.popleft()
            self.send_goal(x, y, yaw)
        else:
            self.get_logger().info('No more goals queued')


# def main(args=None):
#     rclpy.init(args=args)
#     node = NavNode()

#     # Example: add multiple goals
#     node.add_goal(1.0, 1.0, 0.0)
#     node.add_goal(2.0, 2.0, 0.0)
#     node.add_goal(-2.0, -2.0, 0.0)

#     # Example: set a callback
#     node.set_post_goal_callback(lambda: node.get_logger().info("Taking soil sample..."))

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()