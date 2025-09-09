import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from collections import deque
from robot import Robot
import math

'''
cd ros_ws/

colcon build --packages-select 41068_ignition_bringup --symlink-install

ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo

ros2 run teleop_twist_keyboard teleop_twist_keyboard

cd ros_ws/rs1/src/

python mission.py
'''

class MissionControl:
    def __init__(self, num_robots=1):
        self.robots = [Robot(f'robot_{i}') for i in range(num_robots)]

    def run_mission(self, goals):
        """Queue multiple goals to the first robot (example)"""
        for goal in goals:
            x, y = goal
            self.robots[0].move_to_goal(x, y)

    def send_manual_goal(self, x, y, robot_id=0):
        """Send a goal to a specific robot"""
        self.robots[robot_id].move_to_goal(x, y)

    def terminate(self):
        # Cleanup if needed
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()

    # Define your goals
    goals = [
        (4.0, 5.0),
        (-3.0, -4.0),
        (2.0, 1.0)
    ]

    # Initialize Mission Control with 1 robot
    mission_controller = MissionControl(num_robots=1)
    mission_controller.run_mission(goals)

    # Spin the robot's navigator to process actions
    rclpy.spin(mission_controller.robots[0].navigator)

    mission_controller.terminate()
