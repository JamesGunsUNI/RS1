import rclpy
from rclpy.node import Node
from robot import Robot
import time

class MissionControl:
    def __init__(self, num_robots=1):
        self.robots = [Robot(f'robot_{i}') for i in range(num_robots)]

    def run_mission(self, goals):
        """Continuously tick robots at 10 Hz while processing ROS callbacks"""
        tick_rate = 10.0  # Hz
        dt = 1.0 / tick_rate

        tick = 0
        try:
            while rclpy.ok():
                # Process ROS callbacks for all robot nodes
                rclpy.spin_once(self.robots[0], timeout_sec=0)

                # Tick the behaviour tree
                print(f"\n--- TICK {tick} ---")
                self.robots[0].tick()
                tick += 1

                time.sleep(dt)
        except KeyboardInterrupt:
            print("Mission interrupted")

    def send_manual_goal(self, x, y, robot_id=0):
        self.robots[robot_id].move_to_goal(x, y)

    def terminate(self):
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

    mission_controller.terminate()
import rclpy
from rclpy.node import Node
from robot import Robot
import time

class MissionControl:
    def __init__(self, num_robots=1):
        self.robots = [Robot(f'robot_{i}') for i in range(num_robots)]

    def run_mission(self, goals):
        """Continuously tick robots at 10 Hz while processing ROS callbacks"""
        tick_rate = 10.0  # Hz
        dt = 1.0 / tick_rate

        tick = 0
        try:
            while rclpy.ok():
                # Process ROS callbacks for all robot nodes
                rclpy.spin_once(self.robots[0], timeout_sec=0)

                # Tick the behaviour tree
                print(f"\n--- TICK {tick} ---")
                self.robots[0].tick()
                tick += 1

                time.sleep(dt)
        except KeyboardInterrupt:
            print("Mission interrupted")

    def send_manual_goal(self, x, y, robot_id=0):
        self.robots[robot_id].move_to_goal(x, y)

    def terminate(self):
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

    mission_controller.terminate()