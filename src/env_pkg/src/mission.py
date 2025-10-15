import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
from robot import Robot


class MissionControl:
    def __init__(self, num_robots=1):
        # Create all robot instances
        self.robots = [Robot(f'robot_{i}') for i in range(num_robots)]

        # Create a multithreaded executor to handle multiple ROS nodes
        self.executor = MultiThreadedExecutor()

        # Add each robot and its navigator node to the executor
        for robot in self.robots:
            self.executor.add_node(robot)
            self.executor.add_node(robot.navigator)

    def run_mission(self):
        """Continuously tick robots at 10 Hz while processing ROS callbacks"""
        tick_rate = 1.0  # Hz
        dt = 1.0 / tick_rate

        tick = 0
        try:
            # Spin the executor in a separate thread so callbacks run concurrently
            import threading
            spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            spin_thread.start()

            # Behaviour tree ticking loop
            while rclpy.ok():
                print(f"\n--- TICK {tick} ---")

                robot = self.robots[0]

                if robot.homeXY is None:
                    print("ERROR! Robot home not set yet!")
                    time.sleep(dt)
                    continue

                robot.tick()
                tick += 1
                time.sleep(dt)

        except KeyboardInterrupt:
            print("Mission interrupted")
        finally:
            self.terminate()

    def send_manual_goal(self, x, y, robot_id=0):
        """Send a manual goal to a robot"""
        self.robots[robot_id].move_to_goal(x, y)

    def terminate(self):
        """Cleanly shut down the mission and destroy all nodes"""
        print("Shutting down mission...")

        # Stop the executor and destroy nodes
        self.executor.shutdown()
        for robot in self.robots:
            robot.destroy_node()
            robot.navigator.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()

    # Initialize Mission Control with 1 robot
    mission_controller = MissionControl(num_robots=1)
    mission_controller.run_mission()
