#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
import time

# IMPORTANT: make sure Robot can be imported when installed.
# If robot.py lives in env_pkg/env_pkg/robot.py, use this:
# If you currently have `robot.py` somewhere else, see Step 2.
from robot import Robot
class MissionControl:
    def __init__(self, num_robots=1):
        self.robots = [Robot(f'robot_{i}') for i in range(num_robots)]
        self.executor = MultiThreadedExecutor()
        for robot in self.robots:
            self.executor.add_node(robot)
            self.executor.add_node(robot.navigator)

    def run_mission(self):
        tick_rate = 1.0
        dt = 1.0 / tick_rate
        tick = 0
        try:
            import threading
            spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            spin_thread.start()
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
        self.robots[robot_id].move_to_goal(x, y)

    def terminate(self):
        print("Shutting down mission...")
        self.executor.shutdown()
        for robot in self.robots:
            robot.destroy_node()
            robot.navigator.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    mission_controller = MissionControl(num_robots=1)
    mission_controller.run_mission()

if __name__ == '__main__':
    main()
