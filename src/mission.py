import rclpy
from rclpy.node import Node
from navigation import NavNode

'''
cd ros_ws/

colcon build --packages-select 41068_ignition_bringup --symlink-install

ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo

ros2 run teleop_twist_keyboard teleop_twist_keyboard

cd ros_ws/rs1/src/

python mission.py
'''
goals = [
    (4.0, 5.0),
    (3.0, 4.0),
    (2.0, 1.0)
]


class MissionControl:

    def __init__(self):
        self.navigator = NavNode()

    def run_mission(self):
        for goal in goals:
            print(f"moving to new goal: {goal}")
            self.navigator.send_goal(float(goal[0]), float(goal[1]))

    def terminate(self):
        self.navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    mission_controller = MissionControl()
    mission_controller.run_mission()
    rclpy.spin(mission_controller.navigator)
    mission_controller.terminate()

