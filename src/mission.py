import rclpy
from rclpy.node import Node
from robot import Robot

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

NUM_ROBOTS = 1

class MissionControl:

    def __init__(self, robots):
        self.robots = robots
        self.robots = []

        for _ in NUM_ROBOTS:
            self.robots.append(Robot())


    def run_mission(self):
        for goal in goals:
            print(f"moving to new goal: {goal}")
            self.robots[0].move_to_goal(goal[0], goal[1])

    def updateSoilHeatMap(self, soil_sample_data):
        '''
        fucntion that updates UI heat map with latest data
        returns nothing
        
        '''
        raise NotImplementedError("INTERFACE UI HEAT MAP UPDATE HERE")
    
    def send_manual_goal(self, goal_x, goal_y, robot_id):
        '''
        fucntion for UI to use that gets, x,y and int represent robot number to send a goal to the robot

        returns nothing
        
        '''
        self.robots[robot_id].move_to_goal(goal_x, goal_y)

    def terminate(self):
        for _ in NUM_ROBOTS:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    mission_controller = MissionControl()
    mission_controller.run_mission()
    #rclpy.spin(mission_controller.navigator)
    mission_controller.terminate()

