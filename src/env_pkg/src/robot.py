import rclpy
from rclpy.node import Node
from navigation import NavNode
from robot_behaviour import RobotBrain
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math
from enum import Enum

class RobotStatus(Enum):
    IDEL = 1
    RUNNING = 2

class Robot(Node):
    def __init__(self, name='robot'):
        super().__init__(name)

        self.navigator = NavNode(name=f'{name}_navigator')
        self.homeXY = None
        self.battery_multiplier = 2
        self.battery_power = 100
        self.samples_collected = 0
        self.max_samples = 5
        self.mission_complete = False

        self.is_docked = True
        self.dedocking_goal = None

        self.robot_status = RobotStatus.IDEL

        self.distance_threshold = 1.3
        self.current_goal = None
        self.inital_start_pose = None
        self.current_pose = [0,0]
        self.sample_target_goal = None

        self.test_goal_id = 0
        self.test_goals = [
        (4.0, 5.0),
        (-3.0, -4.0),
        (2.0, 1.0)
        ]

        self.visited_trees_list = [] #array of [x,y] poses

        self.navigator.set_post_goal_callback(self.take_soil_sample)
        self.decision = RobotBrain(self)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)

    def tick(self):
        self.decision.root.tick()

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        self.current_pose = [position.x, position.y]
        if self.homeXY == None:
            self.homeXY = self.current_pose
            self.dedocking_goal = self.homeXY
            self.dedocking_goal[0] += 2.4
        orientation = msg.pose.pose.orientation

    ############################
    ### Navigation Functions ###
    ############################
    def move_to_goal(self, goal):
        self.current_goal = goal
        self.robot_status = RobotStatus.RUNNING
        self.inital_start_pose = self.current_pose
        self.navigator.add_goal(float(self.current_goal[0]), float(self.current_goal[1]), yaw = 0.0)

    def take_soil_sample(self):
        """Simulate taking a soil sample after reaching a goal"""
        self.navigator.get_logger().info("Taking soil sample...")
        moisture = 0.42  # placeholder
        self.navigator.get_logger().info(f"Soil moisture: {moisture}")
        return moisture
    
    def decrease_battery(self, amount):
        self.battery_power -= amount
        
    def charge_robot(self):
        self.battery_power = 100

    def has_reached_goal(self):
        dist_2_goal = math.sqrt((self.current_goal[0] - self.current_pose[0])**2 + (self.current_goal[1] - self.current_pose[1])**2)
        print(f"current goal{self.current_goal}, distacner to goal: {dist_2_goal}, current pose: {self.current_pose}")
        if dist_2_goal < self.distance_threshold:
            print("[Robot] Goal Reached!")
            self.robot_status = RobotStatus.IDEL
            total_distance = math.sqrt((self.inital_start_pose[0] - self.current_pose[0])**2 + (self.inital_start_pose[1] - self.current_pose[1])**2)
            power_usage = total_distance * self.battery_multiplier
            print(f"Battery Power redcued by {power_usage}%")
            self.decrease_battery(power_usage)
            self.current_goal = None
            self.sample_target_goal = None
            return True
        
        return False
