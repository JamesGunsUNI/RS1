import rclpy
from rclpy.node import Node
from navigation import NavNode
from robot_behaviour import RobotBrain
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class Robot(Node):
    def __init__(self, name='robot'):
        super().__init__(name)

        self.navigator = NavNode(name=f'{name}_navigator')
        self.homeXY = (0.0, 0.0, 0.0)
        self.battery_power = 100
        self.samples_collected = 0
        self.max_sample = 5
        self.mission_complete = False

        self.current_goal = [0,0]
        self.current_pose = [0,0]

        self.navigator.set_post_goal_callback(self.take_soil_sample)
        self.decision = RobotBrain(self)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)

    def tick(self):
        self.decision.root.tick()

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        self.current_pose = [position.x, position.y]
        orientation = msg.pose.pose.orientation

    ############################
    ### Navigation Functions ###
    ############################
    def move_to_goal(self, x, y, yaw=0.0):
        self.current_goal = [x, y]
        self.navigator.add_goal(float(x), float(y), yaw)

    def take_soil_sample(self):
        """Simulate taking a soil sample after reaching a goal"""
        self.navigator.get_logger().info("Taking soil sample...")
        moisture = 0.42  # placeholder
        self.navigator.get_logger().info(f"Soil moisture: {moisture}")
        return moisture
    
    def move_to_next_tree(self):
        pass

    def move_to_home(self):
        self.navigator.add_goal(*self.homeXY)

    def decrease_battery(self, amount):
        self.battery_power -= amount
        
    def charge_robot(self):
        self.battery_power = 100