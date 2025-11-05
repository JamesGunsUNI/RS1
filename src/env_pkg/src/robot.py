#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from navigation import NavNode
from robot_behaviour import RobotBrain
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
import math
import yaml
import os
import json
from enum import Enum
from datetime import datetime


class RobotStatus(Enum):
    IDEL = 1
    RUNNING = 2


class Robot(Node):
    def __init__(self, name='robot'):
        super().__init__(name)

        #Core modules
        self.navigator = NavNode(name=f'{name}_navigator')
        self.decision = RobotBrain(self)

        #Robot state
        self.homeXY = None
        self.battery_multiplier = 5
        self.battery_power = 100
        self.samples_collected = 0
        self.max_samples = 5
        self.mission_complete = False
        self.is_docked = True
        self.dedocking_goal = None
        self.robot_status = RobotStatus.IDEL
        self.distance_threshold = 1.5
        self.current_goal = None
        self.inital_start_pose = None
        self.current_pose = [0, 0]
        self.sample_target_goal = None
        self.latest_soil_sample = None
        self.nearest_tree = None
        self.current_tree_id = None

        #Sample/Tree tracking
        self.visited_trees_list = []  # array of [x, y] poses
        self.visited_tree_ids = set()  # for JSON object IDs
        self.tree_yaml_file = "tree_distance_graph.yaml"

        #Test goals
        self.roam_goal_id = 0
        self.roam_goals = [
            (3.1, 3.3),
            (7.25, 4.2),
            (8.3, 8.0),
            (4.9, 3.8),
            (0.5, 8.8),
            (-4.2, 5),
            (-3.3, 2.4),
            (2.9, -2.2),
            (-2.6, -5.75),
            (-6.4, -7.7),
            (-6.7, 3.5)
        ]

        #ROS Subscriptions
        self.navigator.set_post_goal_callback(self.take_soil_sample)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        self.soil_sub = self.create_subscription(Float32, '/soil_moisture', self.soil_callback, 10)
        self.obstacle_sub = self.create_subscription(String, '/obstacles_array', self.obstacles_callback, 10)

        self.get_logger().info(f"{name} node started with obstacle tracking.")

    ##############################
    # ---  Behaviour tick loop  ---
    ##############################
    def tick(self):
        self.decision.root.tick()

    ##############################
    # ---  Sensor Callbacks     ---
    ##############################
    def soil_callback(self, msg: Float32):
        self.latest_soil_sample = msg

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        self.current_pose = [position.x, position.y]
        if self.homeXY is None:
            self.homeXY = self.current_pose
            self.dedocking_goal = [self.homeXY[0] + 2.4, self.homeXY[1]]
        orientation = msg.pose.pose.orientation

    def obstacles_callback(self, msg: String):

        self.get_logger().debug("Received /obstacles_array message.")

        try:
            obstacles = json.loads(msg.data)
            self.get_logger().debug(f"Parsed JSON successfully: {obstacles}")
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in /obstacles_array message.")
            return

        trees = [o for o in obstacles if o.get("class_name", "").lower() == "tree"]
        self.get_logger().debug(f"Filtered trees: {trees}")

        if not trees:
            self.get_logger().info("No trees found in obstacle data.")
            return

        nearest_tree = None
        nearest_dist = float("inf")
        nearest_tree_id = None

        for tree in trees:
            tree_id = tree.get("id")
            tree_x = tree.get("map_x", 0.0)
            tree_y = tree.get("map_y", 0.0)

            if tree_id in self.visited_tree_ids:
                self.get_logger().debug(f"Tree ID {tree_id} already visited, skipping.")
                continue

            dx = tree_x - self.current_pose[0]
            dy = tree_y - self.current_pose[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)

            self.get_logger().debug(
                f"Tree ID {tree_id} at ({tree_x}, {tree_y}), "
                f"distance to robot: {distance:.2f} m"
            )

            if distance < nearest_dist:
                nearest_dist = distance
                nearest_tree = [tree_x, tree_y]
                nearest_tree_id = tree_id

            tree_x_r = round(tree_x, 1)
            tree_y_r = round(tree_y, 1)
            distance_r = round(distance, 1)
            robot_x_r = round(self.current_pose[0], 1)
            robot_y_r = round(self.current_pose[1], 1)

            self.save_tree_distance(tree_id, tree_x_r, tree_y_r, robot_x_r, robot_y_r, distance_r)

        if nearest_tree is not None:
            self.nearest_tree = nearest_tree
            self.current_tree_id = nearest_tree_id
            self.get_logger().debug(
                f"Nearest unvisited tree updated: ID {self.current_tree_id} at {self.nearest_tree}"
            )
        else:
            self.nearest_tree = None
            self.current_tree_id = None
            self.get_logger().info("No unvisited trees nearby.")

    ##############################
    # ---  Utility Functions    ---
    ##############################
    def save_tree_distance(self, tree_id, tree_x, tree_y, robot_x, robot_y, distance, yaml_path="tree_distance_graph.yaml"):
        """Append to YAML file for distance transform graph, avoiding duplicates for the same robot position."""
        
        entry = {
            "timestamp": datetime.now().isoformat(),
            "tree_id": tree_id,
            "tree_position": {"x": tree_x, "y": tree_y},
            "robot_position": {"x": robot_x, "y": robot_y},
            "distance": distance
        }

        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                try:
                    data = yaml.safe_load(f)
                    if data is None:
                        data = []
                except yaml.YAMLError:
                    data = []
        else:
            data = []

        exists = any(
            d.get("robot_position", {}).get("x") == robot_x and
            d.get("robot_position", {}).get("y") == robot_y
            for d in data
        )
        
        if not exists:
            data.append(entry)
            with open(yaml_path, 'w') as f:
                yaml.safe_dump(data, f)
            self.get_logger().info(f"Saved tree distance at robot ({robot_x}, {robot_y})")
        else:
            self.get_logger().info(f"Skipped duplicate entry at robot ({robot_x}, {robot_y})")

    ##############################
    # ---  Mission Functions   ---
    ##############################
    def move_to_goal(self, goal):
        self.current_goal = goal
        self.robot_status = RobotStatus.RUNNING
        self.inital_start_pose = self.current_pose
        self.navigator.add_goal(float(goal[0]), float(goal[1]), yaw=0.0)
        self.visited_tree_ids.add(self.current_tree_id)

    def take_soil_sample(self):
        """Simulate taking a soil sample after reaching a goal."""
        self.navigator.get_logger().info("Taking soil sample...")

        # Extract float from ROS Float32 message
        if self.latest_soil_sample is None:
            self.navigator.get_logger().warn("No soil sample available!")
            return

        moisture = float(self.latest_soil_sample.data)  # <-- extract numeric value

        self.navigator.get_logger().info(f"Soil moisture: {moisture:.3f}")

        filename = "heatmap_data.yaml"
        dataset_name = "dataset1"

        # Load or create YAML structure
        if os.path.exists(filename):
            with open(filename, "r") as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        if dataset_name not in data:
            data[dataset_name] = []

        x = float(self.current_pose[0])
        y = float(self.current_pose[1])
        data[dataset_name].append({"x": x, "y": y, "value": moisture})

        with open(filename, "w") as f:
            yaml.dump(data, f)

        self.navigator.get_logger().info(
            f"Saved soil sample at ({x:.2f}, {y:.2f}) -> {moisture:.3f}"
        )


    def decrease_battery(self, amount):
        self.battery_power -= amount

    def charge_robot(self):
        self.battery_power = 100

    def has_reached_goal(self):
        dist_2_goal = math.sqrt(
            (self.current_goal[0] - self.current_pose[0]) ** 2 +
            (self.current_goal[1] - self.current_pose[1]) ** 2
        )

        if dist_2_goal < self.distance_threshold:
            print("[Robot] Goal Reached!")
            self.robot_status = RobotStatus.IDEL

            # Reduce battery
            total_distance = math.sqrt(
                (self.inital_start_pose[0] - self.current_pose[0]) ** 2 +
                (self.inital_start_pose[1] - self.current_pose[1]) ** 2
            )
            power_usage = total_distance * self.battery_multiplier
            self.decrease_battery(power_usage)
            print(f"Battery Power reduced by {power_usage}%")

            # Mark tree as visited **only now**
            if self.current_tree_id is not None:
                self.visited_tree_ids.add(self.current_tree_id)
                self.visited_trees_list.append(self.current_pose.copy())
                print(f"Tree ID {self.current_tree_id} marked as visited.")
                self.current_tree_id = None

            self.current_goal = None
            self.sample_target_goal = None
            return True
        return False
