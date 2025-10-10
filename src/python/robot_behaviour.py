from behaviourTree import NodeTypes, Status
import math

class RobotBrain():
    def __init__(self, robot):
        self.robot = robot
        ################
        ### BT Nodes ###
        ################
        #TERMINAL
        self.terminal = NodeTypes.Sequence([
            NodeTypes.Condition("Mission Complete?", self.mission_done),
            self.terminate
        ])

        #TERMINATE
        self.terminate = NodeTypes.Fallback([
            self.power_off,
            NodeTypes.Action("Return Home", self.return_home)
        ])

        #POWER OFF
        self.power_off = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Home", self.is_robot_home)
        ])

        #INITIALISE
        self.initialise = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Home", self.is_robot_home)
            NodeTypes.Condition("Is Battery Charged?", self.is_battery_charged),
            NodeTypes.Action("Leave Charging Station", self.leave_charging_station)
        ])

        #BATTERY CHECK
        self.battery_check = NodeTypes.Fallback([
            NodeTypes.Sequence([
                NodeTypes.Condition("Battery Critical?", self.is_battery_critical),
                NodeTypes.Action("Return Home", self.return_home)
            ]),
        ])

        #SAMPLE COLLECTION
        self.sample_collection = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Near Unvisited Tree?", self.sample_valid),
            NodeTypes.Action("Collect Sample", self.collect_sample)
        ])

        #NAVIGATION
        self.navigation = NodeTypes.Sequence([
            self.goal_selection,
            NodeTypes.Action("Navigate To Goal", self.navigate_to_sample_site),
        ])

        #GOAL SELECTION
        self.goal_selection = NodeTypes.Sequence([
            NodeTypes.Condition("Does The Robot Have A Goal?", self.has_goal),
            NodeTypes.Action("Select Next Sample Location", self.select_next_sample_location),
        ])

        # CHECK FOR MORE DATA
        self.check_more_data = NodeTypes.Fallback([
            NodeTypes.Sequence([
                NodeTypes.Condition("All Data Collected?", self.all_data_collected),
                NodeTypes.Action("Return Home", self.return_home)
            ]),
            NodeTypes.Action("Continue Mission", lambda: Status.SUCCESS)
        ])


        # MISSION LOOP
        self.mission_loop = NodeTypes.Fallback([
            self.battery_check,
            self.sample_collection,
            self.check_more_data,
            self.error_check
        ])

        # FULL TREE
        self.root = NodeTypes.Sequence([
            self.initialise,
            self.mission_loop,
            self.terminate
        ])

    ###########################
    ### Condition Functions ###
    ###########################
    def is_battery_charged(self):
        print(f"Battery Check - power: {self.robot.battery_power} is charged {self.robot.battery_power >= 90}")
        return self.robot.battery_power >= 90

    def is_battery_critical(self):
        print(f"Battery Critical - power: {self.robot.battery_power} is charged {self.robot.battery_power >= 90}")
        return self.robot.battery_power < 20

    def all_data_collected(self):
        return samples_collected >= self.robot.max_samples

    def valid_sample_location(self):
        return True

    def sample_valid(self):
        return True

    def mission_done(self):
        return self.robot.mission_complete

    ########################
    ### Action Functions ###
    ########################
    def leave_charging_station(self):
        print("Leaving charging station...")
        #if hasnt left chaging station then leave
        goal = [2.4, 0]
        distance_2_goal = math.sqrt((goal[0] - self.robot.current_pose[0])**2 + (goal[1] - self.robot.current_pose[1])**2)
        if distance_2_goal <= 1.1 and goal == self.robot.current_goal:
            return Status.SUCCESS
        elif goal == self.robot.current_goal:
            return Status.RUNNING
        else:
            self.robot.move_to_goal(goal[0], goal[1])
            return Status.RUNNING
        

    def return_home():
        print("Returning home...")
        return Status.SUCCESS

    def go_charge():
        print("Going to charging station...")
        return Status.SUCCESS

    def select_next_sample_location():
        print("Selecting next sample location...")
        return Status.SUCCESS

    def plan_path_to_sample_site():
        print("Planning path to sample site...")
        return Status.SUCCESS

    def navigate_to_sample_site():
        print("Navigating to sample site...")
        return Status.SUCCESS if random.random() > 0.1 else Status.FAILURE

    def take_sample():
        global samples_collected
        print("Taking sample...")
        samples_collected += 1
        return Status.SUCCESS

    def record_sample():
        print("Recording sample data...")
        return Status.SUCCESS

    def record_loop_error():
        print("Recording loop error!")
        return Status.SUCCESS

    def power_off():
        print("Powering off robot...")
        return Status.SUCCESS