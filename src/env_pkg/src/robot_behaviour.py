from behaviourTree import NodeTypes, Status
import math

class RobotBrain():
    def __init__(self, robot):
        self.robot = robot
        ################
        ### BT Nodes ###
        ################

        #TERMINATE
        self.terminate = NodeTypes.Fallback([
            self.power_off,
            NodeTypes.Action("Return Home", self.return_home)
        ])
        #TERMINAL
        self.terminal = NodeTypes.Sequence([
            NodeTypes.Condition("Mission Complete?", self.mission_done),
            self.terminate
        ])

        #POWER OFF
        self.power_off = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Home?", self.is_robot_home),
            NodeTypes.Action("Leave Charging Station", self.power_off)
        ])

        #INITIALISE
        self.initialise = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Docked?", self.is_robot_docked),
            NodeTypes.Condition("Is Battery Charged?", self.is_battery_charged),
            NodeTypes.Action("Leave Charging Station", self.leave_charging_station)
        ])

        #CHARGE
        self.charge = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Docked?", self.is_robot_docked),
            NodeTypes.Action("Charge Robot", self.charge_robot)
        ])

        #RECHARGE
        self.recharge = NodeTypes.Fallback([
            self.charge,
            NodeTypes.Action("Return Home", self.return_home)
        ])

        #BATTERY CHECK
        self.battery_check = NodeTypes.Sequence([
                NodeTypes.Condition("Battery Critical?", self.is_battery_critical),
                self.recharge
        ])

        #SAMPLE COLLECTION
        self.sample_collection = NodeTypes.Sequence([
            NodeTypes.Condition("Is Robot Near Unvisited Tree?", self.is_near_unvisted_tree),
            NodeTypes.Action("Collect Sample", self.collect_sample)
        ])

        #GOAL SELECTION
        self.goal_selection = NodeTypes.Sequence([
            NodeTypes.Condition("Does The Robot Have A Goal?", self.has_goal),
            NodeTypes.Action("Select Next Sample Location", self.select_next_sample_location),
        ])
        #GOAL SELECTION
        self.navigate = NodeTypes.Sequence([
            NodeTypes.Condition("Does The Robot Have A Goal?", self.has_goal),
            NodeTypes.Action("Navigate To Goal", self.navigate_to_goal),
        ])

        #NAVIGATION
        self.navigation = NodeTypes.Fallback([
            self.navigate,
            NodeTypes.Action("Select Next Sample Location", self.select_next_sample_location),
        ])

        # FULL TREE
        self.root = NodeTypes.Fallback([
            self.terminal,
            self.initialise,
            self.battery_check,
            self.sample_collection,
            self.navigation
        ])

    ###########################
    ### Condition Functions ###
    ###########################
    def mission_done(self):
        if self.robot.samples_collected >= self.robot.max_samples:
            self.robot.mission_complete = True
        else:
            self.robot.mission_complete = False
        return self.robot.mission_complete
    
    def is_robot_docked(self):
        return self.robot.is_docked

    def is_robot_home(self):
        curr_pos = self.robot.current_pose
        home_pos = self.robot.homeXY
        dist_2_home = math.sqrt((curr_pos[0] - home_pos[0])**2 + (curr_pos[1] - home_pos[1])**2)
        return dist_2_home < 1.1

    def is_battery_charged(self):
        print(f"Battery Check - power: {self.robot.battery_power} is charged {self.robot.battery_power >= 90}")
        return self.robot.battery_power >= 90

    def is_battery_critical(self):
        print(f"Battery Critical - power: {self.robot.battery_power} is charged {self.robot.battery_power >= 90}") 
        return self.robot.battery_power < 20

    def is_near_unvisted_tree(self):
        if self.robot.current_goal == None:
            return False
        return self.robot.has_reached_goal()
    
    def has_goal(self):
        return self.robot.current_goal != None

    ########################
    ### Action Functions ###
    ########################
    def power_off(self):
        print("Powering off robot...")
        return Status.SUCCESS
    
    def return_home(self):
        print("Returning home...")
        if self.robot.current_goal != self.robot.homeXY:
            self.robot.move_to_goal(self.robot.homeXY)

        return Status.SUCCESS if self.robot.has_reached_goal() else Status.RUNNING
    
    def leave_charging_station(self):
        print("Leaving charging station...")

        if self.robot.current_goal != self.robot.dedocking_goal:
             self.robot.move_to_goal(self.robot.dedocking_goal)

        if self.robot.has_reached_goal():
            self.robot.is_docked = False
            return Status.SUCCESS
        
        return Status.RUNNING
        
    def collect_sample(self):
        print("Taking sample...")
        self.robot.samples_collected += 1
        #add tree to visted list
        return Status.SUCCESS
    
    def select_next_sample_location(self):
        print("Selecting Next Goal")
        self.robot.sample_target_goal = self.robot.test_goals[self.robot.test_goal_id]
        self.robot.move_to_goal(self.robot.sample_target_goal)
        self.robot.test_goal_id += 1
        return Status.SUCCESS

    def navigate_to_goal(self):
        print("Navigating to sample site...")
        return Status.SUCCESS if self.robot.has_reached_goal() else Status.RUNNING




