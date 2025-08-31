from enum import Enum

class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BaseNode:
    def tick(self):
        raise NotImplementedError

class NodeTypes:
    class Action(BaseNode):
        def __init__(self, name, fn):
            self.name = name
            self.fn = fn  #Function that returns SUCCESS/FAILURE/RUNNING

        def tick(self):
            result = self.fn()
            print(f"[Action {self.name}] -> {result}")
            return result

    class Condition(BaseNode):
        def __init__(self, name, fn):
            self.name = name
            self.fn = fn  #Function returns True/False

        def tick(self):
            result = Status.SUCCESS if self.fn() else Status.FAILURE
            print(f"[Condition {self.name}] -> {result}")
            return result

    class Sequence(BaseNode):
        def __init__(self, children):
            self.children = children

        def tick(self):
            for child in self.children:
                result = child.tick()
                if result != Status.SUCCESS:
                    return result
            return Status.SUCCESS

    class Fallback(BaseNode):
        def __init__(self, children):
            self.children = children

        def tick(self):
            for child in self.children:
                result = child.tick()
                if result == Status.SUCCESS:
                    return Status.SUCCESS
            return Status.FAILURE

################################################################################################
#Basic Example

import random

# Dummy robot state
battery_level = 50

def check_battery():
    return battery_level > 20  # True if OK

def go_charge():
    print("Going to charging station...")
    return Status.SUCCESS

def navigate_goal():
    print("Navigating to goal...")
    return Status.SUCCESS if random.random() > 0.2 else Status.FAILURE

# Build the BT
bt = NodeTypes.Fallback([
    NodeTypes.Sequence([
        NodeTypes.Condition("Battery OK?", check_battery),
        NodeTypes.Action("Navigate", navigate_goal)
    ]),
    NodeTypes.Action("Charge", go_charge)
])

# Run it
for _ in range(3):
    result = bt.tick()
    print("Tree result:", result, "\n")
