import random
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
            self.fn = fn

        def tick(self):
            result = self.fn()
            print(f"[Action {self.name}] -> {result}")
            return result

    class Condition(BaseNode):
        def __init__(self, name, fn):
            self.name = name
            self.fn = fn

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
                elif result == Status.RUNNING:
                    return Status.RUNNING
            return Status.FAILURE

