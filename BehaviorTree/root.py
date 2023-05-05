from BehaviorTree.InvalidTree import InvalidTreeError
from enum import Enum

class Status(Enum):
    RUNNING = 1
    FAILURE = 2
    SUCCESS = 3

class Root:
    """
    Defines a behavior tree root node
    The node contains one child
    """
    def __init__(self, child=None):
        self.child = child

    def tick(self):
        if self.child is None:
            raise InvalidTreeError("Root node has no child")
        
        return self.child.tick()
        
