from abc import ABC, abstractmethod
from BehaviorTree.InvalidTree import InvalidTreeError
from BehaviorTree.root import Status

class Control(ABC):
    """
    Abstract class defining a control node
    Can have an abstract number of children (>0)
    Subclasses must define tick method
    """

    def __init__(self, children=[], memory=True):
        """
        Initializes a control node
        Arguments:
         children: list of children of control node
         memory(=True): determines whether to continue from running child, or reset on next tick
        """
        self.children = []
        self.children.extend(children)
        self.status = None
        self.memory = memory
        self.child_index = 0

    @abstractmethod
    def tick(self):
        pass

    def add_child(self, child):
        self.children.append(child)

    def add_children(self, children):
        self.children.extend(children)

    def reset_subtree(self):
        for child in self.children:
            child.reset()
        self.reset()

    def reset(self):
        self.child_index = 0
        self.status = None

def controltick(tick_func):
    """
    Decorator for tick methods of control nodes
    """
    def wrapper(*args, **kwargs):
        self = args[0]
        
        if not self.children:
            raise InvalidTreeError("Control node has no children")
        
        if self.status is not None and self.status != Status.RUNNING:
            return self.status
        
        tick_func(*args, **kwargs)

        status = self.status

        if self.status == Status.SUCCESS:
            self.reset_subtree()

        return status

    return wrapper
