from abc import ABC, abstractmethod
from BehaviorTree.root import Status

class Leaf(ABC):
    """
    Abstract class defining a leaf node
    Has a function that runs on tick
    Subclasses must define tick method
    """
    def __init__(self, function):
        self.function = function

    def reset(self):
        """
        By default, resetting a leaf does nothing
        Leafs that keep status should override this to reset it
        """
        pass

    @abstractmethod
    def tick(self, running_action=False):
        pass
