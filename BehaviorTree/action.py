from BehaviorTree.leaf import Leaf
from BehaviorTree.root import Status

class Action(Leaf):
    def __init__(self, function):
        super().__init__(function)
        self.status = None

    def reset(self):
        self.status = None

    def tick(self):
        if self.status is not None and self.status != Status.RUNNING:
            return self.status

        result = self.function()

        if result is None:
            self.status = Status.RUNNING
        elif result:
            self.status = Status.SUCCESS  
        else:
            self.status = Status.FAILURE
        
        return self.status
