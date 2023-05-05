from BehaviorTree.leaf import Leaf
from BehaviorTree.root import Status

class Condition(Leaf):
    def tick(self, running_action=False):
        if running_action:
            return False
        
        result = self.function()

        if result:
            return Status.SUCCESS
        
        return Status.FAILURE
    