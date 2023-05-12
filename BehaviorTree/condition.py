from BehaviorTree.leaf import Leaf
from BehaviorTree.root import Status

class Condition(Leaf):
    def tick(self):        
        result = self.function()

        if result:
            return Status.SUCCESS
        
        return Status.FAILURE
    