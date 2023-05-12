from BehaviorTree.root import Status
from BehaviorTree.control import Control, controltick

class Fallback(Control):
    """
    Defines a behavior tree fallback control node
    The tick method will return running or true if any of the children return the same,
     or failure if all children return failure
    """
    @controltick
    def tick(self):
        for i in range(self.child_index if self.memory else 0, len(self.children)):
            result = self.children[i].tick()

            if self.memory and result == Status.RUNNING:
                self.child_index = i

            if result == Status.RUNNING or result == Status.SUCCESS:
                self.status = result
                break
        else:
            self.status = Status.FAILURE
