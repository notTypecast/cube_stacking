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
        for child in self.children:
            if isinstance(child, Control):
                result = child.tick()
            else:
                result = child.tick(self.status == Status.RUNNING)

            if result == Status.RUNNING or result == Status.SUCCESS:
                self.status = result
                break
        else:
            self.status = Status.FAILURE
