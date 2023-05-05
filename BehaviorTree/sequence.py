from BehaviorTree.root import Status
from BehaviorTree.control import Control, controltick

class Sequence(Control):
    """
    Defines a behavior tree sequence control node
    The tick method will return running or failure if any of the children return the same,
     or success if all children return success
    """
    @controltick
    def tick(self):
        for child in self.children:
            result = child.tick()

            if result == Status.RUNNING or result == Status.FAILURE:
                self.status = result
                break
        else:
            self.status = Status.SUCCESS
            