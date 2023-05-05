import numpy as np
import RobotDART as rd
import BehaviorTree as bt
from dartpy.math import Isometry3

from utils import create_grid, create_problems, damped_pseudoinverse, create_transformation_matrix, isom3_to_np, calc_error, isclose

# matrix to rotate 90 degrees in z axis
rotate_pi_z = np.array(((0, -1, 0), (1, 0, 0), (0, 0, 1)))

# rotation matrix for regular hold of end effector
R_REG_HOLD = np.array((
    (-1, 1, 0),
    (1, -1, 0),
    (0, 0, -1)
))

# regular hold, rotated by 90 degrees
R_ROT_HOLD = R_REG_HOLD @ rotate_pi_z

ERROR_THRESHOLD = 0.01 # error under which to stop controller
STACK_POSITION = (0.4, 0.45) # (x, y) of stack
STACK_POSITION_MATRIX = create_transformation_matrix(R_REG_HOLD, np.array((*STACK_POSITION, 0.15))) # desired transformation matrix with respect to wf of end effector when above stack location
BOX_END_POS_MATRICES_I3 = [] # matrices of corresponding desired positions before leaving each box, respectively

for i in range(3):
    new_matrix = np.copy(STACK_POSITION_MATRIX)
    new_matrix[2, 3] += i*0.04
    BOX_END_POS_MATRICES_I3.append(Isometry3(new_matrix))


dt = 0.001 # you are NOT allowed to change this
simulation_time = 100.0 # you are allowed to change this
total_steps = int(simulation_time / dt)

# end effector
eef_link_name = "panda_hand"

#########################################################
# DO NOT CHANGE ANYTHING IN HERE
# Create robot
robot = rd.Franka(int(1. / dt))
init_position = [0., np.pi / 4., 0., -np.pi / 4., 0., np.pi / 2., 0., 0.04, 0.04]
robot.set_positions(init_position)

max_force = 5.
robot.set_force_lower_limits([-max_force, -max_force], ["panda_finger_joint1", "panda_finger_joint2"])
robot.set_force_upper_limits([max_force, max_force], ["panda_finger_joint1", "panda_finger_joint2"])
#########################################################
robot.set_actuator_types("servo") # you can use torque here

#########################################################
# DO NOT CHANGE ANYTHING IN HERE
# Create boxes
box_positions = create_grid()

box_size = [0.04, 0.04, 0.04]

# Red Box
# Random cube position
red_box_pt = np.random.choice(len(box_positions))

#box_pose = [0., 0., 0., 0.35, 0.3, box_size[2] / 2.0]
box_pose = [0., 0., 0., box_positions[red_box_pt][0], box_positions[red_box_pt][1], box_size[2] / 2.0]
red_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.9, 0.1, 0.1, 1.0], "red_box")

# Green Box
# Random cube position
green_box_pt = np.random.choice(len(box_positions))
while green_box_pt == red_box_pt:
    green_box_pt = np.random.choice(len(box_positions))

#box_pose = [0., 0., 0., 0.35, 0.25, box_size[2] / 2.0]
box_pose = [0., 0., 0., box_positions[green_box_pt][0], box_positions[green_box_pt][1], box_size[2] / 2.0]
green_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.9, 0.1, 1.0], "green_box")

# Blue Box
# Random cube position
blue_box_pt = np.random.choice(len(box_positions))
while blue_box_pt == green_box_pt or blue_box_pt == red_box_pt:
    box_pt = np.random.choice(len(box_positions))

#box_pose = [0., 0., 0., 0.3, 0.25, box_size[2] / 2.0]
box_pose = [0., 0., 0., box_positions[blue_box_pt][0], box_positions[blue_box_pt][1], box_size[2] / 2.0]
blue_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.1, 0.9, 1.0], "blue_box")
#########################################################

#########################################################
# PROBLEM DEFINITION
# Choose problem
problems = create_problems()
problem_id = np.random.choice(len(problems))
problem = problems[problem_id]

print('We want to put the', problem[2], 'cube on top of the', problem[1], 'and the', problem[1], 'cube on top of the', problem[0], 'cube.')
#########################################################

#########################################################
# Create Graphics
gconfig = rd.gui.Graphics.default_configuration()
gconfig.width = 1280 # you can change the graphics resolution
gconfig.height = 960 # you can change the graphics resolution
graphics = rd.gui.Graphics(gconfig)

# Create simulator object
simu = rd.RobotDARTSimu(dt)
simu.set_collision_detector("fcl") # you can use bullet here
simu.set_control_freq(100)
simu.set_graphics(graphics)
graphics.look_at((0., 2.5, 1.5), (0., 0., 0.25))
simu.add_checkerboard_floor()
simu.add_robot(robot)
simu.add_robot(red_box)
simu.add_robot(blue_box)
simu.add_robot(green_box)
#########################################################


class RobotState:
    """
    Class holding state variables for robot
    """
    # variable indicating if in the middle of moving to grip position motion
    above = False
    # variable indicating the movement state when moving to end position
    # can be any of:
    # 0: currently moving up
    # 1: currently moving to above end position
    # 2: currently moving down to end position
    move_state = None
    # target position can also be saved here
    target = None
    # variable indicating gripper state
    gripping = False
    # variable indicating if currently executing movement to some position
    moving = False
    # flag to indicate prep move is done
    prep_move = None

class PITaskController:
    """
    Task controller for moving the end-effector to a given location
    Handles translation and rotation separately
    """
    def __init__(self, target, dt, Kp=2, Ki=0.005):
        self.target = target
        self.dt = dt
        self.Kp = Kp
        self.Ki = Ki
        self.error_sum = 0
        self.last_error = None

    def set_target(self, target):
        self.error_sum = 0
        self.target = target

    def get_error(self, tf):
        return calc_error(self.target, tf)
    
    def update(self, current):
        error_wf = self.get_error(current)
        self.error_sum += error_wf * self.dt

        self.last_error = np.linalg.norm(error_wf)
        
        return self.Kp * error_wf + self.Ki * self.error_sum
    
# instantiate controller
controller = PITaskController(None, dt)
    
def moveToPosition():
    """
    Low-level controller
    Moves robot so end-effector reaches specific position
    Target position is defined by target in controller (global PITaskController instance)
    """
    tf = robot.body_pose(eef_link_name)
    new_v = controller.update(tf)

    if controller.last_error > ERROR_THRESHOLD:
        jacobian_wf = robot.jacobian(eef_link_name)
        jac_pinv = damped_pseudoinverse(jacobian_wf)
        commands = jac_pinv @ new_v

        # keep gripper speed as-is
        commands[7] = -0.05 if RobotState.gripping else 0

        robot.set_commands(commands)
        
        return None
    
    return True

def moveToGripPosition():
    """
    Similar to moveToPosition, but consists of two motions:
    1) Moving above a position
    2) Moving down to position
    This is necessary to avoid knocking box away with gripper
    """
    # call low-level controller to move to target position
    res = moveToPosition()

    if res:
        # if we just moved above position, set new position to move down to position
        if not RobotState.above:
            desired_total = isom3_to_np(controller.target)
            desired_total[2, 3] -= 0.3
            tf_desired = Isometry3(desired_total)
            controller.set_target(tf_desired)
            RobotState.above = True
            return None
        # else, controller finished moving down to position
        else:
            RobotState.above = False
            return res
        
    return None

def moveToEndPosition():
    """
    Similar to moveToPosition, but consists of three motions:
    1) Moving up from current position
    2) Moving above end position
    3) Moving down to end position
    """
    # if this controller just ran for the first time
    if RobotState.move_state is None:
        RobotState.move_state = 0

        RobotState.target = controller.target

        current_pos = robot.body_pose(eef_link_name)
        current_pos = isom3_to_np(current_pos)
        current_pos[2, 3] += 0.3
        tf = Isometry3(current_pos)
        controller.set_target(tf)
    
    res = moveToPosition()

    if res:
        # if we just moved above initial position
        if RobotState.move_state == 0:
            RobotState.move_state = 1
            new_pos = isom3_to_np(RobotState.target)
            new_pos[2, 3] += 0.3
            tf = Isometry3(new_pos)
            controller.set_target(tf)
        # if we just moved above end position
        elif RobotState.move_state == 1:
            RobotState.move_state = 2
            controller.set_target(RobotState.target)
        # if we just moved to end position
        else:
            RobotState.move_state = None
            return True
        
    return None

    

def closeGripper():
    """
    Low-level controller
    Closes end-effector's gripper
    """
    robot.set_commands([0, 0, 0, 0, 0, 0, 0, -0.1, 0])
    
    if robot.positions()[7] < 0.0201:
        RobotState.gripping = True
        return True
    
    return None

def openGripper():
    """
    Low-level controller
    Opens end-effector's gripper
    """
    robot.set_commands([0, 0, 0, 0, 0, 0, 0, 0.05, 0])
    if robot.positions()[7] > 0.039:
        robot.set_commands([0, 0, 0, 0, 0, 0, 0, 0, 0])
        RobotState.gripping = False
        RobotState.moving = False
        RobotState.above = False
        
        return True
    
    return None

def createBehaviorTree():
    """
    Creates behavior tree that solves given problem
    """
    box_map = {"red": red_box, "green": green_box, "blue": blue_box}
    # determine what hold to use for each box (normal or rotated)
    box_holds = {}
    # index of box that needs to be moved first
    # necessary for when first box is fully surrounded, in which case another box must be moved first
    prep_move_box_idx = None

    box_colors = list(box_map.keys())

    for i in range(3):
        for j in range(i, 3):
            pos1 = box_map[problem[i]].positions()[3:5]
            pos2 = box_map[problem[j]].positions()[3:5]
            # if they are right next to each other (same y, x off by .05)
            if isclose(pos1[1] - pos2[1], 0) and isclose(np.abs(pos1[0] - pos2[0]), 0.05):
                # assign rotated hold to first one to be picked
                box_holds[problem[i]] = R_ROT_HOLD
                box_holds[problem[j]] = R_REG_HOLD

                # if this is the first one to be picked, ensure it is not fully surrounded
                if i == 0:
                    k = (set(range(3)) - {i, j}).pop()
                    pos3 = box_map[problem[k]].positions()[3:5]
                    # check if third box is above or below
                    if isclose(pos1[0] - pos3[0], 0) and isclose(np.abs(pos1[1] - pos3[1]), 0.05):
                        # if surrounded, assign regular hold to above or below and stop
                        RobotState.prep_move = False
                        prep_move_box_idx = k
                        box_holds[problem[prep_move_box_idx]] = R_REG_HOLD
                        break

        if prep_move_box_idx is not None:
            break

    for box in box_map:
        if box not in box_holds:
            box_holds[box] = R_REG_HOLD
                

    def checkBoxPosition(box_color, pos):
        """
        Checks if a box's position is the given one
        """
        current = box_map[box_color].positions()
        #return current[3] == pos[0] and current[4] == pos[1]
        #print(np.abs(current[3] - pos[0]), np.abs(current[4] - pos[1]))
        #print(np.abs(current[3] - pos[0]) < 0.15 and np.abs(current[4] - pos[1]) < 0.15)
        #print("Returning", np.abs(current[3] - pos[0]) < 0.15 and np.abs(current[4] - pos[1]) < 0.15)
        return np.abs(current[3] - pos[0]) < 0.05 and np.abs(current[4] - pos[1]) < 0.05
    
    def checkEEfPositionAgainstBox(box_color, current):
        """
        Checks if end effector is close to box
        """
        box_pos = box_map[box_color].positions()[3:]

        target = Isometry3(create_transformation_matrix(box_holds[box_color], np.array((box_pos[0], box_pos[1], box_pos[2] + 0.1))))

        res = np.linalg.norm(calc_error(target, current)) < ERROR_THRESHOLD

        if res:
            RobotState.moving = False
            RobotState.above = False
        return res
    
    def checkEEfPosition(target, current):
        """
        Checks if end effector is close to given position
        """
        return np.linalg.norm(calc_error(target, current)) < ERROR_THRESHOLD

    def moveToBox(box_color):
        if not RobotState.moving:
            desired_translation = box_map[box_color].body_pose(0).translation()
            desired_total = create_transformation_matrix(box_holds[box_color], np.array((desired_translation[0], desired_translation[1], desired_translation[2] + 0.4)))
            tf_desired = Isometry3(desired_total)
            controller.set_target(tf_desired)

            RobotState.moving = True
            RobotState.above = False
    
        res = moveToGripPosition()

        if res:
            robot.set_commands([0, 0, 0, 0, 0, 0, 0, 0, 0])
            RobotState.moving = False
            RobotState.above = False

        return res

    def setAndMoveToPos(tf):
        if not RobotState.moving:
            controller.set_target(tf)
            RobotState.moving = True
            RobotState.above = False

        res = moveToEndPosition()

        if res == bt.Status.SUCCESS:
            RobotState.moving = False
            RobotState.above = False

        return res


    ### Main sequence node
    main_seq = bt.Sequence()
    ###

    def create_box_fb(box_idx, prep_move=False):
        ## First fallback node
        ## responsible for placing first box in correct position
        main_fb_0 = bt.Fallback()

        if not prep_move:
            low_pos_correct = bt.Condition(lambda: checkBoxPosition(problem[box_idx], STACK_POSITION))
        else:
            # if prep move, condition is whether it has been completed
            low_pos_correct = bt.Condition(lambda: RobotState.prep_move)
            

        # Sequence of decisions and low level controllers for moving box
        move_low_to_correct_pos = bt.Sequence()

        # Branch 1: moving to position and grabbing
        move_to_grab_pos_fb = bt.Fallback()
        gripping_cond = bt.Condition(lambda: RobotState.gripping)
        move_grab_seq = bt.Sequence()
        move_to_grab_pos_fb.add_children([gripping_cond, move_grab_seq])
        
        current_pos_fb = bt.Fallback()
        close_to_pos_cond = bt.Condition(lambda: checkEEfPositionAgainstBox(problem[box_idx], robot.body_pose(eef_link_name)))
        move_to_pos_action = bt.Action(lambda: moveToBox(problem[box_idx]))
        current_pos_fb.add_children([close_to_pos_cond, move_to_pos_action])

        grab_action = bt.Action(closeGripper)

        move_grab_seq.add_children([current_pos_fb, grab_action])

        # Branch 2: moving to stack position
        stack_pos_fb = bt.Fallback()
        if not prep_move:
            in_stack_pos_cond = bt.Condition(lambda: checkEEfPosition(BOX_END_POS_MATRICES_I3[box_idx], robot.body_pose(eef_link_name)))
            move_to_stack_action = bt.Action(lambda: setAndMoveToPos(BOX_END_POS_MATRICES_I3[box_idx]))
        else:
            box_pos = box_map[problem[box_idx]].positions()[3:5]
            new_pos_matrix = Isometry3(create_transformation_matrix(R_REG_HOLD, np.array((box_pos[0], box_pos[1]-0.15, 0.15))))
            in_stack_pos_cond = bt.Condition(lambda: checkEEfPosition(new_pos_matrix, robot.body_pose(eef_link_name)))
            move_to_stack_action = bt.Action(lambda: setAndMoveToPos(new_pos_matrix))
        stack_pos_fb.add_children([in_stack_pos_cond, move_to_stack_action])

        # Branch 3: release gripper
        if not prep_move:
            release_gripper_action = bt.Action(openGripper)
        else:
            # if prep move, also mark flag when done
            def finish_prep_move():
                res = openGripper()
                if res:
                    RobotState.prep_move = True

                return res
            release_gripper_action = bt.Action(finish_prep_move)

        move_low_to_correct_pos.add_children([move_to_grab_pos_fb, stack_pos_fb, release_gripper_action])

        main_fb_0.add_children([low_pos_correct, move_low_to_correct_pos])

        return main_fb_0
    
    if prep_move_box_idx is not None:
        seq = create_box_fb(prep_move_box_idx, prep_move=True)
        main_seq.add_child(seq)

    for i, _ in enumerate(problem):
        fb = create_box_fb(i)
        main_seq.add_child(fb)

    root = bt.Root(main_seq)
    return root


root = createBehaviorTree()

for step in range(total_steps):
    if (simu.schedule(simu.control_freq())):
        root.tick()

    if (simu.step_world()):
        break
