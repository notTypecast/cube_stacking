import argparse
import utils
from utils import R_REG_HOLD, R_ROT_HOLD
import numpy as np
import RobotDART as rd
import BehaviorTree as bt
from dartpy.math import Isometry3
from math import ceil
from time import time

def mtime_lim(arg):
    """
    Argparse type function - for limiting mtime values
    """
    try:
        f = float(arg)
    except ValueError:
        raise argparse.ArgumentTypeError(f"invalid float value: '{arg}'")

    if f < 1 or f > 5:
        raise argparse.ArgumentTypeError(f"invalid value {f}, expected 1 <= mtime <= 5")
    
    return f

parser = argparse.ArgumentParser(description = "\
An implementation allowing the Franka robotic arm to stack three cubes in any order. \
The cubes are placed in a grid, spanning [0.3, 0.7] for x and [-0.4, 0.4] for y. \
Task space control, or trajectory optimization may be used to control the arm's movement.")

parser.add_argument("-c", "--control", choices=["task", "topt"], default="task", help="Use task space control or trajectory optimization.")
parser.add_argument("--mtime", type=mtime_lim, default=2, help="Total time per movement in trajectory optimization.")
parser.add_argument("--cubepos", choices=["random", "line", "triangle"], default="random", help="Use random box positions, or one of two edge cases.")
parser.add_argument("--order", choices=["random", "rgb", "rbg", "grb", "gbr", "brg", "bgr"], default="random", help="Use random box order, or specify some order.")
args = parser.parse_args()

CONTROLLER = args.control.upper()

ERROR_THRESHOLD_LOW = 0.005 # low threshold, precise movement
ERROR_THRESHOLD_HIGH = 0.1 # high threshold, fast movement

if CONTROLLER == "TOPT":
    from gekko import GEKKO
    from scipy.optimize import minimize
    # joint angle limits, as (upper, lower) pairs
    JOINT_LIMITS = [(2.8973, -2.8973), (1.7628, -1.7628), (2.8973, -2.8973), (-0.0698, -3.0718), (2.8973, -2.8973), (3.7525, -0.0175), (2.8973, -2.8973)]
    # joint speed limits, absolute value
    JOINT_SPEED_LIMITS = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]
    # move time is standard for trajectory optimization, so don't need very high threshold
    ERROR_THRESHOLD_HIGH = 2*ERROR_THRESHOLD_LOW

STACK_POSITION = (0.4, 0.5) # (x, y) of stack
STACK_POSITION_MATRIX = utils.create_transformation_matrix(R_REG_HOLD, np.array((*STACK_POSITION, 0.15))) # desired transformation matrix with respect to wf of end effector when above stack location
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
box_positions = utils.create_grid()

box_size = [0.04, 0.04, 0.04]

if args.cubepos == "random":
    red_box_pt = np.random.choice(len(box_positions))
    red_box_pose = [0., 0., 0., box_positions[red_box_pt][0], box_positions[red_box_pt][1], box_size[2] / 2.0]
    green_box_pt = np.random.choice(len(box_positions))
    while green_box_pt == red_box_pt:
        green_box_pt = np.random.choice(len(box_positions))
    green_box_pose = [0., 0., 0., box_positions[green_box_pt][0], box_positions[green_box_pt][1], box_size[2] / 2.0]
    blue_box_pt = np.random.choice(len(box_positions))
    while blue_box_pt == green_box_pt or blue_box_pt == red_box_pt:
        box_pt = np.random.choice(len(box_positions))
    blue_box_pose = [0., 0., 0., box_positions[blue_box_pt][0], box_positions[blue_box_pt][1], box_size[2] / 2.0]
elif args.cubepos == "line":
    red_box_pose = [0., 0., 0., 0.25, 0.25, box_size[2] / 2.0]
    green_box_pose = [0., 0., 0., 0.35, 0.25, box_size[2] / 2.0]
    blue_box_pose = [0., 0., 0., 0.3, 0.25, box_size[2] / 2.0]
else:
    red_box_pose = [0., 0., 0., 0.35, 0.3, box_size[2] / 2.0]
    green_box_pose = [0., 0., 0., 0.35, 0.25, box_size[2] / 2.0]
    blue_box_pose = [0., 0., 0., 0.3, 0.25, box_size[2] / 2.0]

 
#box_pose = [0., 0., np.pi/4, box_positions[red_box_pt][0], box_positions[red_box_pt][1], box_size[2] / 2.0] # EDGE CASE: rotated
#box_pose = [np.pi/2, 5*np.pi/4, np.pi/4, box_positions[green_box_pt][0], box_positions[green_box_pt][1], 4*box_size[2] / 2.0] # EDGE CASE: rotated
#box_pose = [0., np.pi/2, np.pi/4., box_positions[blue_box_pt][0], box_positions[blue_box_pt][1], 4*box_size[2] / 2.0] # EDGE CASE: rotated

red_box = rd.Robot.create_box(box_size, red_box_pose, "free", 0.1, [0.9, 0.1, 0.1, 1.0], "red_box")
green_box = rd.Robot.create_box(box_size, green_box_pose, "free", 0.1, [0.1, 0.9, 0.1, 1.0], "green_box")
blue_box = rd.Robot.create_box(box_size, blue_box_pose, "free", 0.1, [0.1, 0.1, 0.9, 1.0], "blue_box")

#########################################################

#########################################################
# PROBLEM DEFINITION
# Choose problem
if args.order == "random":
    problems = utils.create_problems()
    problem_id = np.random.choice(len(problems))
    problem = problems[problem_id]
else:
    problem = []
    cmap = {"r": "red", "g": "green", "b": "blue"}
    for c in args.order:
        problem.append(cmap[c])

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
    # error threshold
    error_threshold = ERROR_THRESHOLD_HIGH
    # total time to complete movement (only relevant for trajectory optimization)
    move_time = args.mtime
    # time started gripping, used to avoid getting stuck when gripping if threshold can't be reached
    grip_start_time = None

class PITaskController:
    """
    Task space controller for moving the end-effector to a given location
    Handles translation and rotation separately
    """
    def __init__(self, target, dt, Kp=2, Ki=0.005):
        self.target = target
        self.dt = dt
        self.Kp = Kp
        self.Ki = Ki
        self.error_sum = 0

    def set_target(self, target):
        self.error_sum = 0
        self.target = target

    def get_error(self, tf):
        return utils.calc_error(self.target, tf)
    
    def update(self):
        current = robot.body_pose(eef_link_name)
        # calculate error in world frame
        error_wf = self.get_error(current)
        # add error*dt to total sum (to simulate integral calculation for each step)
        self.error_sum += error_wf * self.dt

        # PI Controller: Kp*Xe + Ki*Int
        new_v = self.Kp * error_wf + self.Ki * self.error_sum

        # if error is large enough, continue control
        if np.linalg.norm(error_wf) > RobotState.error_threshold:
            jacobian_wf = robot.jacobian(eef_link_name)
            jac_pinv = utils.damped_pseudoinverse(jacobian_wf)
            commands = jac_pinv @ new_v
                
            return commands
        
        return None
    
class TrajectoryOptController:
    """
    Trajectory optimization controller for moving the end-effector to a given location
    """
    def __init__(self, target, dt):
        """
        Initializes the controller with some position, and the simulation timestep
        dt is multiplied by 10 to match simulation frequency
        """
        if target is not None:
            self.set_target(target)
        else:
            self.target = None
        self.dt = dt*10
        self.solution = None

    def set_target(self, tf):
        """
        Sets target transformation matrix and corresponding joint positions
        """
        self.target = tf
        self.target_pos = TrajectoryOptController.ik_opt(robot.positions(), tf)

    def update(self):
        """
        Returns next commands for control to given target
        """
        if self.solution is None:
            self.solution = self.calculate(RobotState.move_time)
            if not self.solution:
                return False
        try:
            return next(self.solution)
        except StopIteration:
            self.solution = None
            return None

    def calculate(self, t_final=1):
        """
        Calculates the commands required to get to target
        t_final: total time to complete movement (in seconds)
        """
        # create model
        model = GEKKO(remote=False)

        # time discretization constant
        N = 40
        # split total time into equal intervals
        model.time = np.linspace(0, t_final, N)

        # state is joint positions
        state = model.Array(model.Var, 7)

        # control is joint speed
        control = model.Array(model.MV, 7)

        # set initial position for state, and limits for state and speed
        pos = robot.positions()
        for i in range(7):
            state[i].value = pos[i]
            state[i].lower = JOINT_LIMITS[i][1]
            state[i].upper = JOINT_LIMITS[i][0]
            control[i].STATUS = 1
            control[i].lower = -JOINT_SPEED_LIMITS[i]
            control[i].upper = JOINT_SPEED_LIMITS[i]

        # system dynamics
        for i in range(7):
            # dx_i/dt = u_i
            model.Equation(state[i].dt() == control[i])

        # boundary constraint (final position should be target position)
        for i in range(7):
            model.fix(state[i], pos=len(model.time)-1, val=self.target_pos[i])

        # objective function to minimize (sum of squared joint speeds)
        model.Obj(sum([control[i]**2 for i in range(7)]))

        # non-linear solver
        model.options.IMODE = 6
        try:
            model.solve(disp=False)
        except:
            return False

        #print(model.options.SOLVETIME)

        # need to return Tf/dt total commands, to span Tf simulation seconds
        for i in range(int(t_final/self.dt)):
            # we have a total of N commands, which need to be spread over a range of Tf/dt
            # as such, we return each command a total of D = Tf/(dt*N) times
            # therefore, we use the index ceil(i/D), so each i is used D times
            index = ceil(i*N*self.dt/t_final)
            if index < len(control[0]):
                # we only solved for the 7 DOFs, append 0s for the last two joints
                commands = [c[index] for c in control]
                commands.extend([0, 0])
                yield commands
            else:
                yield [0 for _ in range(9)]
    
    # black-box optimization
    @staticmethod
    def ik_opt(pos, tf_desired, min_error = 1e-12, r_const=0.001):
        """
        Calculates robot positions (joint angles) from given transformation matrix using black-box optimization
        """
        def eval(x):
            robot.set_positions(x)
            tf = robot.body_pose(eef_link_name)
            # we do not need to convert to world frame since we are not using the Jacobian
            error_in_body_frame = rd.math.logMap(tf.inverse().multiply(tf_desired))

            ferror = np.linalg.norm(error_in_body_frame)
            fjoint = np.linalg.norm(x)
            # term r*fjoint keeps joint angles close to 0
            return ferror + r_const*fjoint

        # Optimize using any optimizer
        res = minimize(eval, pos, method='SLSQP', tol=min_error)

        # set initial positions again
        robot.set_positions(pos)

        return res.x
    
# instantiate controller
if CONTROLLER == "TOPT":
    controller = TrajectoryOptController(None, dt)
else:
    controller = PITaskController(None, dt)
    
#########################################################
## LOW LEVEL CONTROLLERS

def moveToPosition():
    """
    Low-level controller
    Moves robot so end-effector reaches specific position
    """
    while True:
        commands = controller.update()
        if commands is not False:
            break
        # TODO: Somehow reset position here
        print("Failed to find solution")
        exit(0)

    if commands is None:
        return True

    # if gripping, keep pressure on object
    if RobotState.gripping:
        pos = robot.positions()[7]
        # apply command only if not at right position
        if pos < 0.02:
            commands[7] = 0
        else:
            commands[7] = -0.1
    # if not gripping and gripper not open, open it
    elif robot.positions()[7] < 0.039:
        commands[7] = 0.1

    robot.set_commands(commands)

    return None

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
            # we need more precise movement before gripping, so reduce error threshold
            # we don't need this for trajectory optimization
            if CONTROLLER == "TASK":
                RobotState.error_threshold = ERROR_THRESHOLD_LOW
            desired_total = utils.isom3_to_np(controller.target)
            # subtract from z direction
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

        # First, robot will move up
        # therefore, save existing target for later
        RobotState.target = controller.target

        # set target as current + 0.3 in z direction
        current_pos = robot.body_pose(eef_link_name)
        current_pos = utils.isom3_to_np(current_pos)
        current_pos[2, 3] += 0.3
        tf = Isometry3(current_pos)
        controller.set_target(tf)
    
    # Continue control
    res = moveToPosition()

    if res:
        # if we just moved above initial position
        if RobotState.move_state == 0:
            RobotState.move_state = 1
            # Set new target as 0.2 above final target
            new_pos = utils.isom3_to_np(RobotState.target)
            new_pos[2, 3] += 0.2
            tf = Isometry3(new_pos)
            controller.set_target(tf)
        # if we just moved above end position
        elif RobotState.move_state == 1:
            # we need more precise movement when releasing, so reduce error threshold
            if CONTROLLER == "TASK":
                RobotState.error_threshold = ERROR_THRESHOLD_LOW
            RobotState.move_state = 2
            # Set new target as final target
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

    if RobotState.grip_start_time is None:
        RobotState.grip_start_time = time()

    if robot.positions()[7] < 0.0201 or time() - RobotState.grip_start_time > 1:
        RobotState.grip_start_time = None
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

#########################################################

#########################################################
# Function creating behavior tree with desired behavior
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

    # loop through boxes in order that they must be picked up in
    for i in range(3):
        for j in range(i, 3):
            pos1 = box_map[problem[i]].positions()[3:5]
            pos2 = box_map[problem[j]].positions()[3:5]
            # if they are right next to each other (same y, x off by .05)
            if utils.isclose(pos1[1] - pos2[1], 0) and utils.isclose(np.abs(pos1[0] - pos2[0]), 0.05):
                # assign rotated hold to first one to be picked
                box_holds[problem[i]] = R_ROT_HOLD
                box_holds[problem[j]] = R_REG_HOLD

                # if this is the first one to be picked, ensure it is not fully surrounded
                if i == 0:
                    k = (set(range(3)) - {i, j}).pop()
                    pos3 = box_map[problem[k]].positions()[3:5]
                    # check if third box is above or below
                    if utils.isclose(pos1[0] - pos3[0], 0) and utils.isclose(np.abs(pos1[1] - pos3[1]), 0.05):
                        # if surrounded, assign regular hold to above or below and stop
                        RobotState.prep_move = False
                        prep_move_box_idx = k
                        box_holds[problem[prep_move_box_idx]] = R_REG_HOLD
                        break

        if prep_move_box_idx is not None:
            break

    # assign regular hold to any box that might not have one yet
    for box in box_map:
        if box not in box_holds:
            box_holds[box] = R_REG_HOLD                

    def checkBoxPosition(box_color, pos):
        """
        Checks if a box's position is the given one
        """
        current = box_map[box_color].positions()
        res = np.abs(current[3] - pos[0]) < 0.05 and np.abs(current[4] - pos[1]) < 0.05
        if not res:
            RobotState.moving = False

        return res
    
    def checkEEfPositionAgainstBox(box_color, current):
        """
        Checks if end effector is close to box
        """
        # create target matrix
        target = utils.get_tf_above_box(box_holds[box_color], box_map[box_color])
        
        # check norm of error between target and current matrices
        res = np.linalg.norm(utils.calc_error(target, current)) < RobotState.error_threshold

        if res:
            RobotState.moving = False
            RobotState.above = False
        elif RobotState.grip_start_time is not None:
            RobotState.grip_start_time = None

        return res
    
    def checkEEfPosition(target, current):
        """
        Checks if end effector is close to given position
        """
        return np.linalg.norm(utils.calc_error(target, current)) < RobotState.error_threshold

    def moveToBox(box_color):
        if not RobotState.moving:
            # we don't need precise movement for moving above
            RobotState.error_threshold = ERROR_THRESHOLD_HIGH
            target = utils.get_tf_above_box(box_holds[box_color], box_map[box_color], z_offset=True)
            controller.set_target(target)

            RobotState.moving = True
            RobotState.above = False
    
        res = moveToGripPosition()

        # if successful, stop moving
        if res:
            robot.set_commands([0, 0, 0, 0, 0, 0, 0, 0, 0])
            RobotState.moving = False
            RobotState.above = False

        return res

    def setAndMoveToPos(tf):
        if not RobotState.moving:
            # we don't need precise movement for moving up and above target
            RobotState.error_threshold = ERROR_THRESHOLD_HIGH

            # set new target position
            controller.set_target(tf)
            RobotState.moving = True
            RobotState.above = False

        res = moveToEndPosition()

        if res == bt.Status.SUCCESS:
            RobotState.moving = False
            RobotState.above = False

        return res


    ### Main sequence node
    main_seq = bt.Sequence(memory=False)
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
        move_low_to_correct_pos = bt.Sequence(memory=False)

        # Branch 1: moving to position and grabbing
        move_to_grab_pos_fb = bt.Fallback()
        gripping_cond = bt.Condition(lambda: RobotState.gripping)
        move_grab_seq = bt.Sequence(memory=False)
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
            new_pos_matrix = Isometry3(utils.create_transformation_matrix(R_REG_HOLD, np.array((box_pos[0], box_pos[1]-0.15, 0.15))))
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

#########################################################

root = createBehaviorTree()

#red_box.set_draw_axis(red_box.body_name(0), 1.)
#blue_box.set_draw_axis(blue_box.body_name(0), 1.)
#green_box.set_draw_axis(green_box.body_name(0), 1.)
#robot.set_draw_axis(robot.body_name(0), 1.)
#robot.set_draw_axis(eef_link_name, 1.)
#np.set_printoptions(suppress=True)

#graphics.record_video("demos/demo_topt.mp4")

for step in range(total_steps):
    if (simu.schedule(simu.control_freq())):
        root.tick()

    if (simu.step_world()):
        break
