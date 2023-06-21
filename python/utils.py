import numpy as np
import RobotDART as rd
from dartpy.math import Isometry3

# X axis rotation matrix for a (in radians)
Rot_X = lambda a: np.array((
    (1, 0, 0),
    (0, np.cos(a), -np.sin(a)),
    (0, np.sin(a), np.cos(a))
))

# Y axis rotation matrix for a (in radians)
Rot_Y = lambda a: np.array((
    (np.cos(a), 0, np.sin(a)),
    (0, 1, 0),
    (-np.sin(a), 0, np.cos(a))
))

# Z axis rotation matrix for a (in radians)
Rot_Z = lambda a: np.array((
    (np.cos(a), -np.sin(a), 0),
    (np.sin(a), np.cos(a), 0),
    (0, 0, 1)
))

# matrix to rotate 90 degrees in z axis
rotate_90_z = np.array(((0, -1, 0), (1, 0, 0), (0, 0, 1)))

# rotation matrix for regular hold of end effector
R_REG_HOLD = np.array((
    (0, 1, 0),
    (1, 0, 0),
    (0, 0, -1)
))

# regular hold, rotated by 90 degrees
R_ROT_HOLD = R_REG_HOLD @ rotate_90_z

def create_grid(box_step_x=0.05, box_step_y=0.05):
    box_positions = []
    box_x_min = 0.3
    box_x_max = 0.7
    box_y_min = -0.4
    box_y_max = 0.4

    box_nx_steps = int(np.floor((box_x_max-box_x_min) / box_step_x))
    box_ny_steps = int(np.floor((box_y_max-box_y_min) / box_step_y))

    for x in range(box_nx_steps+1):
        for y in range(box_ny_steps+1):
            box_x = box_x_min + x * box_step_x
            box_y = box_y_min + y * box_step_y
            # if (np.linalg.norm([box_x, box_y]) < 1.):
            #     continue
            box_positions.append((box_x, box_y))

    return box_positions


def create_problems():
    cubes = ['red', 'green', 'blue']

    problems = []

    for cubeA in cubes:
        for cubeB in cubes:
            if cubeB == cubeA:
                continue
            for cubeC in cubes:
                if cubeC == cubeA or cubeC == cubeB:
                    continue
                problems.append([cubeA, cubeB, cubeC])
    
    return problems

# function for damped pseudo-inverse
def damped_pseudoinverse(jac, l = 0.01):
    m, n = jac.shape
    if n >= m:
        return jac.T @ np.linalg.inv(jac @ jac.T + l*l*np.eye(m))
    return np.linalg.inv(jac.T @ jac + l*l*np.eye(n)) @ jac.T

def create_transformation_matrix(R, t):
    """
    Merges rotation matrix and translation vector to create transformation matrix
    """
    return np.array((
        (R[0, 0], R[0, 1], R[0, 2], t[0]),
        (R[1, 0], R[1, 1], R[1, 2], t[1]),
        (R[2, 0], R[2, 1], R[2, 2], t[2]),
        (0, 0, 0, 1)
    ))

def isom3_to_np(isom3):
    """
    Returns np array of Isometry3 object
    """
    return create_transformation_matrix(isom3.rotation(), isom3.translation())

def calc_error(m1, m2):
    """
    Calculates errors for rotation and translation
    """
    rotational_error = rd.math.logMap(m1.rotation() @ m2.rotation().T)
    linear_error = m1.translation() - m2.translation()

    return np.r_[rotational_error, linear_error]

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    """
    Used to compare floats for almost-equality
    """
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def angle_wrap(a):
    """
    Wraps an angle to [-pi, pi)
    """
    return (a + np.pi) % (2 * np.pi) - np.pi

def angle_wrap_pi(a):
    """
    Wraps an angle to [-pi/2, pi/2]
    """
    a = angle_wrap(a)

    return np.arctan2(np.sin(a), abs(np.cos(a)))

def get_tf_above_box(hold_matrix, box, z_offset=False):
    angle = angle_wrap_pi(-box.positions()[2])
    rot_matrix = Rot_Z(angle)

    desired_translation = box.body_pose(0).translation()
    desired_total = create_transformation_matrix(hold_matrix @ rot_matrix, np.array((desired_translation[0], desired_translation[1], desired_translation[2] + (0.4 if z_offset else 0.1))))
    tf_desired = Isometry3(desired_total)
    
    return tf_desired
