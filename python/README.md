# Cube Stacking
#### Using the Franka robotic arm on RobotDART

**Problem description**

This repository contains an implementation for solving the cube stacking problem. The problem is described as follows.

The goal is to control the robotic arm, in order to place a number of cubes in a stack, in a given order. In this case, there are 3 cubes, distinguished by their individual colors (red, green and blue).

The initial positions of the cubes are random. More specifically, a grid is initialized, containing a total of 128 cells. Each of those is a potential box position. The initial coordinates for those positions are 0.4 to 0.7 in the x axis and -0.4 to 0.4 in the y axis, with a step of 0.05. As such, the cubes will occupy any of those positions.

In the implementation, the stack is constructed at a different position, outside the grid. This means that each cube is individually picked up, in the required order, and placed on the stack. Another possible solution would be to construct the stack at the initial position of the bottom cube, thus avoiding picking up that cube at all.

**Control**

There are two different implementations available for controlling the robotic arm to move to a goal. The default one utilizes a PI Controller in task space. There is also the option to use trajectory optimization instead.

**Execution**

In order to execute the script *cube_stacking*,  Python 3, numpy and RobotDART must be available on the target system. If trajectory optimization is selected, gekko must also be available.

By default, task space control is used. Trajectory optimization can be enabled using the command-line argument `-c topt`.

Certain edge cases when it comes to cube positions can also be enabled using the `cubepos` argument. Specifically, `--cubepos line` will place all three cubes next to each other (in a line), whereas `--cubepos triangle` will place two of them next to each other, and the third one below.

The order the cubes must be picked up in can also be specified using the `order` argument. By default, that order is random. To specify an order, use any permutation of the letters r, g and b.

**Potential Improvements**

Cubes with non-zero angles for roll and pitch will usually not be grabbed correctly. Therefore, one improvement would consist of calculating the rotation angle required for the gripper to correctly grip a cube, based on all of its angles, instead of assuming roll and pitch are zero.

For trajectory optimization, on certain occasions, a solution cannot be found, resulting in a crash. Additionally, the motion the gripper makes when moving towards a cube in order to grip it is sometimes "curved", resulting in one of the fingers crashing into the cube. Additional constraints could be added to the solver to prevent such motions.

