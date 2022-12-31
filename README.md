# Mobile Manipulation Using the Kuka youBot in CoppeliaSim

## Software
The code directory contains `main_program.py`, which is the program that needs to be run
to get the necessary csv files for the robot configurations and the error twist (Xerr). Within the
program, there are three important functions: `NextState`, `TrajectoryGenerator`, and
`FeedbackControl`. `NextState` takes in the current robot configuration and the joint speeds to
calculate the future configuration (chassis phi, chassis x, chassis y, arm joint angles, and the
wheel angles). `TrajectoryGenerator` generates configurations (transformation matrices and
gripper states) of the end-effector along the eight-segment reference trajectory. Finally,
`FeedbackControl` calculates the kinematic task-space feedforward plus feedback control law
using Kp, Ki, and the robot configurations. Using the commanded end-effector twist (Ve)
calculated from the control law and the pseudoinverse of the end-effector Jacobian (Je), the
commanded wheel and arm joint speeds are computed.

## Results
The results directory contains three directories: best, overshoot, and newTask. Each
directory contains 6 files: a README.pdf, a csv file of the configurations, a video of the robot
performing the pick-and-place task in CoppeliaSim, a csv file of Xerr, a png file of the plot of
Xerr as a function of time, and a txt file showing my program being called in the terminal.

*best:*<br />
In best, a well-tuned controller (feedforward-plus-PI with Kp = 7 and Ki = 0.01) was used
to allow the robot to pick up the cube and place it at a designated location. In the animation, the
motion was smooth, and there was no oscillation. As shown in the plot, the error converges to
zero before the end-effector reaches a standoff configuration above the cube. However, there is
a very little error when the end-effector goes to the final configuration of the cube.

*overshoot:*<br />
In overshoot, a less well-tuned controller (feedforward-plus-PI with Kp = 2 and Ki = 7)
was used to allow the robot to accomplish the same tasks. In the animation, when the robot
goes to the cube, the end-effector slightly goes past the initial location of the cube, but adjusts
its position to pick up the cube and continues to move to the final location to place the cube. The
plot clearly shows that the end-effector oscillates back after it slightly goes past the cube.

*newTask:*<br />
In newTask, a well-tuned controller (feedforward-plus-PI with Kp = 7 and Ki = 0.01) was
used to allow the robot to pick up the cube at a different location and place it at a different goal
location. The initial configuration of the cube was changed to (0.5 m, 0.5 m, 0 rad) and the goal
configuration of the cube was changed to (0 m, -0.6 m, -1.57079632679 rad) in CoppeliaSim.
Tsc_initial and Tsc_final were also changed in main_program.py to be used in
TrajectoryGenerator. Just like in the plot for best, the error converges to zero before the
end-effector reaches a standoff configuration above the cube. However, there is a very little
error when the end-effector goes to the final configuration of the cube.

## Important Note
Please note that main_program.py currently gives you the results for best. To get other
results, comment out the code used to get the results for best (Kp, Ki, code to generate csv
files, and plot title) and uncomment other code used to get the results for either overshoot or
newTask (Kp, Ki, code to generate csv files, and plot title). Please also note that for newTask,
you will also have to comment out the original Tsc_initial and Tsc_final and uncomment the ones
right below them. To run the program, you will need to enter `python3 main_program.py` in the
terminal.