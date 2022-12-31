"""
This file currently gives you the results for best. To get other results, comment out the code used
to get the results for best (Kp, Ki, code to generate csv files, and plot title) and uncomment
other code used to get the results for either overshoot or newTask (Kp, Ki, code to generate csv
files, and plot title). Note that for newTask, you will also have to comment out the original 
Tsc_initial and Tsc_final and uncomment the ones right below them.

To run this file, use this command in the terminal:
python3 main_program.py
"""

import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    """
    Generates the reference trajectory for the end-effector frame {e}

    Args:
        Tse_initial: the initial configuration of the end-effector in the 
                     reference trajectory
        Tsc_initial: the cube's initial configuration
        Tsc_final: the cube's desired final configuration
        Tce_grasp: the end-effector's configuration relative to the cube when 
                   it is grasping the cube
        Tce_standoff: the end-effector's standoff configuration above the cube, 
                      before and after grasping, relative to the cube
        k: the number of trajectory reference configurations per 0.01 seconds

    Returns:
        traj_seg_list: a list that represents N configurations of the end-effector
        along the eight-segment reference trajectory
    """
    Tf = 3 # Total time of the motion
    N = (Tf * k) / 0.01 # The number of points in the discrete representation
    method = 5 # Time-scaling method
    traj_seg_list = []

    # 1. Move the gripper from its initial configuration to standoff above the block
    gripper_state = 0
    start_config = Tse_initial
    final_config = np.matmul(Tsc_initial, Tce_standoff)
    traj_seg1 = mr.ScrewTrajectory(start_config, final_config, Tf, N, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg1:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 2. Move the gripper down
    gripper_state = 0
    start_config = np.matmul(Tsc_initial, Tce_standoff)
    final_config = np.matmul(Tsc_initial, Tce_grasp)
    traj_seg2 = mr.ScrewTrajectory(start_config, final_config, Tf/3, N/3, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg2:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 3. Close the gripper
    gripper_state = 1
    start_config = np.matmul(Tsc_initial, Tce_grasp)
    final_config = np.matmul(Tsc_initial, Tce_grasp)
    traj_seg3 = mr.ScrewTrajectory(start_config, final_config, Tf/3, N/3, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg3:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 4. Move the gripper back up to standoff
    gripper_state = 1
    start_config = np.matmul(Tsc_initial, Tce_grasp)
    final_config = np.matmul(Tsc_initial, Tce_standoff)
    traj_seg4 = mr.ScrewTrajectory(start_config, final_config, Tf/3, N/3, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg4:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 5. Move the gripper to standoff above the final configuration
    gripper_state = 1
    start_config = np.matmul(Tsc_initial, Tce_standoff)
    final_config = np.matmul(Tsc_final, Tce_standoff)
    traj_seg5 = mr.ScrewTrajectory(start_config, final_config, Tf, N, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg5:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 6. Move the gripper to the final configuration
    gripper_state = 1
    start_config = np.matmul(Tsc_final, Tce_standoff)
    final_config = np.matmul(Tsc_final, Tce_grasp)
    traj_seg6 = mr.ScrewTrajectory(start_config, final_config, Tf/3, N/3, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg6:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 7. Open the gripper
    gripper_state = 0
    start_config = np.matmul(Tsc_final, Tce_grasp)
    final_config = np.matmul(Tsc_final, Tce_grasp)
    traj_seg7 = mr.ScrewTrajectory(start_config, final_config, Tf/3, N/3, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg7:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)

    # 8. Move the gripper back to standoff
    gripper_state = 0
    start_config = np.matmul(Tsc_final, Tce_grasp)
    final_config = np.matmul(Tsc_final, Tce_standoff)
    traj_seg8 = mr.ScrewTrajectory(start_config, final_config, Tf/3, N/3, method)
    # Iterate over a list of arrays, extract values, and append them to a list
    for traj in traj_seg8:
        entry = [traj[0, 0], traj[0, 1], traj[0, 2], traj[1, 0], traj[1, 1], traj[1, 2], 
                 traj[2, 0], traj[2, 1], traj[2, 2], traj[0, 3], traj[1, 3], traj[2, 3], 
                 gripper_state]
        traj_seg_list.append(entry)
    
    return traj_seg_list

def NextState(curr_config, speed_controls, dt, max_ang_speed):
    """
    Simulates the kinematics of the youBot

    Args:
        curr_config: a 12-vector representing the current configuration of the robot
        speed_controls: a 9-vector of controls indicating the wheel speeds and the arm joint speeds
        dt: a timestep
        max_ang_speed: a positive real value indicating the maximum angular speed of the arm joints
                       and the wheels

    Returns:
        later_config: a 12-vector representing the configuration of the robot time dt later
    """
    r = 0.0475 # Radius of each wheel
    l = 0.235 # Forward-backward distance between the wheels
    w = 0.15 # Side-to-side distance between wheels

    # Define configuration variables
    phi = curr_config[0]
    chas_config = curr_config[0:3]
    wheel_speeds = speed_controls[0:4]
    joint_speeds = speed_controls[4:9]

    # Check if each speed is within the range
    for i in range(0, len(wheel_speeds)):
        if wheel_speeds[i] < -max_ang_speed:
            wheel_speeds[i] = -max_ang_speed
        elif wheel_speeds[i] > max_ang_speed:
            wheel_speeds[i] = max_ang_speed
    for j in range(0, len(joint_speeds)):
        if joint_speeds[j] < -max_ang_speed:
            joint_speeds[j] = -max_ang_speed
        elif joint_speeds[j] > max_ang_speed:
            joint_speeds[j] = max_ang_speed

    # Calculate body twist (Vb)
    F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])
    Vb = np.matmul(F, wheel_speeds)

    # Extract w_bz, v_bx, and v_by from Vb
    w_bz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]

    # Calculate dq
    if w_bz == 0:
        dq_b = np.array([0, v_bx, v_by])
    else:
        dq_b = np.array([w_bz, (v_bx * np.sin(w_bz) + v_by * (np.cos(w_bz) - 1)) / w_bz, 
                        (v_by * np.sin(w_bz) + v_bx * (1 - np.cos(w_bz))) / w_bz])
    mat = np.array([[1, 0, 0],
                    [0, np.cos(phi), -np.sin(phi)],
                    [0, np.sin(phi), np.cos(phi)]])
    dq = np.matmul(mat, dq_b.T)

    # Get the updated configuration
    old_arm_joint_angles = curr_config[3:8]
    old_wheel_angles = curr_config[8:12]
    new_arm_joint_angles = old_arm_joint_angles + joint_speeds * dt
    new_wheel_angles = old_wheel_angles + wheel_speeds * dt
    updated_chas_config = chas_config + dq * dt
    later_config = np.concatenate((updated_chas_config, new_arm_joint_angles, new_wheel_angles), axis=None)
    later_config = list(later_config)

    return later_config

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_integ, curr_config):
    """
    Calculates the kinematic task-space feedforward plus feedback control law

    Args:
        X: the current actual end-effector configuration
        Xd: the current end-effector reference configuration
        Xd_next: the end-effector reference configuration at the next timestep in the 
                 reference trajectory
        Kp: the Kp gain matrix
        Ki: the Ki gain matrix
        dt: the timestep between reference trajectory configurations
        Xerr_integ: the integral of errors
        curr_config: the robot configuration

    Returns:
        Ve: the commanded end-effector twist expressed in the end-effector frame
        Xerr: the error twist
        Xerr_integ: the integral of errors
        com_joint_speeds: the commanded wheel and arm joint speeds
    """
    # Calculate Xerr
    X_inv = np.linalg.inv(X)
    Xerr = mr.se3ToVec(mr.MatrixLog6(np.matmul(X_inv, Xd)))

    # Compute the feedforward reference twist (Vd)
    Xd_inv = np.linalg.inv(Xd)
    Vd = mr.se3ToVec((1 / dt) * mr.MatrixLog6(np.matmul(Xd_inv, Xd_next)))

    # Compute the commanded end-effector twist (Ve)
    Ad_term = np.matmul(mr.Adjoint(np.matmul(X_inv, Xd)), Vd)
    Xerr_integ += Xerr * dt
    Ve = Ad_term + np.matmul(Kp, Xerr) + np.matmul(Ki, Xerr_integ)

    # Define parameters
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])
    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]])
    Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                      [0, -1, 0, -0.5076, 0, 0],
                      [0, -1, 0, -0.3526, 0, 0],
                      [0, -1, 0, -0.2176, 0, 0],
                      [0, 0, 1, 0, 0, 0]]).T
    r = 0.0475 # Radius of each wheel
    l = 0.235 # Forward-backward distance between the wheels
    w = 0.15 # Side-to-side distance between wheels
    joint_angles = curr_config[3:8]
    F6 = (r / 4) * np.array([[0, 0, 0, 0],
                             [0, 0, 0, 0],
                             [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                             [1, 1, 1, 1],
                             [-1, 1, -1, 1],
                             [0, 0, 0, 0]])
    
    T0b = np.linalg.inv(Tb0)
    T0e = mr.FKinBody(M0e, Blist, joint_angles)
    Te0 = np.linalg.inv(T0e)
    Ja = mr.JacobianBody(Blist, joint_angles)
    Ja = Ja.T # Ja is transposed to be used in check_joint_limits
    # Ja = check_joint_limits(joint_angles, Ja)
    Ja = Ja.T
    Jb = np.matmul(mr.Adjoint(np.matmul(Te0, T0b)), F6)
    Je = np.hstack((Jb, Ja))

    # Get the commanded wheel and arm joint speeds
    com_joint_speeds = np.matmul(np.linalg.pinv(Je), Ve)

    # Accounting for singularities
    # com_joint_speeds = np.matmul(np.linalg.pinv(Je, 1e-3), Ve)

    return Ve, Xerr, Xerr_integ, com_joint_speeds

def check_joint_limits(joint_angles, Ja):
    """
    Checks if a joint angle is within a certain range

    Args:
        joint_angles: the arm joint angles
        Ja: the Jacobian of the arm

    Returns:
        Ja_mod: the modified Ja with columns of 0
    """
    if joint_angles[0] > 2.75 or joint_angles[0] < -2.75:
        Ja[0] = Ja[0] * 0
    if joint_angles[1] > 2 or joint_angles[1] < -2:
        Ja[1] = Ja[1] * 0
    if joint_angles[2] > 2.35 or joint_angles[2] < -2.35:
        Ja[2] = Ja[2] * 0
    if joint_angles[3] > 4 or joint_angles[3] < -4:
        Ja[3] = Ja[3] * 0
    if joint_angles[4] > 2.75 or joint_angles[4] < -2.75:
        Ja[4] = Ja[4] * 0
    Ja_mod = Ja
    return Ja_mod

def main():
    """
    The main function
    """
    # Initialize parameters
    Tse_initial = np.array([[0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [-1, 0, 0, 0.5],
                            [0, 0, 0, 1]])
    Tsc_initial = np.array([[1, 0, 0, 1],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.025],
                            [0, 0, 0, 1]])
    Tsc_final = np.array([[0, 1, 0, 0],
                          [-1, 0, 0, -1],
                          [0, 0, 1, 0.025],
                          [0, 0, 0, 1]])

    # Tsc_initial and Tsc_final for the newTask

    # Tsc_initial = np.array([[1, 0, 0, 0.5],
    #                         [0, 1, 0, 0.5],
    #                         [0, 0, 1, 0.025],
    #                         [0, 0, 0, 1]])
    # Tsc_final = np.array([[0, 1, 0, 0],
    #                       [-1, 0, 0, -0.6],
    #                       [0, 0, 1, 0.025],
    #                       [0, 0, 0, 1]])
    
    Tce_grasp = np.array([[np.cos(2*np.pi/3), 0, np.sin(2*np.pi/3), 0],
                          [0, 1, 0, 0],
                          [-np.sin(2*np.pi/3), 0, np.cos(2*np.pi/3), 0],
                          [0, 0, 0, 1]])
    Tce_standoff = np.array([[np.cos(2*np.pi/3), 0, np.sin(2*np.pi/3), 0],
                             [0, 1, 0, 0],
                             [-np.sin(2*np.pi/3), 0, np.cos(2*np.pi/3), 0.1],
                             [0, 0, 0, 1]])
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])
    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]])
    Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                      [0, -1, 0, -0.5076, 0, 0],
                      [0, -1, 0, -0.3526, 0, 0],
                      [0, -1, 0, -0.2176, 0, 0],
                      [0, 0, 1, 0, 0, 0]]).T
    k = 1
    max_ang_speed = 15

    Kp = 7 * np.identity(6) # best
    Ki = 0.01 * np.identity(6) # best

    # Kp = 2 * np.identity(6) # overshoot
    # Ki = 7 * np.identity(6) # overshoot

    # Kp = 7 * np.identity(6) # newTask
    # Ki = 0.01 * np.identity(6) # newTask

    dt = 0.01
    Xerr_integ = ([0, 0, 0, 0, 0, 0])

    # Define initial configuration
    curr_config = np.array([-0.1, -0.4, 0.2, 0.4, 0.2, -0.2, -0.2, 0, 0, 0, 0, 0, 0])

    print("Creating trajectory segment list")
    traj_seg_list = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, 
                                        Tce_standoff, k)
    
    # Define a list to store configurations
    config_list = [curr_config]

    # Define lists to store errors
    error_list = []
    error_integ_list = []
    error1_list = []
    error2_list = []
    error3_list = []
    error4_list = []
    error5_list = []
    error6_list = []

    for i in range(0, len(traj_seg_list) - 1):
        # Get parameters each time through the loop
        phi = curr_config[0]
        x = curr_config[1]
        y = curr_config[2]
        joint_angles = curr_config[3:8]
        Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                        [np.sin(phi), np.cos(phi), 0, y],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
        T0e = mr.FKinBody(M0e, Blist, joint_angles)
        Ts0 = np.matmul(Tsb, Tb0)
        X = np.matmul(Ts0, T0e)
        Xd = np.array([[traj_seg_list[i][0], traj_seg_list[i][1], traj_seg_list[i][2], traj_seg_list[i][9]],
                       [traj_seg_list[i][3], traj_seg_list[i][4], traj_seg_list[i][5], traj_seg_list[i][10]],
                       [traj_seg_list[i][6], traj_seg_list[i][7], traj_seg_list[i][8], traj_seg_list[i][11]],
                       [0, 0, 0, 1]])
        Xd_next = np.array([[traj_seg_list[i + 1][0], traj_seg_list[i + 1][1], traj_seg_list[i + 1][2], traj_seg_list[i + 1][9]],
                            [traj_seg_list[i + 1][3], traj_seg_list[i + 1][4], traj_seg_list[i + 1][5], traj_seg_list[i + 1][10]],
                            [traj_seg_list[i + 1][6], traj_seg_list[i + 1][7], traj_seg_list[i + 1][8], traj_seg_list[i + 1][11]],
                            [0, 0, 0, 1]])

        # Calculate the control law and generate the wheel and joint controls
        Ve, Xerr, Xerr_integ, com_joint_speeds = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, 
                                                                 Xerr_integ, curr_config)

        # Calculate the new configuration using the controls, configuration, and timestep
        curr_config = NextState(curr_config[0:12], com_joint_speeds, dt, max_ang_speed)

        curr_config = np.concatenate((curr_config, traj_seg_list[i][-1]), axis=None)
        config_list.append(curr_config)
        error_list.append(Xerr)
        error_integ_list.append(Xerr_integ)
        error1_list.append(Xerr[0])
        error2_list.append(Xerr[1])
        error3_list.append(Xerr[2])
        error4_list.append(Xerr[3])
        error5_list.append(Xerr[4])
        error6_list.append(Xerr[5])
    
    print("Generating csv file")

    # Code to generate csv files

    np.savetxt("best.csv", config_list, delimiter=",")
    np.savetxt("best_error.csv", error_list, delimiter=",")
    # np.savetxt("overshoot.csv", config_list, delimiter=",")
    # np.savetxt("overshoot_error.csv", error_list, delimiter=",")
    # np.savetxt("newTask.csv", config_list, delimiter=",")
    # np.savetxt("newTask_error.csv", error_list, delimiter=",")

    print("Plotting error data")

    # Plot the errors
    plt.plot(np.linspace(0, 25, len(error_integ_list)), error1_list, label="Roll Error")
    plt.plot(np.linspace(0, 25, len(error_integ_list)), error2_list, label="Pitch Error")
    plt.plot(np.linspace(0, 25, len(error_integ_list)), error3_list, label="Yaw Error")
    plt.plot(np.linspace(0, 25, len(error_integ_list)), error4_list, label="X Error")
    plt.plot(np.linspace(0, 25, len(error_integ_list)), error5_list, label="Y Error")
    plt.plot(np.linspace(0, 25, len(error_integ_list)), error6_list, label="Z Error")
    plt.title('Best, Kp = 7, Ki = 0.01')
    # plt.title('Overshoot, Kp = 7, Ki = 0.01')
    # plt.title('NewTask, Kp = 7, Ki = 0.01')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m or rad)')
    plt.legend()
    plt.show()

    # Plot the errors in subplots

    # fig, axs = plt.subplots(2, 3)
    # axs[0, 0].plot(error1_list)
    # axs[0, 0].set_title('Roll Error')
    # axs[0, 0].set(ylabel="Error (rad)")
    # axs[0, 1].plot(error2_list)
    # axs[0, 1].set_title('Pitch Error')
    # axs[0, 1].set(ylabel="Error (rad)")
    # axs[0, 2].plot(error3_list)
    # axs[0, 2].set_title('Yaw Error')
    # axs[0, 2].set(ylabel="Error (rad)")
    # axs[1, 0].plot(error4_list)
    # axs[1, 0].set_title('X Error')
    # axs[1, 0].set(ylabel="Error (m)")
    # axs[1, 1].plot(error5_list)
    # axs[1, 1].set_title('Y Error')
    # axs[1, 1].set(ylabel="Error (m)")
    # axs[1, 2].plot(error6_list)
    # axs[1, 2].set_title('Z Error')
    # axs[1, 2].set(ylabel="Error (m)")
    # for ax in axs.flat:
    #     ax.set(xlabel='Time (s)')
    # fig.tight_layout()
    # plt.show()
    print("Done")

if __name__ == '__main__':
    main()