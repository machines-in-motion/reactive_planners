""" @namespace Demos of Bolt step adjustment
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import numpy as np
# np.set_printoptions(precision=2, suppress=True)
import pybullet as p
from matplotlib import pyplot as plt
from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot
from py_blmc_controllers.bolt_centroidal_controller import BoltCentroidalController
from py_blmc_controllers.bolt_impedance_controller import BoltImpedanceController
from py_reactive_planners.lipm_simulator import LipmSimpulator
import pinocchio as se3
import time as Time
from scipy.spatial.transform import Rotation as R
from reactive_planners import DcmReactiveStepper

def zero_cnt_gain(kp, cnt_array):
    gain = np.array(kp).copy()
    for i, v in enumerate(cnt_array):
        if v == 1:
            gain[3 * i:3 * (i + 1)] = 0.
    return gain


def joint_controller(q, desired_q, qdot, desired_qdot, kp, kd, cnt_array):
    torque = np.zeros((6, 1))
    number_of_joints_per_leg = 3
    for i in range(7, len(q)):
        torque[i - 7] = (cnt_array[int((i - 7) / number_of_joints_per_leg)] * (kp[i - 7] * (desired_q[i] - q[i])) + \
                         cnt_array[int((i - 7) / number_of_joints_per_leg)] * (
                                 kd[i - 7] * (desired_qdot[i - 1] - qdot[i - 1])))
    return torque


def yaw(q):
    return np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, 2]


if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    robot = BoltRobot()
    tau = np.zeros(6)
    p.resetDebugVisualizerCamera(1.2, 50, -35, (0., 0., 0.))
    p.setTimeStep(0.001)
    # p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "turnRight.mp4")

    q = np.matrix(BoltConfig.initial_configuration).T
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    total_mass = 1.13  # sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    warmup = 10
    kp = np.array([150., 150., 150., 150., 150., 150.])
    kd = [15., 15., 15., 15., 15., 15.]
    kp_joint = np.array([2., 2., 2., 2., 2., 2.])
    kd_joint = [.1, .01, .01, .1, .01, .01]
    x_ori = [0., 0., 0., 1.]
    x_angvel = [0., 0., 0]
    bolt_leg_ctrl = BoltImpedanceController(robot)
    centr_controller = BoltCentroidalController(robot.pin_robot, total_mass, mu=1, kp=[0, 0, 100], kd=[0, 0, 10],
                                                kpa=[100, 100, 100], kda=[10., 10, 10], eff_ids=robot.pinocchio_endeff_ids)

    sim = LipmSimpulator(.2)
    dcm_reactive_stepper = DcmReactiveStepper()
    is_left_leg_in_contact = True
    l_min = -0.12
    l_max = 0.12
    w_min = -0.1
    w_max = 0.3
    t_min = 0.2
    t_max = 0.5
    l_p = 0.1235 * 1
    com_height = 0.26487417
    weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
    mid_air_foot_height = .05
    control_period = 0.001
    x_des_local = [0., 0.01, 0., 0., -0.01, 0.]
    dcm_reactive_stepper.initialize(is_left_leg_in_contact, l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height,
                                    weight, mid_air_foot_height, control_period, x_des_local[:3], x_des_local[3:])
    #previous_support_foot=[[0.0], [-0.075], [0.]],
    #current_support_foot=[[0.0], [0.075], [0.]]
    dcm_reactive_stepper.set_costs(1e1, 1e1, 1e0, 1e-9)
    v_des = [0., .0, .0]
    dcm_reactive_stepper.set_desired_com_velocity(v_des)

    x_com = np.zeros((3, 1))
    x_com[:] = [[.0], [.0], [com_height]]
    xd_com = np.zeros((3, 1))
    x_com_cent = x_com.copy()
    xd_com_cent = xd_com.copy()
    omega = np.sqrt(9.8 / com_height)
    cnt_array = [1, 1]
    time = 0

    #plot
    plt_time = []
    plt_x_com = []
    plt_xd_com = []
    plt_right_foot_position = []
    plt_right_foot_velocity = []
    plt_right_foot_acceleration = []
    plt_left_foot_position = []
    plt_left_foot_velocity = []
    plt_left_foot_acceleration = []
    plt_time_from_last_step_touchdown = []
    # plt_duration_before_step_landing = []
    plt_current_support_foot = []
    plt_step_time = []
    plt_dcm_local = []
    plt_left_eef_real_pos = []
    plt_right_eef_real_pos = []
    plt_r = []
    plt_is_left_in_contact = []
    plt_pos_des_local = []
    plt_q_com = []
    plt_qdot_com = []
    plt_F = []
    plt_x = []
    plt_q = []
    plt_qdot = []
    plt_tau = []
    plt_euler_angles = []
    plt_dcm = []
    plt_next_step_location = []

    dcm_reactive_stepper.start()
    for i in range(3000):
        q, qdot = robot.get_state()
        robot.pin_robot.com(q, qdot)
        x_com = robot.pin_robot.com(q, qdot)[0]
        xd_com = robot.pin_robot.com(q, qdot)[1]

        #External force
        # if i > 1575 and i < 1700:
        #     force = np.array([-10, 0, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)

        if warmup <= i:
            left = bolt_leg_ctrl.imps[0]
            right = bolt_leg_ctrl.imps[1]
            left_foot_location = np.array(left.pin_robot.data.oMf[left.frame_end_idx].translation).\
                                  reshape(-1)
            left_foot_location[2] = 0.
            right_foot_location = np.array(right.pin_robot.data.oMf[right.frame_end_idx].translation).\
                                    reshape(-1)
            right_foot_location[2] = 0.

            dcm_reactive_stepper.run(time, left_foot_location,
                                     right_foot_location, x_com, xd_com, yaw(q))
            print(yaw(q))
            # if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
            #     desired_q = np.array(q.copy())[:, 0]
            # else:
            #     dcm_reactive_stepper.run(time, dcm_reactive_stepper.flying_foot_position, x_com.copy(), xd_com.copy(), 0)  # q[5])

            x_des_local = []
            x_des_local.extend(dcm_reactive_stepper.get_left_foot_position())
            x_des_local.extend(dcm_reactive_stepper.get_right_foot_position())
            x_des_local[2] += 0.0171
            x_des_local[5] += 0.0171

            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                cnt_array = [1, 0]
            else:
                cnt_array = [0, 1]

            plt_time.append(time)
            plt_right_foot_position.append(x_des_local[3:6])
            plt_right_foot_velocity.append(dcm_reactive_stepper.get_right_foot_velocity().copy())
            plt_right_foot_acceleration.append(dcm_reactive_stepper.get_right_foot_acceleration().copy())
            plt_left_foot_position.append(x_des_local[:3])
            plt_left_foot_velocity.append(dcm_reactive_stepper.get_left_foot_velocity().copy())
            plt_left_foot_acceleration.append(dcm_reactive_stepper.get_left_foot_acceleration().copy())
            plt_time_from_last_step_touchdown.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
            # plt_duration_before_step_landing.append(dcm_reactive_stepper.duration_before_step_landing)
            plt_current_support_foot.append(dcm_reactive_stepper.get_current_support_foot_position().copy())
            # plt_dcm.append(dcm_reactive_stepper.dcm_vrp_planner.get_dcm_local().copy())
            plt_is_left_in_contact.append(dcm_reactive_stepper.get_is_left_leg_in_contact())
            plt_next_step_location.append(dcm_reactive_stepper.get_next_support_foot_position().copy())
            plt_dcm_local.append(x_com + xd_com / omega)
            if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
                plt_step_time.append(int(i) - warmup)
            time += 0.001

        for j in range(2):
            imp = bolt_leg_ctrl.imps[j]
            print("LhumImp", j, " ", np.array(imp.pin_robot.data.oMf[imp.frame_root_idx].translation).\
                                                       reshape(-1))
            x_des_local[3 * j:3 * (j + 1)] -= np.array(imp.pin_robot.data.oMf[imp.frame_root_idx].translation).\
                                                       reshape(-1)
            if j == 0:
                plt_left_eef_real_pos.append(
                    np.array(imp.pin_robot.data.oMf[imp.frame_end_idx].translation).reshape(-1))
            else:
                plt_right_eef_real_pos.append(
                    np.array(imp.pin_robot.data.oMf[imp.frame_end_idx].translation).reshape(-1))

        w_com = centr_controller.compute_com_wrench(q.copy(), qdot.copy(), [0.0, 0.0, com_height], [0.0, 0.0, 0.0],
                                                    [0, 0., 0., 1.], [0., 0., 0.])
        w_com[0] = 0.0
        w_com[1] = 0.0
        w_com[2] += total_mass * 9.81

        F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)
        # torque = joint_controller(q, desired_q, qdot, desired_qdot, kp_joint, kd_joint, cnt_array)
        print("$$$$$$$$$$$$$")
        print([qdot[0].item(), qdot[1].item(), qdot[2].item()])
        print(dcm_reactive_stepper.get_left_foot_velocity() - [qdot[0].item(), qdot[1].item(), qdot[2].item()])
        des_vel = np.concatenate((dcm_reactive_stepper.get_left_foot_velocity() -[qdot[0].item(), qdot[1].item(), qdot[2].item()],
                                  dcm_reactive_stepper.get_right_foot_velocity() - [qdot[0].item(), qdot[1].item(), qdot[2].item()]))
        print(dcm_reactive_stepper.get_left_foot_velocity() + dcm_reactive_stepper.get_right_foot_velocity())
        tau, r = bolt_leg_ctrl.return_joint_torques(q.copy(), qdot.copy(), zero_cnt_gain(kp, cnt_array),
                                                 zero_cnt_gain(kd, cnt_array),
                                                 x_des_local, des_vel, F)
        plt_r.append(r)
        plt_F.append(F)
        plt_x.append(x_des_local)
        plt_tau.append(tau)
        plt_xd_com.append(xd_com.copy())
        plt_qdot_com.append(qdot)
        plt_x_com.append(x_com.copy())
        plt_pos_des_local.append([x_des_local[1], x_des_local[4]])
        plt_euler_angles.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
        plt_q.append(q[7:].copy())
        plt_qdot.append(qdot[6:].copy())
        # plt_q_com.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
        # plt_desired_q.append(desired_q[7:].copy())

        # for i in range(10):
        robot.send_joint_command(tau)
        p.stepSimulation()
        # Time.sleep(0.01)
    print(plt_step_time)
    dcm_reactive_stepper.stop()
    # p.stopStateLogging()

    # plt.figure("com")
    # plt.plot(plt_time, np.array(plt_x_com)[:,0])
    # plt.plot(plt_time, np.array(plt_x_com)[:,1])
    # plt.plot(plt_time, np.array(plt_x_com)[:,2])

    # plt.figure("xd")
    # plt.plot(plt_time, np.array(plt_xd_com)[:,0])
    # plt.plot(plt_time, np.array(plt_xd_com)[:,1])
    # plt.plot(plt_time, np.array(plt_xd_com)[:,2])

    # plt.figure("right_foot_pos")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,0])
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,1])
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,2])

    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,0])
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,1])
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,2])

    # plt.figure("feet_pos_y")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,2])
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,2])

    plt.figure("y")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 1], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 1], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
    plt.plot(plt_time, np.array(plt_next_step_location)[:, 1], label="next_step_location")
    # plt.plot(plt_time, np.array(plt_dcm)[:, 1], label="dcm")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 1], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 1], label="right_eef_real_pos")
    plt.plot(plt_time, np.array(plt_current_support_foot)[:, 1], label="current_support_foot")
    # plt.plot(plt_time, plt_pos_des_local[warmup + 1:], label = "pos des_local_eef")
    plt.legend()
    for time in plt_step_time:
        plt.axvline(time / 1000)

    # plt.figure("q")
    # plt.plot(plt_time, np.array(plt_qdot_com)[:, 6:, 0], label="plt_qdot_com")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("x")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="des_left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="des_right")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 0], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 0], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    plt.plot(plt_time, np.array(plt_next_step_location)[:, 0], label="next_step_location")
    # plt.plot(plt_time, np.array(plt_dcm)[:, 0], label="dcm")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 0], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 0], label="right_eef_real_pos")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 0], label="current_support_foot")
    # plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:], label="plt_duration_before_step_landing")
    plt.legend()
    for time in plt_step_time:
        plt.axvline(time / 1000)

    plt.figure("tau")
    plt.plot(plt_time[:], np.array(plt_tau)[warmup:, :, 0], label="tau")
    plt.legend()

    plt.figure("z")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 2], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 2], label="right")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 2], label="com")
    # plt.plot(plt_time, np.array(plt_dcm_local)[:, 2], label="dcm_local")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 2], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 2], label="right_eef_real_pos")
    plt.legend()
    for time in plt_step_time:
        plt.axvline(time / 1000)

    # plt.figure("q")
    # plt.plot(plt_time, np.array(plt_q_com)[:, 3], label="x")
    # plt.plot(plt_time, np.array(plt_q_com)[:, 4], label="y")
    # plt.plot(plt_time, np.array(plt_q_com)[:, 5], label="z")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    # plt.figure("F")
    # plt.plot(plt_time, plt_F[warmup:], label="F")
    # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
    # plt.legend()

    # plt.figure("last_step_touchdown")
    # plt.plot(plt_time, np.array(plt_time_from_last_step_touchdown)[:])
    # plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:])

    # plt.figure("support_foot")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:,0])
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:,1])
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:,2])

    plt.figure("warmup2")
    # plt.plot(np.array(plt_x)[:, :], label="des")
    # plt.plot(np.array(plt_r)[warmup:, 2, 0], label = "real")
    plt.plot(np.array(plt_F)[warmup:, :], label = "Force")
    plt.legend()
    #
    # plt.figure("warm up")
    # plt.plot(np.array(plt_left_eef_real_pos)[1:, :], label="left_eef_real_pos")
    # plt.plot(np.array(plt_right_eef_real_pos)[1:, :], label="right_eef_real_pos")
    # plt.legend()
    #
    # plt.figure("warm up1")
    # plt.plot(np.array(plt_x_com)[:, :], label="com")
    # plt.plot(np.array(plt_xd_com)[:, :], label="xd_com")
    # plt.plot(np.array(plt_qdot_com)[:, :6, 0], label="qdot")
    # plt.legend()

    plt.show()
