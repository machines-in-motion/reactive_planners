#!/usr/bin/env python

""" @namespace Demos of the reactive_planners.dcm_reactive_planner.
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""

from matplotlib import pyplot as plt
from reactive_planners.dcm_reactive_stepper import DcmReactiveStepper
from reactive_planners.lipm_simulator import LipmSimpulator
import numpy as np

if __name__ == "__main__":
    x_com = np.zeros((3, 1))
    x_com[:] = [[0], [0], [0.3]]
    xd_com = np.zeros((3, 1))
    v_z = (2/((np.e **(10.18 * .1)) + 1) * (0.3 - (9.81 / (10.18 * 10.18))) - 0.3 + (9.81 / (10.18 * 10.18))) * 10.18
    xd_com[:] = [[0], [0], [-0.980718500274]]
    print("! ", xd_com)
    omega = 10.18
    sim = LipmSimpulator(x_com, xd_com, omega)
    print("@ ", xd_com)
    t_s = 0.1

    dcm_reactive_stepper = DcmReactiveStepper(
        is_left_leg_in_contact=True,
        l_min=-1.5,
        l_max=1.5,
        w_min=-1.5,
        w_max=1.5,
        t_min=0.05,
        t_max=0.15,
        l_p=0.1235,
        com_height=0.26487417,
        weight=[1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000],
        mid_air_foot_height=0.05,
        control_period=0.001,
        previous_support_foot=[[0.0], [-0.075], [0.0]],
        current_support_foot=[[0.0], [0.075], [0.0]],
    )
    print("# ", xd_com)
    dcm_reactive_stepper.set_end_eff_traj_costs(1e1, 1e1, 1e0, 1e-6)
    v_des = np.zeros((3, 1))
    v_des[:] = [[0.0], [0.0], [0.0]]
    dcm_reactive_stepper.set_des_com_vel(v_des)
    time = 0
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
    plt_duration_before_step_landing = []
    plt_current_support_foot = []
    plt_support_foot = []
    plt_swing_foot = []
    plt_step_time = []
    plt_dcm_local = []
    plt_is_left_leg_in_contact = []
    t = 0
    contact = [True, False]
    support_foot = [0.0, 0.075, 0.0]
    swing_foot = [0.0, -0.075, 0.0]
    for i in range(3265):
        print("s1 ", support_foot)
        if dcm_reactive_stepper.time_from_last_step_touchdown == 0 and i != 1:
            support_foot[0] = swing_foot[0]
            support_foot[1] = swing_foot[1]
        print("s2 ", support_foot)
        if dcm_reactive_stepper.is_left_leg_in_contact:
            swing_foot = dcm_reactive_stepper.dcm_vrp_planner.get_next_step_location().copy()

        else:
            swing_foot = dcm_reactive_stepper.dcm_vrp_planner.get_next_step_location().copy()

        dcm_reactive_stepper.run(
            time, swing_foot, support_foot, x_com, xd_com, 0, contact
        )
        print("s3 ", support_foot)
        # if i == 760:
        #     xd_com[1] = xd_com[1] - 0.2
        #     t = dcm_reactive_stepper.time_from_last_step_touchdown
        # if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
        #     t = 0
        if dcm_reactive_stepper.time_from_last_step_touchdown - t <= t_s - 0.00001:
            contact = [dcm_reactive_stepper.is_left_leg_in_contact,
                       not dcm_reactive_stepper.is_left_leg_in_contact]
        else:
            contact = [0, 0]
        x_com, xd_com, _ = sim.step(
            dcm_reactive_stepper.time_from_last_step_touchdown - t,
            dcm_reactive_stepper.current_support_foot,
            x_com,
            xd_com,
            contact[0] + contact[1]
            )
        plt_time.append(time)
        plt_x_com.append(x_com.copy())
        plt_xd_com.append(xd_com.copy())
        plt_right_foot_position.append(
            dcm_reactive_stepper.right_foot_position.copy()
        )
        plt_right_foot_velocity.append(
            dcm_reactive_stepper.right_foot_velocity.copy()
        )
        plt_right_foot_acceleration.append(
            dcm_reactive_stepper.right_foot_acceleration.copy()
        )
        plt_left_foot_position.append(
            dcm_reactive_stepper.left_foot_position.copy()
        )
        plt_left_foot_velocity.append(
            dcm_reactive_stepper.left_foot_velocity.copy()
        )
        plt_left_foot_acceleration.append(
            dcm_reactive_stepper.left_foot_acceleration.copy()
        )
        plt_time_from_last_step_touchdown.append(
            dcm_reactive_stepper.time_from_last_step_touchdown
        )
        plt_duration_before_step_landing.append(
            dcm_reactive_stepper.duration_before_step_landing
        )
        plt_support_foot.append(support_foot.copy())
        plt_swing_foot.append(swing_foot.copy())
        plt_dcm_local.append(x_com + xd_com / omega)
        plt_is_left_leg_in_contact.append(
            dcm_reactive_stepper.is_left_leg_in_contact
        )
        if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
            plt_step_time.append(i)
        time += 0.001

    plt.figure("y")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
    plt.plot(plt_time, np.array(plt_x_com)[:, 1], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[:, 1], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
    # plt.plot(
    #     plt_time, plt_is_left_leg_in_contact, label="is_left_leg_in_contact"
    # )
    plt.plot(plt_time, np.array(plt_support_foot)[:, 1], label="support_foot")
    plt.plot(plt_time, np.array(plt_swing_foot)[:, 1], label="swing_foot")
    for time in plt_step_time:
        plt.axvline(time / 1000)
    plt.legend()
    #
    # plt.figure("x")
    # # plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="right")
    # # plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="left")
    # plt.plot(plt_time, np.array(plt_x_com)[:, 0], label="com")
    # plt.plot(plt_time, np.array(plt_xd_com)[:, 0], label="xd_com")
    # plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    # plt.plot(plt_time, np.array(plt_support_foot)[:, 0], label="support_foot")
    # plt.plot(plt_time, np.array(plt_swing_foot)[:, 0], label="swing_foot")
    # for time in plt_step_time:
    #     plt.axvline(time / 10000)
    # plt.legend()

    plt.figure("z")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:, 2], label="right")
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:, 2], label="left")
    plt.plot(plt_time, np.array(plt_x_com)[:, 2], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[:, 2], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 2], label="dcm_local")
    plt.plot(plt_time, np.array(plt_support_foot)[:, 2], label="support_foot")
    plt.plot(plt_time, np.array(plt_swing_foot)[:, 2], label="swing_foot")
    for time in plt_step_time:
        plt.axvline(time / 1000)
    plt.legend()

    plt.show()
