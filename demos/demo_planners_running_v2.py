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
from math import exp

if __name__ == "__main__":
    x_com = np.zeros((3, 1))
    x_com[:] = [[0.05], [0.0622], [0.3]]
    xd_com = np.zeros((3, 1))
    xd_com[:] = [[0.0], [.595], [-0.680718500274]]
    swing_foot = [[0.0], [-0.1235], [0.0]]
    support_foot = [[0.0], [0.1235], [0.0]]
    omega = 10.18
    t_s = 0.1
    l_min = -0.1
    l_max = 0.1
    w_min = -0.1
    w_max = 0.1
    t_min = 0.05
    t_max = 0.15
    l_p = 0.1235
    u_y_nom = l_p
    u_x_nom = 0
    t_f_nom_ = 0.30182 - t_s
    is_left_leg_in_contact = True
    dcm_reactive_stepper = DcmReactiveStepper(
        is_left_leg_in_contact=True,
        l_min=l_min,
        l_max=l_max,
        w_min=w_min,
        w_max=w_max,
        t_min=t_min,
        t_max=t_max,
        l_p=l_p,
        com_height=x_com[2][0],
        weight=[1, 1, 5, 1000, 1000, 5, 100000, 100000, 100000, 100000],
        mid_air_foot_height=0.05,
        control_period=0.001,
        previous_support_foot=swing_foot,
        current_support_foot=support_foot,
    )
    v_des = np.zeros((3, 1))
    v_des[:] = [[0.0], [0.0], [0.0]]
    dcm_reactive_stepper.set_des_com_vel(v_des)
    time = 0
    # time_from_last_step_touchdown = 0
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
    by_nom_ = 0
    sim = LipmSimpulator(x_com, xd_com, omega)

    for i in range(3265):
        dcm_reactive_stepper.run(
            time, swing_foot, support_foot, x_com, xd_com, 0
        )
        # if i == 760:
        #     xd_com[1] = xd_com[1] - 0.2
        #     t = dcm_reactive_stepper.time_from_last_step_touchdown
        # if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
        #     t = 0
        # swing_foot = init_next_step_position()
        if dcm_reactive_stepper.time_from_last_step_touchdown - t <= dcm_reactive_stepper.duration_before_step_landing - 0.00001:#Lhum TODO update t_s
            contact = [dcm_reactive_stepper.is_left_leg_in_contact,
                       not dcm_reactive_stepper.is_left_leg_in_contact]
        else:
            contact = [0, 0]

        if dcm_reactive_stepper.time_from_last_step_touchdown == 0 and i > 2:
            support_foot[0] = swing_foot[0]
            support_foot[1] = swing_foot[1]
            # support_foot[1] = [(x_com[1] + (xd_com[1] / omega) - pow(-1, is_left_leg_in_contact) * (u_y_nom / (1 + pow(np.e, omega * t_s))))[0]]
            # support_foot[0] = [(x_com[0] + (xd_com[0] / omega) - (u_x_nom / (1 + pow(np.e, omega * t_s))))[0]]
            support_foot[2] = [0]

        x_com, xd_com, _ = sim.step(
            dcm_reactive_stepper.time_from_last_step_touchdown - t,
            dcm_reactive_stepper.duration_before_step_landing,
            support_foot,
            x_com,
            xd_com,
            contact[0] + contact[1]
            )

        swing_foot = dcm_reactive_stepper.swing_foot
        swing_foot = [[swing_foot[0]], [swing_foot[1]], [swing_foot[2]]]
        plt_time.append(time)
        plt_x_com.append(x_com.copy())
        plt_xd_com.append(xd_com.copy())
        plt_support_foot.append(support_foot.copy())
        plt_swing_foot.append(swing_foot.copy())
        plt_dcm_local.append(x_com + xd_com / omega)
        plt_is_left_leg_in_contact.append(dcm_reactive_stepper.is_left_leg_in_contact)
        time += 0.001
        # time_from_last_step_touchdown += 0.001
        # if time_from_last_step_touchdown > t_s and xd_com[2] <= 0 and x_com[2] + (xd_com[2] / omega) <= 0.2:
        #     print(time_from_last_step_touchdown)
        #     time_from_last_step_touchdown = 0
        #     is_left_leg_in_contact = not is_left_leg_in_contact


    FIGSIZE = 3.7
    FONT_SIZE = 8
    FONT_WEIGHT = "normal"
    # set the parameters
    font = {'family' : 'normal',
            'weight' : FONT_WEIGHT,
            'size'   : FONT_SIZE}
    plt.rc('font', **font)
    FIGURE_SIZE = ( FIGSIZE , FIGSIZE * 9.0/16.0)

    plt.figure("x")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="right")
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="left")
    plt.plot(plt_time, np.array(plt_x_com)[:, 0], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[:, 0], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    plt.plot(plt_time, np.array(plt_support_foot)[:, 0], label="support_foot")
    plt.plot(plt_time, np.array(plt_swing_foot)[:, 0], label="swing_foot")
    for time in plt_step_time:
        plt.axvline(time / 10000)
    plt.legend()

    fig, ax1 = plt.subplots(2, 1, sharex=True)
    # ax1[0].figure("y")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
    ax1[0].plot(plt_time, np.array(plt_x_com)[:, 1], label="CoM")
    ax1[0].plot(plt_time, np.array(plt_xd_com)[:, 1], label="CoM velocity")
    ax1[0].plot(plt_time, np.array(plt_dcm_local)[:, 1], label="DCM")
    # plt.plot(
    #     plt_time, plt_is_left_leg_in_contact, label="is_left_leg_in_contact"
    # )
    # ax1[0].plot(plt_time, np.array(plt_support_foot)[:, 1], label="support_foot")
    # ax1[0].plot(plt_time, np.array(plt_swing_foot)[:, 1], label="swing_foot")
    # for time in plt_step_time:
    #     ax1[0].axvline(time / 1000)
    # ax1[0].legend()
    #

    # plt.figure("z")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:, 2], label="right")
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:, 2], label="left")
    ax1[1].plot(plt_time, np.array(plt_x_com)[:, 2], label="CoM")
    ax1[1].plot(plt_time, np.array(plt_xd_com)[:, 2], label="CoM velocity")
    ax1[1].plot(plt_time, np.array(plt_dcm_local)[:, 2], label="DCM")
    # ax1[1].plot(plt_time, np.array(plt_support_foot)[:, 2], label="support_foot")
    # ax1[1].plot(plt_time, np.array(plt_swing_foot)[:, 2], label="swing_foot")
    # for time in plt_step_time:
    #     ax1[1].axvline(time / 1000)
    # ax1[1].legend()


    # ax1[0].set_ylabel("Position Y [m]")
    # ax1[1].set_ylabel("Position Z [m]")
    # ax1[1].set_xlabel("Time [ms]")
    # plt.tight_layout()
    # plt.savefig("z" + ".pdf")

    plt.show()