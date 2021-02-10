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
    sim = LipmSimpulator(0.2)

    dcm_reactive_stepper = DcmReactiveStepper(
        is_left_leg_in_contact=True,
        l_min=-0.5,
        l_max=0.5,
        w_min=-0.5,
        w_max=0.5,
        t_min=0.1,
        t_max=0.2,
        l_p=0.1235 * 2,
        com_height=0.26487417,
        weight=[1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000],
        mid_air_foot_height=0.05,
        control_period=0.001,
        previous_support_foot=[[0.0], [-0.075], [0.0]],
        current_support_foot=[[0.0], [0.075], [0.0]],
    )
    dcm_reactive_stepper.set_end_eff_traj_costs(1e1, 1e1, 1e0, 1e-6)
    v_des = np.zeros((3, 1))
    v_des[:] = [[0.0], [0.0], [0.0]]
    dcm_reactive_stepper.set_des_com_vel(v_des)
    x_com = np.zeros((3, 1))
    x_com[:] = [[0.0], [0.0], [0.26487417]]
    xd_com = np.zeros((3, 1))
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
    plt_step_time = []
    plt_dcm_local = []
    plt_is_left_leg_in_contact = []
    t = 0
    omega = np.sqrt(9.8 / 0.2)
    for i in range(1000):
        time += 0.001
        if dcm_reactive_stepper.is_left_leg_in_contact:
            support_foot = dcm_reactive_stepper.left_foot_position.copy()
            swing_foot = dcm_reactive_stepper.right_foot_position.copy()
        else:
            support_foot = dcm_reactive_stepper.right_foot_position.copy()
            swing_foot = dcm_reactive_stepper.left_foot_position.copy()
        dcm_reactive_stepper.run(
            time, swing_foot, support_foot, x_com, xd_com, 0, [0, 0]
        )
        if i == 760:
            xd_com[1] = xd_com[1] - 0.2
            t = dcm_reactive_stepper.time_from_last_step_touchdown
        if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
            t = 0
        x_com, xd_com, _ = sim.step(
            dcm_reactive_stepper.time_from_last_step_touchdown - t,
            dcm_reactive_stepper.current_support_foot,
            x_com,
            xd_com,
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
        plt_current_support_foot.append(
            dcm_reactive_stepper.current_support_foot.copy()
        )
        plt_support_foot.append(support_foot)
        plt_dcm_local.append(x_com + xd_com / omega)
        plt_is_left_leg_in_contact.append(
            dcm_reactive_stepper.is_left_leg_in_contact
        )
        if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
            plt_step_time.append(i)

    plt.figure("y")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
    plt.plot(plt_time, np.array(plt_x_com)[:, 1], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[:, 1], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
    plt.plot(
        plt_time, plt_is_left_leg_in_contact, label="is_left_leg_in_contact"
    )
    plt.plot(plt_time, np.array(plt_support_foot)[:, 1], label="support_foot")
    for time in plt_step_time:
        plt.axvline(time / 1000)
    plt.legend()
    print(plt_step_time)

    plt.figure("x")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="right")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="left")
    plt.plot(plt_time, np.array(plt_x_com)[:, 0], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[:, 0], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    for time in plt_step_time:
        plt.axvline(time / 1000)
    plt.legend()

    plt.show()
