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

def init_next_step_position():
    if is_left_leg_in_contact:
        contact_switcher = 2.0
    else:
        contact_switcher = 1.0
    fabs_l_min = min(abs(l_min), abs(l_max))
    if l_min < 0 and 0 < l_max:
        fabs_l_min = 0
    fabs_l_max = max(abs(l_min), abs(l_max))
    fabs_w_min = min(abs(w_min), abs(w_max))
    if w_min < 0 and w_max > 0:
        fabs_w_min = 0
    fabs_w_max = max(abs(w_min), abs(w_max))
    if abs(v_des[0]) < 1e-5 or abs(v_des[1]) < 1e-5:
        t_lower_bound = t_min
        t_upper_bound = t_max
    else:
        t_lower_bound = max(fabs_l_min / abs(v_des[0]), max(fabs_w_min / abs(v_des[1]), t_min))
        t_upper_bound = min(fabs_l_max / abs(v_des[0]), min(fabs_w_max / abs(v_des[1]), t_max))

    t_nom_ = (t_lower_bound + t_upper_bound) * 0.5
    print("T", t_nom_)
    t_f_nom_ = 0.2
    tau_nom_ = exp(omega * t_nom_)
    l_nom_ = v_des[0] * t_nom_
    if is_left_leg_in_contact:
        w_nom_ = v_des[1] * t_nom_ - l_p
    else:
        w_nom_ = v_des[1] * t_nom_ + l_p
    print("X test", support_foot[0][0])
    v_x = omega / 2 * (x_com[0].item() - support_foot[0][0] + xd_com[0].item() / omega) * tau_nom_ - omega / 2 * (x_com[0].item() - support_foot[0][0] - xd_com[0].item() / omega) / tau_nom_
    v_y = omega / 2 * (x_com[1].item() - support_foot[1][0] + xd_com[1].item() / omega) * tau_nom_ - omega / 2 * (x_com[1].item() - support_foot[1][0] - xd_com[1].item() / omega) / tau_nom_
    bx_nom_ = (l_nom_ - v_x * t_f_nom_) / (tau_nom_ - 1)
    by_nom_ = (pow(-1, contact_switcher) * l_p / (1 + tau_nom_)) - ((v_des[1] * t_nom_ - v_y * t_f_nom_) / (1 - tau_nom_))
    print("!  ", by_nom_)

    b = [bx_nom_, by_nom_, 0.]
    print("B", b)
    if is_left_leg_in_contact:
        kesay_nom = [0., 0.0455043286361, 0.]
    else:
        kesay_nom = [0., -0.00346, 0.]
    print("!!!!!!!!!!!!", [[kesay_nom[0] + b[0]], [kesay_nom[1] + b[1]], [kesay_nom[2] + b[2]]])
    return [[kesay_nom[0] - b[0]], [kesay_nom[1] - b[1]], [kesay_nom[2] - b[2]]]

# def next_step_position():
#     if is_left_leg_in_contact:
#         contact_switcher = 2.0
#     else:
#         contact_switcher = 1.0
#     fabs_l_min = min(abs(l_min), abs(l_max))
#     if l_min < 0 and 0 < l_max:
#         fabs_l_min = 0
#     fabs_l_max = max(abs(l_min), abs(l_max))
#     fabs_w_min = min(abs(w_min), abs(w_max))
#     if w_min < 0 and w_max > 0:
#         fabs_w_min = 0
#     fabs_w_max = max(abs(w_min), abs(w_max))
#     if abs(v_des[0]) < 1e-5 or abs(v_des[1]) < 1e-5:
#         t_lower_bound = t_min
#         t_upper_bound = t_max
#     else:
#         t_lower_bound = max(fabs_l_min / abs(v_des[0]), max(fabs_w_min / abs(v_des[1]), t_min))
#         t_upper_bound = min(fabs_l_max / abs(v_des[0]), min(fabs_w_max / abs(v_des[1]), t_max))
#
#     t_nom_ = (t_lower_bound + t_upper_bound) * 0.5
#     print("T", t_nom_)
#     t_f_nom_ = 0.2
#     tau_nom_ = exp(omega * t_nom_)
#     l_nom_ = v_des[0] * t_nom_
#     if is_left_leg_in_contact:
#         w_nom_ = v_des[1] * t_nom_ - l_p
#     else:
#         w_nom_ = v_des[1] * t_nom_ + l_p
#     bx_nom_ = l_nom_ / ((tau_nom_ - 1) * (1 + (pow(omega, 2) / 9.81) * (9.81 / (pow(omega, 2)) - 0.3)))
#     by_nom_ = (pow(-1, contact_switcher) * (l_p / (1 + tau_nom_ + pow(omega, 2) / 9.81 * (tau_nom_ - 1) * ((9.81 / pow(omega, 2)) - 0.3))) - \
#                (v_des[1] * t_nom_ / ((1 - tau_nom_) * (1 + (pow(omega, 2) / 9.81) * (9.81 / (pow(omega, 2)) - 0.3)))))
#     print("!  ", (pow(-1, contact_switcher) * (l_p / (1 + tau_nom_ + pow(omega, 2) / 9.81 * (tau_nom_ - 1) * ((9.81 / pow(omega, 2)) - 0.3))) -
#                   (v_des[1] * t_nom_ / ((1 - tau_nom_) * (1 + (pow(omega, 2) / 9.81) * (9.81 / (pow(omega, 2)) - 0.3))))))
#     b = [[bx_nom_], [by_nom_], [0.]]
#     print(b)
#     kesay_nom = x_com + xd_com / omega
#     return kesay_nom + b

if __name__ == "__main__":
    xd_com = np.zeros((3, 1))
    v_z = (2 / ((np.e ** (10.18 * .1)) + 1) * (0.3 - (9.81 / (10.18 * 10.18))) - 0.3 + (9.81 / (10.18 * 10.18))) * 10.18
    xd_com[:] = [[0], [0], [-0.980718500274]]
    omega = 10.18
    t_s = 0.1
    l_min = -1.5
    l_max = 1.5
    w_min = -1.5
    w_max = 1.5
    t_min = 0.05
    t_max = 0.15
    l_p = 0.1235
    t_f_nom_ = 0.2
    is_left_leg_in_contact = True

    v_des = np.zeros((3, 1))
    v_des[:] = [[0.0], [0.0], [0.0]]
    time = 0
    time_from_last_step_touchdown = 0
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
    x_com = np.zeros((3, 1))
    x_com[:] = [[0], [0.0455043286361], [0.3]]
    sim = LipmSimpulator(x_com, xd_com, omega)
    print(x_com)
    swing_foot = [[0.0], [0.06175], [0.0]]
    support_foot = [[0.0], [-0.06175], [0.0]]

    for i in range(1365):
        if time_from_last_step_touchdown == 0 and i != 1:
            a = support_foot
            support_foot = swing_foot
            swing_foot = a



        # if i == 760:
        #     xd_com[1] = xd_com[1] - 0.2
        #     t = dcm_reactive_stepper.time_from_last_step_touchdown
        # if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
        #     t = 0
        # swing_foot = init_next_step_position()

        if time_from_last_step_touchdown - t <= t_s - 0.00001:
            contact = [is_left_leg_in_contact,
                       not is_left_leg_in_contact]
        else:
            contact = [0, 0]

        print(i)
        print("SF", support_foot)
        x_com, xd_com, _ = sim.step(
            time_from_last_step_touchdown,
            support_foot,
            x_com,
            xd_com,
            contact[0] + contact[1]
            )
        plt_time.append(time)
        plt_x_com.append(x_com.copy())
        plt_xd_com.append(xd_com.copy())
        plt_support_foot.append(support_foot.copy())
        plt_swing_foot.append(swing_foot.copy())
        plt_dcm_local.append(x_com + xd_com / omega)
        plt_is_left_leg_in_contact.append(is_left_leg_in_contact)
        time += 0.001
        time_from_last_step_touchdown += 0.001
        if time_from_last_step_touchdown >= t_s + t_f_nom_:
            time_from_last_step_touchdown = 0
            is_left_leg_in_contact = not is_left_leg_in_contact


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
