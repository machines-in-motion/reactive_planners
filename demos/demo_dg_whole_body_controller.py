"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date:   Feb 16, 2021
"""

import numpy as np
np.set_printoptions(suppress=True, precision=3)

from robot_properties_solo.config import Solo12Config
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot

from mim_control.dynamic_graph.wbc_graph import WholeBodyController

if __name__ == "__main__":
    pin_robot = Solo12Config.buildRobotWrapper()

    qp_penalty_weights = np.array([5e5, 5e5, 5e5, 1e6, 1e6, 1e6])

    ###
    # Create the whole body controller.
    wbc = WholeBodyController('test_wbc', pin_robot, Solo12Config.end_effector_names, 0.2, qp_penalty_weights)

    ###
    # Specify gains for the controller.
    x_des = np.array([
        0.195,  0.147, 0.015,
        0.195, -0.147, 0.015,
        -0.195,  0.147, 0.015,
        -0.195, -0.147, 0.015
    ]).reshape(4, 3)

    # For the centroidal controllers.
    wbc.kc_sin.value = np.array([100., 100., 100.])
    wbc.dc_sin.value = np.array([15., 15., 15.])
    wbc.kb_sin.value = np.array([25., 50., 25.])
    wbc.db_sin.value = np.array([10., 10., 10.])

    wbc.des_com_pos_sin.value = np.array([0., 0., 0.20])
    wbc.des_com_vel_sin.value = np.zeros(3)
    wbc.des_ori_pos_sin.value = np.array([0., 0., 0., 1.])
    wbc.des_ori_vel_sin.value = np.zeros(3)

    wbc.cnt_array_sin.value = np.array([1., 1., 1., 1.])

    # Impedance controllers.
    for i, imp in enumerate(wbc.imps):
        imp.gain_proportional_sin.value = np.array([50., 50., 50., 0., 0., 0.])
        imp.gain_derivative_sin.value = np.array([0.7, 0.7, 0.7, 0., 0., 0.])
        imp.desired_end_frame_placement_sin.value = np.hstack([x_des[i], np.zeros(4)])
        imp.desired_end_frame_velocity_sin.value = np.zeros(6)
        imp.gain_feed_forward_force_sin.value = 1.0

    wbc.w_com_ff_sin.value = np.array([0., 0., 9.81 * 2.5, 0., 0., 0.])

    ###
    # Create the simulated robot
    robot = get_solo12_robot()

    # Change the position of the robot.
    q0 = Solo12Config.q0
    q0[0] = 0.
    robot.reset_state(q0, Solo12Config.v0)

    # Plug the simulated robot to the controller.
    base_signals = robot.base_signals()
    wbc.plug(robot, base_signals[0], base_signals[1])

    # Simulate for 4 seconds.
    robot.run(4000, sleep=True)
