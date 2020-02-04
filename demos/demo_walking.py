#!/usr/bin/env python

""" @namespace Demos of the reactive_planners.dcm_reactive_planner.
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""

from matplotlib import pyplot as plt
from py_reactive_planners.dcm_reactive_stepper import DcmReactiveStepper
from py_reactive_planners.lipm_simulator import LipmSimpulator
import numpy as np

if __name__ == "__main__":
    sim = LipmSimpulator(.2)

    dcm_reactive_stepper = DcmReactiveStepper(is_left_leg_in_contact=True, l_min=-0.5, l_max=0.5, w_min=-0.5, w_max=0.5, t_min=0.1,
                             t_max=0.2, l_p=0.1235 * 2, com_height=0.26487417,
                             weight=[1, 1, 5, 100, 100, 100, 100, 100, 100], mid_air_foot_height=.05,
                            control_period= 0.001)
    u_current_step = np.zeros((3, 1)); u_current_step[:] = [[.0], [0.1235], [.0]]
    v_des = np.zeros((3, 1))
    x_com = np.zeros((3, 1)); x_com[:] = [[.0], [.0], [.2]]
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
    for i in range(1000):
        time += 0.001
        dcm_reactive_stepper.run(time, dcm_reactive_stepper.flying_foot_position, x_com, xd_com, 0)
        x_com, xd_com , _ = sim.step(dcm_reactive_stepper.time_from_last_step_touchdown,
                                     dcm_reactive_stepper.current_support_foot, x_com, xd_com)
        plt_time.append(time)
        plt_x_com.append(x_com.copy())
        plt_xd_com.append(xd_com.copy())
        plt_right_foot_position.append(dcm_reactive_stepper.right_foot_position.copy())
        plt_right_foot_velocity.append(dcm_reactive_stepper.right_foot_velocity.copy())
        plt_right_foot_acceleration.append(dcm_reactive_stepper.right_foot_acceleration.copy())
        plt_left_foot_position.append(dcm_reactive_stepper.left_foot_position.copy())
        plt_left_foot_velocity.append(dcm_reactive_stepper.left_foot_velocity.copy())
        plt_left_foot_acceleration.append(dcm_reactive_stepper.left_foot_acceleration.copy())
        plt_time_from_last_step_touchdown.append(dcm_reactive_stepper.time_from_last_step_touchdown)
        plt_duration_before_step_landing.append(dcm_reactive_stepper.duration_before_step_landing)
        plt_current_support_foot.append(dcm_reactive_stepper.current_support_foot.copy())

    print(dcm_reactive_stepper.time_from_last_step_touchdown)
    plt.figure("com")
    plt.plot(plt_time, np.array(plt_x_com)[:,0])
    plt.plot(plt_time, np.array(plt_x_com)[:,1])
    plt.plot(plt_time, np.array(plt_x_com)[:,2])

    plt.figure("xd")
    plt.plot(plt_time, np.array(plt_xd_com)[:,0])
    plt.plot(plt_time, np.array(plt_xd_com)[:,1])
    plt.plot(plt_time, np.array(plt_xd_com)[:,2])
    
    plt.figure("right_foot_pos")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:,0])
    plt.plot(plt_time, np.array(plt_right_foot_position)[:,1])
    plt.plot(plt_time, np.array(plt_right_foot_position)[:,2])

    plt.figure("left_foot_pos")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:,0])
    plt.plot(plt_time, np.array(plt_left_foot_position)[:,1])
    plt.plot(plt_time, np.array(plt_left_foot_position)[:,2])


    plt.figure("foot_pos_z")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:,2])
    plt.plot(plt_time, np.array(plt_left_foot_position)[:,2])

    plt.figure("last_step_touchdown")
    plt.plot(plt_time, np.array(plt_time_from_last_step_touchdown)[:])
    plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:])

    plt.figure("support_foot")
    plt.plot(plt_time, np.array(plt_current_support_foot)[:,0])
    plt.plot(plt_time, np.array(plt_current_support_foot)[:,1])
    plt.plot(plt_time, np.array(plt_current_support_foot)[:,2])
    plt.show()