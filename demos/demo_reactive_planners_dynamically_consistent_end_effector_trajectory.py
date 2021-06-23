#!/usr/bin/env python

""" @namespace Demos of the reactive_planners.end_effector_trajectory.
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""

import numpy as np
from matplotlib import pyplot as plt
from reactive_planners_cpp import DynamicallyConsistentEndEffectorTrajectory

if __name__ == "__main__":

    end_eff_traj3d = DynamicallyConsistentEndEffectorTrajectory()

    planner_loop_period = 0.01
    mid_air_height = 0.1
    floor_height = 0.00
    end_eff_traj3d.set_mid_air_height(mid_air_height + floor_height)
    end_eff_traj3d.set_planner_loop(planner_loop_period)

    # front_left_foot_last_support_position_ = 0.197796 0.146787        0 ;
    # front_left_foot_position_ = 0.190135 0.140959 0.283463 ;
    # front_left_foot_next_support_position_ = 0.205785 0.144659        0 ;
    # front_left_foot_velocity_ =  -0.15171 -0.115403   5.61313 ; mid_air_height = 0.1 ; start_time = 0 ; current_time = 0.1 ; end_time = 0.199999 ; 

    previous_support_location = np.array([0.197796, 0.146787   ,     0])
    foot_position = np.array([0.190135, 0.140959, 0.283463])
    foot_velocity = np.array([ -0.15171, -0.115403,   5.61313 ])
    foot_acceleration = np.zeros((3, 1))
    next_support_location = np.array([0.205785, 0.144659,        0])

    start_time = 0.0
    end_time = 0.2
    control_period = 0.001

    forces = np.zeros((int((end_time-start_time)/control_period * 3 + 1000), 1))

    plt_foot_position = []
    plt_forces = []

    duration = int(end_time / control_period / 2)
    for i in range(int(duration/2), duration):
        print(i)
        current_time = i * control_period

        end_eff_traj3d.compute(
            previous_support_location,
            foot_position,
            foot_velocity,
            next_support_location,
            start_time,
            current_time,
            end_time,
            True,
        )
        end_eff_traj3d.get_forces(
            forces,
            foot_position,
            foot_velocity,
            foot_acceleration,
        )

        plt_forces += [forces[:3].copy()]
        plt_foot_position += [foot_position.copy()]

    plt.figure(0)
    plt.plot(np.array(plt_foot_position)[:, 0], label="x")
    plt.plot(np.array(plt_foot_position)[:, 1], label="y")
    plt.plot(np.array(plt_foot_position)[:, 2], label="z")
    plt.legend()
    plt.figure(1)
    plt.plot(np.array(plt_forces)[:, 0], label="fx")
    plt.plot(np.array(plt_forces)[:, 1], label="fy")
    plt.plot(np.array(plt_forces)[:, 2], label="fz")
    plt.legend()
    plt.show()
