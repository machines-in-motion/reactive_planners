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
from reactive_planners_cpp import EndEffectorTrajectory3D

if __name__ == "__main__":

    end_eff_traj3d = EndEffectorTrajectory3D()

    end_eff_traj3d.set_mid_air_height(0.05)

    previous_support_location = np.zeros((3, 1))
    foot_position = np.zeros((3, 1))
    foot_velocity = np.zeros((3, 1))
    foot_acceleration = np.zeros((3, 1))
    next_support_location = np.zeros((3, 1))

    start_time = 0.0
    end_time = 1.0
    control_period = 0.001

    plt_foot_position = []

    for i in range(int(1 / control_period)):
        current_time = i * control_period

        end_eff_traj3d.compute(
            previous_support_location,
            foot_position,
            foot_velocity,
            foot_acceleration,
            next_support_location,
            start_time,
            current_time,
            end_time,
        )

        end_eff_traj3d.get_next_state(
            current_time + control_period,
            foot_position,
            foot_velocity,
            foot_acceleration,
        )

        plt_foot_position += [foot_position.copy()]

    plt.plot(np.array(plt_foot_position)[:, 2])
    plt.show()
