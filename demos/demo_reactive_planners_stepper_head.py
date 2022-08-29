#!/usr/bin/env python

""" @namespace Demos of the reactive_planners.stepper_head controller.
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""

# Python 3 compatibility, has to be called just after the hashbang.
from __future__ import print_function, division

import numpy as np

# Import the class to demo from.
from reactive_planners_cpp import StepperHead


def log_traj(data):
    import csv

    with open("/tmp/demo_stepper_head_py.dat", "w") as csvfile:
        spamwriter = csv.writer(
            csvfile, delimiter="\t", quotechar="|", quoting=csv.QUOTE_MINIMAL
        )
        for row in data:
            spamwriter.writerow(row)


if __name__ == "__main__":

    #
    # Parameters.
    #
    time = 0.0
    dt = 0.001
    duration_before_step_landing = 0.1
    next_support_foot = [0, 0, 0]

    #
    # Initialization
    #
    # Create the stepper head object
    stepper_head = StepperHead()
    # Collect the data.
    data = []

    #
    # Compute the trajectory.
    #
    data += [
        [
            time,
            stepper_head.get_time_from_last_step_touchdown(),
            stepper_head.get_is_left_leg_in_contact(),
        ]
        + stepper_head.get_previous_support_location().tolist()
        + stepper_head.get_current_support_location().tolist()
    ]

    for i in range(10000):
        stepper_head.run(duration_before_step_landing, next_support_foot, time)

        next_support_foot = np.array([0.5 * time, time, 2 * time])
        time += dt

        data += [
            [
                time,
                stepper_head.get_time_from_last_step_touchdown(),
                stepper_head.get_is_left_leg_in_contact(),
            ]
            + stepper_head.get_previous_support_location().tolist()
            + stepper_head.get_current_support_location().tolist()
        ]

    log_traj(data)
