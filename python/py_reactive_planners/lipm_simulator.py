## @namespace reactive_planners.LipmSimulator
"""
    @author Elham Daneshmand (exledh@gmail.com)
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np


class LipmSimpulator:
    def __init__(self, com_height):
        self.omega = np.sqrt(9.8 / com_height)
        self.last_x_com = np.zeros((3, 1))
        self.last_x_com[:] = [[0], [0], [com_height]]
        self.last_xd_com = np.zeros((3, 1))
        self.x_com = np.zeros((3, 1))
        self.xd_com = np.zeros((3, 1))
        self.xdd_com = np.zeros((3, 1))
        self.u_current_step = np.zeros((3, 1))
        self.xdd_com = np.zeros((3, 1))

    def step(self, time, u_current_step, previous_x_com, previous_xd_com):
        if time == 0.0:
            self.last_x_com[:] = previous_x_com
            self.last_xd_com[:] = previous_xd_com
        self.u_current_step[:] = u_current_step

        self.x_com[:] = (
            (
                0.5
                * (
                    self.last_x_com
                    - self.u_current_step
                    + (self.last_xd_com / self.omega)
                )
                * pow(np.e, (self.omega * time))
            )
            + (
                0.5
                * (
                    self.last_x_com
                    - self.u_current_step
                    - (self.last_xd_com / self.omega)
                )
                * pow(np.e, (-self.omega * time))
            )
            + self.u_current_step
        )
        self.xd_com[:] = (
            self.omega
            * 0.5
            * (
                self.last_x_com
                - self.u_current_step
                + (self.last_xd_com / self.omega)
            )
            * pow(np.e, (self.omega * time))
        ) - (
            self.omega
            * 0.5
            * (
                self.last_x_com
                - self.u_current_step
                - (self.last_xd_com / self.omega)
            )
            * pow(np.e, (-self.omega * time))
        )
        self.xdd_com[:] = (self.omega ** 2) * (
            self.last_x_com - self.u_current_step
        )

        self.x_com[2] = self.last_x_com[2]
        self.xd_com[2] = 0
        self.xdd_com[2] = 0
        return self.x_com, self.xd_com, self.xdd_com
