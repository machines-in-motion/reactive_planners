## @namespace reactive_planners.LipmSimulator
"""
    @author Elham Daneshmand (exledh@gmail.com)
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np

class LipmSimpulator():
    def __init__(self, com_height):
        self.omega = np.sqrt(9.8 / com_height)
        self.last_x_com = np.array([0,0,com_height])
        self.last_xd_com = np.array([0, 0, 0])

    def step(self, time, u_current_step, previous_x_com, previous_xd_com):
        if time == 0.0:
            self.last_x_com = previous_x_com
            self.last_xd_com = previous_xd_com

        x_com = (0.5 * (self.last_x_com - u_current_step + (self.last_xd_com / self.omega)) * pow(np.e, (self.omega * time))) + \
                (0.5 * (self.last_x_com - u_current_step - (self.last_xd_com / self.omega)) * pow(np.e, (-self.omega * time))) + \
                u_current_step
        xd_com = (self.omega * 0.5 * (self.last_x_com - u_current_step + (self.last_xd_com / self.omega)) *
                  pow(np.e, (self.omega * time))) - \
                 (self.omega * 0.5 * (self.last_x_com - u_current_step - (self.last_xd_com / self.omega)) *
                  pow(np.e, (-self.omega * time)))
        xdd_com = (self.omega ** 2) * (self.last_x_com - u_current_step)

        return x_com, xd_com, xdd_com

