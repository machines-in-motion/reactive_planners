## @namespace reactive_planners.LipmSimulator
"""
    @author Elham Daneshmand (exledh@gmail.com)
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np


class LipmSimpulator:
    def __init__(self, x_com, xd_com, omega,):
        self.omega = omega
        self.last_x_com = x_com.copy()
        self.last_xd_com = xd_com.copy()
        self.x_com = x_com.copy()
        self.xd_com = xd_com.copy()
        self.xdd_com = np.zeros((3, 1))
        self.u_current_step = np.zeros((3, 1))
        self.xdd_com = np.zeros((3, 1))
        self.first = True
        self.second = True
        self.control_loop = 0.001

    def step(self, time, t_s, u_current_step, previous_x_com, previous_xd_com, contact):
        if contact is 0:
            return self.floating_base(time, t_s, previous_x_com, previous_xd_com)
        else:
            return self.LIPM(time, u_current_step, previous_x_com, previous_xd_com)

    def floating_base(self, time, t_s, previous_x_com, previous_xd_com):
        if self.xdd_com[2, 0] != 0:
            # if self.second:
            #     self.last_xd_com[2, 0] = -self.last_xd_com[2, 0]
            #     self.second = False
            # else:
            self.last_x_com[:] = previous_x_com
            self.last_xd_com[:] = previous_xd_com
        self.x_com[:2] = self.xd_com[:2] * (time - t_s + self.control_loop) + self.last_x_com[:2]
        self.x_com[2] = -0.5 * 9.81 * (time - t_s + self.control_loop) * (time - t_s + self.control_loop) + \
                        self.last_xd_com[2] * (time - t_s + self.control_loop) + self.last_x_com[2]
        self.xd_com[2] = -9.81 * (time - t_s + self.control_loop) + self.last_xd_com[2]
        self.xdd_com[:] = [[0], [0], [0]]
        return self.x_com, self.xd_com, self.xdd_com

    def LIPM(self, time, u_current_step, previous_x_com, previous_xd_com):
        time += self.control_loop
        if self.xdd_com[2, 0] == 0:
            self.last_x_com[:] = previous_x_com
            self.last_xd_com[:] = previous_xd_com
            self.u_current_step[:] = u_current_step
            self.u_current_step[2, 0] += 9.81 / (self.omega * self.omega)
            # self.first = False
        self.x_com[:] = (0.5 * (self.last_x_com - self.u_current_step + (self.last_xd_com / self.omega)) * pow(np.e, (self.omega * time))) + \
                        (0.5 * (self.last_x_com - self.u_current_step - (self.last_xd_com / self.omega)) * pow(np.e, (-self.omega * time))) + self.u_current_step
        self.xd_com[:] = (self.omega * 0.5 * (self.last_x_com - self.u_current_step + (self.last_xd_com / self.omega)) * pow(np.e, (self.omega * time))) - \
                         (self.omega * 0.5 * (self.last_x_com - self.u_current_step - (self.last_xd_com / self.omega)) * pow(np.e, (-self.omega * time)))
        self.xdd_com[:] = (self.omega ** 2) * (self.last_x_com - self.u_current_step)

        return self.x_com, self.xd_com, self.xdd_com
