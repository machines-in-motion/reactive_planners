## @namespace reactive_planners.step_adjustment
""" This module find the step  adjustment for biped robots.
    @author Elham Daneshmand (exledh@gmail.com)
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
from matplotlib import pyplot as plt
from reactive_planners_cpp import DcmVrpPlanner


class StepAdjustment:
    def __init__(
        self,
        is_left_leg_in_contact,
        l_min,
        l_max,
        w_min,
        w_max,
        t_min,
        t_max,
        l_p,
        com_height,
        weight,
    ):
        self.kp = np.array(6 * [200.0])
        self.kd = 6 * [5.0]
        self.com_height = com_height
        self.dcm_vrp_planner = DcmVrpPlanner()
        self.dcm_vrp_planner.initialize(
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            self.com_height,
            weight,
        )
        self.is_left_leg_in_contact = is_left_leg_in_contact
        self.omega = np.sqrt(9.8 / self.com_height)

        # for plotting
        self.x_com_history = []
        self.xd_com_history = []
        self.t_nom = []
        self.tau_nom = []
        self.l_nom = []
        self.w_nom = []
        self.bx_nom = []
        self.by_nom = []
        self.dcm_local = []
        self.current_step_location_local = []
        self.dcm_nominal = []
        self.next_step_location = []
        self.current_step_location = []
        self.duration_before_step_landing = []
        self.xPhHistory = []
        self.xdPhHistory = []
        self.last_x_com_history = []
        self.last_xd_com_history = []
        self.time_step = []

    def simulator(self, u_current_step, v_des, x_com, xd_com):
        t = 0
        last_x_com = x_com
        last_xd_com = xd_com

        for i in range(1000):
            print(i)
            t += 0.001
            if (
                i is not 0
                and t > self.dcm_vrp_planner.get_duration_before_step_landing()
            ):
                self.time_step.append(i)
                last_x_com = x_com
                last_xd_com = xd_com
                u_current_step = (
                    self.dcm_vrp_planner.get_next_step_location().copy()
                )
                t = 0.0
                self.is_left_leg_in_contact = not self.is_left_leg_in_contact

            # Analytical solution of x and xd (just for testing)
            xddPh = (self.omega ** 2) * (last_x_com - u_current_step)
            xPh = 0.5 * xddPh * (0.001 ** 2) + xd_com * 0.001 + x_com
            xdPh = xddPh * 0.001 + xd_com

            # Numerical solution of x and xd
            x_com = (
                (
                    0.5
                    * (
                        last_x_com
                        - u_current_step
                        + (last_xd_com / self.omega)
                    )
                    * pow(np.e, (self.omega * t))
                )
                + (
                    0.5
                    * (
                        last_x_com
                        - u_current_step
                        - (last_xd_com / self.omega)
                    )
                    * pow(np.e, (-self.omega * t))
                )
                + u_current_step
            )
            xd_com = (
                self.omega
                * 0.5
                * (last_x_com - u_current_step + (last_xd_com / self.omega))
                * pow(np.e, (self.omega * t))
            ) - (
                self.omega
                * 0.5
                * (last_x_com - u_current_step - (last_xd_com / self.omega))
                * pow(np.e, (-self.omega * t))
            )

            self.update_planner(
                u_current_step, t * 1.0, v_des, x_com, xd_com, 0
            )

            self.xPhHistory.append(xPh.copy())
            self.xdPhHistory.append(xdPh.copy())
            self.x_com_history.append(x_com.copy())
            self.xd_com_history.append(xd_com.copy())
            self.t_nom.append(self.dcm_vrp_planner.get_t_nom())
            self.tau_nom.append(self.dcm_vrp_planner.get_tau_nom())
            self.l_nom.append(self.dcm_vrp_planner.get_l_nom())
            self.w_nom.append(self.dcm_vrp_planner.get_w_nom())
            self.bx_nom.append(self.dcm_vrp_planner.get_bx_nom())
            self.by_nom.append(self.dcm_vrp_planner.get_by_nom())
            self.dcm_local.append(self.dcm_vrp_planner.get_dcm_local().copy())
            self.current_step_location_local.append(
                self.dcm_vrp_planner.get_current_step_location_local().copy()
            )
            self.dcm_nominal.append(
                self.dcm_vrp_planner.get_dcm_nominal().copy()
            )
            self.next_step_location.append(
                self.dcm_vrp_planner.get_next_step_location().copy()
            )
            self.current_step_location.append(u_current_step.copy())
            self.duration_before_step_landing.append(
                self.dcm_vrp_planner.get_duration_before_step_landing()
            )
            self.last_x_com_history.append(last_x_com)
            self.last_xd_com_history.append(last_xd_com)

        self.plot()

    def update_planner(self, u_current_step, t, v_des, x_com, xd_com, yaw):
        self.dcm_vrp_planner.update(
            u_current_step,
            t,
            self.is_left_leg_in_contact,
            v_des,
            x_com,
            xd_com,
            yaw,
        )
        self.dcm_vrp_planner.solve()

    def plot(self):
        fig1, ax1 = plt.subplots(1, 1)
        ax1.plot(np.array(self.x_com_history)[:, 1], label="x_com_history")
        ax1.plot(self.by_nom, label="by_nom")
        ax1.plot(np.array(self.dcm_local)[:, 1], label="dcm_local")
        ax1.plot(
            np.array(self.current_step_location_local)[:, 1],
            label="current_step_location_local",
        )
        ax1.plot(
            np.array(self.next_step_location)[:, 1], label="next_step_location"
        )
        ax1.plot(
            np.array(self.current_step_location)[:, 1],
            label="current_step_location",
        )
        ax1.plot(
            self.duration_before_step_landing,
            label="duration_before_step_landing",
        )
        for t in self.time_step:
            ax1.axvline(t)
        ax1.grid()
        ax1.legend()

        fig1, ax1 = plt.subplots(1, 1)
        ax1.plot(np.array(self.x_com_history)[:, 1], label="x_com_history")
        ax1.plot(np.array(self.xd_com_history)[:, 1], label="xd_com_history")
        ax1.plot(np.array(self.xPhHistory)[:, 1], label="x")
        ax1.plot(np.array(self.xdPhHistory)[:, 1], label="xd")
        # ax1.plot(np.array(self.last_x_com_history)[:, 1], label = "last_X")
        # ax1.plot(np.array(self.last_xd_com_history)[:, 1], label = "last_xd")
        ax1.grid()
        ax1.legend()

        fig1, ax1 = plt.subplots(1, 1)
        ax1.plot(self.t_nom, label="t_nom")
        # ax1.plot(self.tau_nom, label = "tau_nom")
        ax1.plot(self.l_nom, label="l_nom")
        ax1.plot(self.w_nom, label="w_nom")
        ax1.plot(self.bx_nom, label="bx_nom")
        ax1.plot(self.by_nom, label="by_nom")
        ax1.grid()
        ax1.legend()
        plt.show()

    def zero_cnt_gain(self, kp, cnt_array):
        gain = np.array(kp).copy()
        for i, v in enumerate(cnt_array):
            if v == 1:
                gain[3 * i : 3 * (i + 1)] = 0.0
        return gain


if __name__ == "__main__":
    planner = StepAdjustment(
        is_left_leg_in_contact=True,
        l_min=-0.5,
        l_max=0.5,
        w_min=-0.5,
        w_max=0.5,
        t_min=0.1,
        t_max=0.2,
        l_p=0.1235 * 2,
        com_height=0.26487417,
        weight=[1, 1, 5, 100, 100, 100, 100, 100, 100],
    )
    planner.simulator(
        u_current_step=np.array([0.0, 0.1235, 0.0]),
        v_des=np.array([0.0, 0.0, 0.0]),
        x_com=np.array([0.0, 0.0, 0.2]),
        xd_com=np.array([0.0, 0.0, 0.0]),
    )
