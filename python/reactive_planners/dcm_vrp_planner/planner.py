### Author : Avadesh Meduri
### Date : 27/09/2019
### This is the implementation of the paper "walking control based on step timing Adaption" by Majid Et al.


import numpy as np
from reactive_planners.utils.qp_solver import quadprog_solve_qp


class DcmVrpPlanner:
    def __init__(
        self, l_min, l_max, w_min, w_max, t_min, t_max, v_des, l_p, ht
    ):

        """
        Input:
            l_min : minimum step length in the x direction (in the direction of forward motion)
            l_max : maximum step length in the x direction (in the direction of forward motion)
            w_min : minimum step length in the y direction (in the lateral direction)
            w_max : maximum step lenght in the y direction (in the lateratl direction)
            t_min : minimum step time
            t_max : maximum step time
            v_des : desired average velocity in the x and y ([v_x, v_y]) 2d vector
            l_p : default step width
            ht : average desired height of the com above the ground
        """

        assert np.shape(v_des) == (2,)

        self.l_min = l_min
        self.l_max = l_max
        self.w_min = w_min
        self.w_max = w_max
        self.t_min = t_min
        self.t_max = t_max
        self.v_des = v_des
        self.l_p = l_p
        self.ht = ht

        self.omega = np.sqrt(9.81 / self.ht)
        self.bx_max = self.l_max / (
            np.power(np.e, self.omega * self.t_min) - 1
        )
        self.bx_min = self.l_min / (
            np.power(np.e, self.omega * self.t_max) - 1
        )
        self.by_max_out = (
            self.l_p / (1 + np.power(np.e, self.omega * t_min))
        ) + (
            self.w_max - self.w_min * np.power(np.e, self.omega * self.t_min)
        ) / (
            1 - np.power(np.e, 2 * self.omega * self.t_min)
        )

        self.by_max_in = (
            self.l_p / (1 + np.power(np.e, self.omega * t_min))
        ) + (
            self.w_min - self.w_max * np.power(np.e, self.omega * self.t_min)
        ) / (
            1 - np.power(np.e, 2 * self.omega * self.t_min)
        )

    def compute_nominal_step_values(self, n):

        """
        computes nominal step location and step length at each time step
        input :
            n : 1 if left leg and 2 if right le is in contact
        """

        if self.v_des[0] == 0 or self.v_des[1] == 0:
            B_l = self.t_min
            B_u = self.t_max
        else:
            B_l = np.max(
                [
                    self.l_min / abs(self.v_des[0]),
                    self.w_min / abs(self.v_des[1]),
                    self.t_min,
                ]
            )
            B_u = np.min(
                [
                    self.l_max / abs(self.v_des[0]),
                    self.w_max / abs(self.v_des[1]),
                    self.t_max,
                ]
            )

        t_nom = (B_l + B_u) / 2.0
        l_nom = self.v_des[0] * t_nom
        w_nom = self.v_des[1] * t_nom

        bx_nom = l_nom / (np.power(np.e, self.omega * t_nom) - 1)
        by_nom = (
            np.power(-1, n)
            * (self.l_p / (1 + np.power(np.e, self.omega * t_nom)))
        ) - w_nom / (1 - np.power(np.e, self.omega * t_nom))

        return l_nom, w_nom, t_nom, bx_nom, by_nom

    def compute_dcm_current(self, x, xd):
        """
        computes the current location of the dcm
        input :
            x : center of mass location at current time step
            xd : center of mass velocity of current time step
        """
        self.x = x
        return (xd / self.omega) + x

    def compute_alpha(self, xd_com, v_des):
        """
        computes the current value of alpha. Alpha takes a value of 1 if v_des and
        xd_com are zero. When alpha is one t has to be zero to satisfy complementary constraint.
        Otherwise it takes a value of 0.
        """

        if max(v_des) == 0 and np.square(max(xd_com)) < 0.001:
            print("****************************************")
            return 1
        else:
            return 0

    def compute_adapted_step_locations(self, u, t, n, psi_current, alpha, W):
        """
        computes adapted step location after solving QP
        Input:
            u : the location of the previous step (2d vector) [ux, uy]
            t : time elapsed after the previous step has occured
            n : 1 if left leg and 2 if right le is in contact
            psi_current : current dcm location [psi_x, psi_y]
            W : wieght array 5d
        """

        assert np.shape(u) == (2,)
        assert np.shape(psi_current) == (2,)

        l_nom, w_nom, t_nom, bx_nom, by_nom = self.compute_nominal_step_values(
            n
        )
        t_nom = np.power(
            np.e, self.omega * t_nom
        )  ### take exp as T is considered as e^wt in qp

        P = np.identity(6)  ## quadratic cost matrix
        P[0][0], P[1][1], P[2][2], P[3][3], P[4][4], P[5][5] = W
        q = np.array(
            [
                -W[0] * (l_nom),
                -W[1] * (w_nom),
                -W[2] * t_nom,
                -W[3] * bx_nom,
                -W[4] * by_nom,
                0,
            ]
        )  ##

        G = np.matrix(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [-1, 0, 0, 0, 0, 0],
                [0, -1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, -1, 0, 0, 0],
                [0, 0, alpha, 0, 0, 1],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, -1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, -1, 0],
            ]
        )

        h = np.array(
            [
                self.l_max,
                self.w_max,
                -1 * (self.l_min),
                -1 * (self.w_min),
                np.power(np.e, self.omega * self.t_max),
                -1 * np.power(np.e, self.omega * self.t_min),
                -alpha,
                self.bx_max,
                -1 * self.bx_min,
                self.by_max_in,
                -1 * self.by_max_out,
            ]
        )

        tmp = [0.0, 0.0]
        tmp[0] = (psi_current[0] - u[0]) * np.power(np.e, -1 * self.omega * t)
        tmp[1] = (psi_current[1] - u[1]) * np.power(np.e, -1 * self.omega * t)
        A = np.matrix(
            [[1, 0, -1 * tmp[0], 1, 0, 0], [0, 1, -1 * tmp[1], 0, 1, 0]]
        )

        b = np.array([0.0, 0.0])

        P = P.astype(float)
        q = q.astype(float)
        G = G.astype(float)
        h = h.astype(float)
        A = A.astype(float)
        b = b.astype(float)

        x_opt = quadprog_solve_qp(P, q, G, h, A, b)
        t_end = np.log(x_opt[2]) / self.omega

        self._quad_params = [P, q, G, h, A, b]
        print("^^^^^^^")
        print(alpha)
        print(x_opt[4])
        return (x_opt[0] + u[0], x_opt[1] + u[1], t_end, x_opt[3], x_opt[4])

    def compute_adapted_step_locations_gurobi(
        self, u, t, n, psi_current, alpha, W
    ):
        """
        computes adapted step location after solving QP
        Input:
            u : the location of the previous step (2d vector) [ux, uy]
            t : time elapsed after the previous step has occured
            n : 1 if left leg and 2 if right le is in contact
            psi_current : current dcm location [psi_x, psi_y]
            W : wieght array 5d
        """
        # Import only when used.
        # from gurobipy import *

        assert np.shape(u) == (2,)
        assert np.shape(psi_current) == (2,)

        l_nom, w_nom, t_nom, bx_nom, by_nom = self.compute_nominal_step_values(
            n
        )

        t_nom = np.power(
            np.e, self.omega * t_nom
        )  ### take exp as T is considered as e^wt in qp

        # creating model

        m = Model("qp")

        ## creating variables
        ut_x = m.addVar(lb=self.l_min, ub=self.l_max, name="ut_x")
        ut_y = m.addVar(lb=self.w_min, ub=self.w_max, name="ut_y")
        t_step = m.addVar(
            lb=np.power(np.e, self.omega * self.t_min),
            ub=np.power(np.e, self.omega * self.t_max),
            name="t_step",
        )
        b_x = m.addVar(lb=self.bx_min, ub=self.bx_max, name="b_x")
        b_y = m.addVar(lb=self.by_max_out, ub=self.by_max_in, name="b_y")

        ### Cost

        c = (
            (W[0] * (ut_x - l_nom) * (ut_x - l_nom))
            + (W[1] * (ut_y - w_nom) * (ut_y - w_nom))
            + (W[2] * (t_step - t_nom) * (t_step - t_nom))
            + (W[3] * (b_x - bx_nom) * (b_x - bx_nom))
            + (W[4] * (b_y - by_nom) * (b_y - by_nom))
        )

        m.setObjective(c)

        ## constraints
        tmp = [0.0, 0.0]
        tmp[0] = (psi_current[0] - u[0]) * np.power(np.e, -1 * self.omega * t)
        tmp[1] = (psi_current[1] - u[1]) * np.power(np.e, -1 * self.omega * t)

        m.addConstr(ut_x == (tmp[0] * t_step) - b_x, "dyn_constr_x")
        m.addConstr(ut_y == (tmp[1] * t_step) - b_y, "dyn_constr_y")

        m.optimize()

        x_opt = m.getVars()

        t_end = np.log(x_opt[2].x) / self.omega

        return (
            x_opt[0].x + u[0],
            x_opt[1].x + u[1],
            t_end,
            x_opt[3].x,
            x_opt[4].x,
        )

    def compute_which_end_effector(
        self, v_des, u_current, u_next, dcm_t, n_current, alpha, W
    ):

        """
        This function decides which end effector leaves the ground next
        """

        x_opt_same_eff = self.compute_adapted_step_locations(
            u_current, 0, n_current, dcm_t, alpha, W
        )
        x_opt_diff_eff = self.compute_adapted_step_locations(
            u_next, 0, n_current + 1, dcm_t, alpha, W
        )

        # print(x_opt_same_eff[0:2], x_opt_diff_eff[0:2])

        if np.linalg.norm(
            np.subtract(x_opt_same_eff[0:2], u_current)
        ) < np.linalg.norm(np.subtract(x_opt_diff_eff[0:2], u_next)):
            n = n_current
        else:
            n = n_current + 1

        return n

    #
    # dcm_vrp_planner.generate_foot_trajectory(x_opt[0:2], u_current_step, u_old, tend_scale * t_end, t, 0.1 , floor_height)

    def generate_foot_trajectory(
        self, u_t_end, u, u_old, t_end, t, z_max, z_ht, ctrl_timestep=0.001
    ):
        """
        This function generates a linear trajectory from the current foot location
        to the desired step location and returns the desired location of the foot
        at the next step.

        Input :
            u_t_end : desried step location
            u : current step location
            u_old : the location of the previous step
            t_end : time duration of the step
            t : current timestep
            z_max : maximum height the foot should reach(will reach at middle of the step time)
            z_ht : the height the robot must be above the ground
            ctrl_timestep : the timestep at which value is recomputed
        """

        x_foot_des_air = np.zeros(3)
        x_foot_des_ground = np.zeros(3)

        ## for impedance the leg length has to be set to zero to move center of mass forward
        if t_end > 0.001:
            x_foot_des_air[0] = u_old[0] + (u_t_end[0] - u_old[0]) * (
                t / t_end
            )
            x_foot_des_air[1] = u_old[1] + (u_t_end[1] - u_old[1]) * (
                t / t_end
            )

            x_foot_des_ground[0] = u[0]
            x_foot_des_ground[1] = u[1]

            x_foot_des_ground[2] = z_ht
            x_foot_des_air[2] = (z_ht) + (z_max) * np.sin(
                (np.pi * t) / (t_end)
            )
        else:
            x_foot_des_air = [u_t_end[0], u_t_end[1], z_ht]
            x_foot_des_ground = [u[0], u[1], z_ht]

        return x_foot_des_air, x_foot_des_ground
