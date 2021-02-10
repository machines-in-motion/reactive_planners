## Implementation of the dcm planner
## Author : Avadesh Meduri
## Date : 20/11/ 2019

import numpy as np
from gurobipy import *
from reactive_planners.utils.qp_solver import quadprog_solve_qp


class DCMStepPlanner:
    def __init__(self, l_min, l_max, w_min, w_max, t_min, t_max, l_p, ht):

        self.l_min = l_min
        self.l_max = l_max
        self.w_min = w_min
        self.w_max = w_max
        self.t_min = t_min
        self.t_max = t_max
        self.ht = ht
        self.l_p = l_p
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

    def compute_nominal_values(self, n, v_des):

        if v_des[0] == 0 or v_des[1] == 0:
            B_l = self.t_min
            B_u = self.t_max
        else:
            B_l = np.max(
                [
                    self.l_min / abs(v_des[0]),
                    self.w_min / abs(v_des[1]),
                    self.t_min,
                ]
            )
            B_u = np.min(
                [
                    self.l_max / abs(v_des[0]),
                    self.w_max / abs(v_des[1]),
                    self.t_max,
                ]
            )

        t_nom = (B_l + B_u) / 2.0
        t_nom = np.power(
            np.e, self.omega * t_nom
        )  ### take exp as T is considered as e^wt in qp

        l_nom = v_des[0] * t_nom
        w_nom = v_des[1] * t_nom

        bx_nom = l_nom / (np.power(np.e, self.omega * t_nom) - 1)
        by_nom = (
            np.power(-1, n)
            * (self.l_p / (1 + np.power(np.e, self.omega * t_nom)))
        ) - w_nom / (1 - np.power(np.e, self.omega * t_nom))

        return l_nom, w_nom, t_nom, bx_nom, by_nom

    def compute_adapted_step_location(self, u, t, n, psi, W, v_des):

        l_nom, w_nom, t_nom, bx_nom, by_nom = self.compute_nominal_values(
            n, v_des
        )

        P = np.identity(5)  ## quadratic cost matrix
        P[0][0], P[1][1], P[2][2], P[3][3], P[4][4] = W
        q = np.array(
            [
                -W[0] * (l_nom),
                -W[1] * (w_nom),
                -W[2] * t_nom,
                -W[3] * bx_nom,
                -W[4] * by_nom,
            ]
        )  ##

        G = np.matrix(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [-1, 0, 0, 0, 0],
                [0, -1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, -1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, -1, 0],
                [0, 0, 0, 0, 1],
                [0, 0, 0, 0, -1],
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
                self.bx_max,
                -1 * self.bx_min,
                self.by_max_in,
                -1 * self.by_max_out,
            ]
        )

        tmp = [0.0, 0.0]
        tmp[0] = (psi[0] - u[0]) * np.power(np.e, -1 * self.omega * t)
        tmp[1] = (psi[1] - u[1]) * np.power(np.e, -1 * self.omega * t)
        A = np.matrix([[1, 0, -1 * tmp[0], 1, 0], [0, 1, -1 * tmp[1], 0, 1]])

        b = np.array([0.0, 0.0])
        # b = np.array([u[0], u[1]])

        P = P.astype(float)
        q = q.astype(float)
        G = G.astype(float)
        h = h.astype(float)
        A = A.astype(float)
        b = b.astype(float)

        try:
            self.x_opt = quadprog_solve_qp(P, q, G, h, A, b)
        except ValueError as err:
            import pdb

            pdb.set_trace()

        t_end = np.log(self.x_opt[2]) / self.omega

        return self.x_opt[0] + u[0], self.x_opt[1] + u[1], t_end

    def compute_adapted_step_location_gurobi(self, u1, t1, n1, psi, W, v_des):

        l_nom1, w_nom1, t_nom1, bx_nom1, by_nom1 = self.compute_nominal_values(
            n1, v_des
        )

        m = Model("split_dcm")

        # Creating Variables
        ut1_x = m.addVar(
            lb=self.l_min + u1[0], ub=self.l_max + u1[0], name="ut1_x"
        )
        ut1_y = m.addVar(
            lb=self.w_min + u1[1], ub=self.w_max + u1[1], name="ut1_y"
        )
        t1_step = m.addVar(
            lb=np.power(np.e, self.omega * self.t_min),
            ub=np.power(np.e, self.omega * self.t_max),
            name="t1_step",
        )
        b1_x = m.addVar(lb=self.bx_min, ub=self.bx_max, name="b1_x")
        b1_y = m.addVar(lb=self.by_max_out, ub=self.by_max_in, name="b1_y")

        # Creating Cost
        c1 = (
            W[0] * (ut1_x - u1[0] - l_nom1) * (ut1_x - u1[0] - l_nom1)
            + W[1] * (ut1_y - u1[1] - w_nom1) * (ut1_y - u1[1] - w_nom1)
            + W[2] * (t1_step - t_nom1) * (t1_step - t_nom1)
            + W[3] * (b1_x - bx_nom1) * (b1_x - bx_nom1)
            + W[4] * (b1_y - by_nom1) * (b1_y - by_nom1)
        )

        m.setObjective(c1)

        ## Constraints
        tmp1 = np.subtract(psi[0:2], u1) * np.power(np.e, -1 * self.omega * t1)
        m.addConstr(ut1_x == tmp1[0] * t1_step + u1[0] - b1_x, "dyn_1_x")
        m.addConstr(ut1_y == tmp1[1] * t1_step + u1[1] - b1_y, "dyn_1_y")

        m.optimize()

        x_opt = m.getVars()

        t1_end = np.log(x_opt[2].x) / self.omega

        return x_opt[0].x, x_opt[1].x, t1_end

    def compute_viability_boundary(self, u, t, psi):
        """
        copmute the maximum and minimum step location considering ll above constraints
        Input:
            u: current step location
            t: current step time
            psi: current dcm location
        """

        u_max = np.subtract(psi[0:2], u) * np.power(
            np.e, -1 * self.omega * t
        ) * np.power(np.e, self.omega * self.t_max) + np.subtract(
            u[0:2], np.array([self.bx_min, self.by_max_out])
        )
        u_min = np.subtract(psi[0:2], u) * np.power(
            np.e, -1 * self.omega * t
        ) * np.power(np.e, self.omega * self.t_min) + np.subtract(
            u[0:2], np.array([self.bx_max, self.by_max_in])
        )

        if u_max[0] > self.l_max + u[0]:
            u_max[0] = self.l_max + u[0]
        elif u_max[1] > self.w_max + u[1]:
            u_max[1] = self.w_max + u[1]

        if u_min[0] < self.l_min + u[0]:
            u_min[0] = self.l_min + u[0]
        elif u_min[1] < self.w_min + u[1]:
            u_min[1] = self.w_min + u[1]

        return u_max, u_min
