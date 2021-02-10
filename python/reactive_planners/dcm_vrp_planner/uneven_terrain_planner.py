### Author : Avadesh Meduri
### Date : 11/11/2019
### Implementation of the Split DCM planner for uneven terrain using MIP

import numpy as np
from gurobipy import *


class SplitDcmContactPlanner:
    def __init__(
        self,
        l_min,
        l_max,
        w_min,
        w_max,
        h_min,
        h_max,
        t_min,
        t_max,
        v_des,
        l_p,
        ht,
        terrain_constraints=None,
    ):

        """
        Input:
            l_min : minimum step length in the x direction (in the direction of forward motion)
            l_max : maximum step length in the x direction (in the direction of forward motion)
            w_min : minimum step length in the y direction (in the lateral direction)
            w_max : maximum steo lenght in the y direction (in the lateratl direction)
            h_min : minimum step length in the z direction (in the vertical direction)
            h_max : maximum step lenght in the z direction (in the vertical direction)
            t_min : minimum step time
            t_max : maximum step time
            v_des : desired average velocity in the x and y ([v_x, v_y, v_z]) 2d vector
            l_p : default step widt-alphah
            ht : average desired height of the com above the ground
            terrain_constraints : terrain_constraints from the utils.py file
        """

        assert np.shape(v_des) == (3,)

        self.l_min = l_min
        self.l_max = l_max
        self.w_min = w_min
        self.w_max = w_max
        self.h_min = h_min
        self.h_max = h_max
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

        self.bz_max = self.h_max / (
            np.power(np.e, self.omega * self.t_min) - 1
        )
        self.bz_min = self.h_min / (
            np.power(np.e, self.omega * self.t_max) - 1
        )

        self.ter_cts = terrain_constraints

    def compute_nominal_step_values(self, n):
        """
        computes nominal step location and step length at each time step
        input :
            n : 1 if left leg and 2 if right le is in contact
        """

        if self.v_des[0] == 0 or self.v_des[1] == 0 or self.v_des[2] == 0:
            B_l = self.t_min
            B_u = self.t_max
        else:
            B_l = np.max(
                [
                    self.l_min / abs(self.v_des[0]),
                    self.w_min / abs(self.v_des[1]),
                    self.h_min / abs(self.v_des[2]),
                    self.t_min,
                ]
            )
            B_u = np.min(
                [
                    self.l_max / abs(self.v_des[0]),
                    self.w_max / abs(self.v_des[1]),
                    self.h_max / abs(self.v_des[2]),
                    self.t_max,
                ]
            )

        t_nom = (B_l + B_u) / 2.0

        l_nom = self.v_des[0] * t_nom
        w_nom = self.v_des[1] * t_nom
        h_nom = self.v_des[2] * t_nom

        bx_nom = l_nom / (np.power(np.e, self.omega * t_nom) - 1)
        by_nom = (
            np.power(-1, n)
            * (self.l_p / (1 + np.power(np.e, self.omega * t_nom)))
        ) - w_nom / (1 - np.power(np.e, self.omega * t_nom))
        bz_nom = h_nom / (np.power(np.e, self.omega * t_nom) - 1)

        return l_nom, w_nom, h_nom, t_nom, bx_nom, by_nom, bz_nom

    def compute_dcm_current(self, x, xd):
        """
        computes the current location of the dcm
        input :
            x : center of mass location at current time step
            xd : center of mass velocity of current time step
        """
        assert np.shape(x) == (3,)

        return (xd / self.omega) + x

    def compute_alpha(self, xd_com, v_des):
        """
        computes the current value of alpha. Alpha takes a value of 1 if v_des and
        xd_com are zero. When alpha is one t has to be zero to satisfy complementary constraint.
        Otherwise it takes a value of 0.
        """

        if max(v_des) == 0 and np.square(max(xd_com)) < 0.001:
            return 1
        else:
            return 0

    def compute_adapted_step_locations(
        self,
        u1,
        u2,
        t1,
        t2,
        n1,
        n2,
        psi_current,
        alpha,
        W,
        off_1=None,
        off_2=None,
    ):
        """
        computes the next step location for the two VRPs
        Input :
            u1 : the location of the previous step of the first dcm (2d vector) [ux, uy]
            u2 : the location of the previous step of the second dcm (2d vector) [ux, uy]
            t1 : time elapsed after the previous step of the first dcm
            t2 : time elapsed after the previous step of the second dcm
            n1: 1 if left leg and 2 if right leg is in contact for the first dcm
            n2 :1 if left leg and 2 if right leg is in contact for the second dcm
            psi_current : current location of the dcm
            W : wieght array
            alpha : variable that sets step time to zero if robot is not moving and should not move
            off_1 : offset by which vrp 1 is moved due to external force
            off_2 : offset by which vrp 2 is moved due to external force
        """

        ##### To improve speed this can be defined at the initialisation so that this is not recomputed

        (
            l_nom1,
            w_nom1,
            h_nom1,
            t_nom1,
            bx_nom1,
            by_nom1,
            bz_nom1,
        ) = self.compute_nominal_step_values(n1)
        (
            l_nom2,
            w_nom2,
            h_nom2,
            t_nom2,
            bx_nom2,
            by_nom2,
            bz_nom2,
        ) = self.compute_nominal_step_values(n2)

        t_nom1 = np.power(
            np.e, self.omega * t_nom1
        )  ### take exp as T is considered as e^wt in qp
        t_nom2 = np.power(
            np.e, self.omega * t_nom2
        )  ### take exp as T is considered as e^wt in qp

        ### setting up the optimizer model
        ### move this to the constructor to save computation time
        m = Model("step_planner")
        ## creating variables
        ut_x1 = m.addVar(lb=self.l_min, ub=self.l_max, name="ut_x1")
        ut_y1 = m.addVar(lb=self.w_min, ub=self.w_max, name="ut_y1")
        ut_z1 = m.addVar(lb=self.h_min, ub=self.h_max, name="ut_z1")
        t_step1 = m.addVar(
            lb=np.power(np.e, self.omega * self.t_min),
            ub=np.power(np.e, self.omega * self.t_max),
            name="t_step1",
        )
        b_x1 = m.addVar(lb=self.bx_min, ub=self.bx_max, name="b_x1")
        b_y1 = m.addVar(lb=self.by_max_out, ub=self.by_max_in, name="b_y1")
        b_z1 = m.addVar(lb=self.bz_min, ub=self.bz_max, name="b_z1")

        ut_x2 = m.addVar(lb=self.l_min, ub=self.l_max, name="ut_x2")
        ut_y2 = m.addVar(lb=self.w_min, ub=self.w_max, name="ut_y2")
        ut_z2 = m.addVar(lb=self.h_min, ub=self.h_max, name="ut_z2")
        t_step2 = m.addVar(
            lb=np.power(np.e, self.omega * self.t_min),
            ub=np.power(np.e, self.omega * self.t_max),
            name="t_step2",
        )
        b_x2 = m.addVar(lb=self.bx_min, ub=self.bx_max, name="b_x2")
        b_y2 = m.addVar(lb=self.by_max_out, ub=self.by_max_in, name="b_y2")
        b_z2 = m.addVar(lb=self.bz_min, ub=self.bz_max, name="b_z2")

        ## creating cost
        c1 = (
            (W[0] * (ut_x1 - l_nom1) * (ut_x1 - l_nom1))
            + (W[1] * (ut_y1 - w_nom1) * (ut_y1 - w_nom1))
            + (W[2] * (ut_z1 - h_nom1) * (ut_z1 - h_nom1))
            + (W[3] * (t_step1 - t_nom1) * (t_step1 - t_nom1))
            + (W[4] * (b_x1 - bx_nom1) * (b_x1 - bx_nom1))
            + (W[5] * (b_y1 - by_nom1) * (b_y1 - by_nom1))
            + (W[6] * (b_z1 - bz_nom1) * (b_z1 - bz_nom1))
        )
        c2 = (
            (W[7] * (ut_x2 - l_nom2) * (ut_x2 - l_nom2))
            + (W[8] * (ut_y2 - w_nom2) * (ut_y2 - w_nom2))
            + (W[9] * (ut_z2 - h_nom2) * (ut_z2 - h_nom2))
            + (W[10] * (t_step2 - t_nom2) * (t_step2 - t_nom2))
            + (W[11] * (b_x2 - bx_nom2) * (b_x2 - bx_nom2))
            + (W[12] * (b_y2 - by_nom2) * (b_y2 - by_nom2))
            + (W[13] * (b_z2 - bz_nom2) * (b_z2 - bz_nom2))
        )

        m.setObjective(c1 + c2)

        ## creating dyamics constraints
        tmp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        tmp[0] = (psi_current[0] - u1[0]) * np.power(
            np.e, -1 * self.omega * t1
        )
        tmp[1] = (psi_current[1] - u1[1]) * np.power(
            np.e, -1 * self.omega * t1
        )
        tmp[2] = (psi_current[2] - u1[2]) * np.power(
            np.e, -1 * self.omega * t1
        )
        tmp[3] = (psi_current[0] - u2[0]) * np.power(
            np.e, -1 * self.omega * t2
        )
        tmp[4] = (psi_current[1] - u2[1]) * np.power(
            np.e, -1 * self.omega * t2
        )
        tmp[5] = (psi_current[2] - u2[2]) * np.power(
            np.e, -1 * self.omega * t2
        )

        m.addConstr(ut_x1 == (tmp[0] * t_step1) - b_x1, "dyn_constr_x1")
        m.addConstr(ut_y1 == (tmp[1] * t_step1) - b_y1, "dyn_constr_y1")
        m.addConstr(ut_z1 == (tmp[2] * t_step1) - b_z1, "dyn_constr_z1")

        m.addConstr(ut_x2 == (tmp[3] * t_step2) - b_x2, "dyn_constr_x2")
        m.addConstr(ut_y2 == (tmp[4] * t_step2) - b_y2, "dyn_constr_y2")
        m.addConstr(ut_z2 == (tmp[5] * t_step2) - b_z2, "dyn_constr_z2")

        ### kinematic constraints
        m.addConstr(
            (ut_x1 + u1[0] - off_1[0]) - (ut_x2 + u2[0] - off_2[0]) <= 0.5,
            "kin_constr_min_dist",
        )  ## make them based on L_maxa and L_min
        m.addConstr(
            (ut_x1 + u1[0] - off_1[0]) - (ut_x2 + u2[0] - off_2[0]) >= 0.3,
            "kin_constr_min_dist",
        )

        if self.ter_cts != None:
            ### adding terrain constraints
            M = 999999
            ter_vars_1 = []  # DCM1
            ter_vars_2 = []  # DCM2
            no_terrains = int(len(self.ter_cts) / 6)
            # no_terrains = 1
            for i in range(no_terrains):
                name_1 = "ter1_" + str(i)
                name_2 = "ter2_" + str(i)
                ter_vars_1.append(m.addVar(vtype=GRB.BINARY, name=name_1))
                ter_vars_2.append(m.addVar(vtype=GRB.BINARY, name=name_2))

                m.addLConstr(
                    ut_x1 + u1[0] - off_1[0]
                    <= (M * ter_vars_1[-1]) + self.ter_cts[6 * i + 0]
                )
                m.addLConstr(
                    ut_x1 + u1[0] - off_1[0]
                    >= (-M * ter_vars_1[-1]) + self.ter_cts[6 * i + 1]
                )
                # m.addLConstr(ut_y1 + u1[1] - off_1[1] <= (M*ter_vars_1[-1]) + self.ter_cts[6*i + 2])
                # m.addLConstr(ut_y1 + u1[1] - off_1[2] >= (-M*ter_vars_1[-1]) + self.ter_cts[6*i + 3])
                #     # m.addLConstr(ut_z1 + u1[2] <= (M*ter_vars_1[-1]) + self.ter_cts[6*i + 4])
                #     # m.addLConstr(ut_z1 + u1[2] >= (-M*ter_vars_1[-1]) + self.ter_cts[6*i + 5])

                m.addLConstr(
                    ut_x2 + u2[0] - off_2[0]
                    <= (M * ter_vars_2[-1]) + self.ter_cts[6 * i + 0]
                )
                m.addLConstr(
                    ut_x2 + u2[0] - off_2[0]
                    >= (-M * ter_vars_2[-1]) + self.ter_cts[6 * i + 1]
                )
                # m.addLConstr(ut_y2 + u2[1] - off_2[1] <= (M*ter_vars_2[-1]) + self.ter_cts[6*i + 2])
                # m.addLConstr(ut_y2 + u2[1] - off_2[1] >= (-M*ter_vars_2[-1]) + self.ter_cts[6*i + 3])
            #     # m.addLConstr(ut_z2 + u2[2] <= (M*ter_vars_2[-1]) + self.ter_cts[6*i + 4])
            #     # m.addLConstr(ut_z2 + u2[2] >= (-M*ter_vars_2[-1]) + self.ter_cts[6*i + 5])

            m.addConstr(
                sum(ter_vars_1) == (no_terrains - 1), "dcm1_terrain_constraint"
            )
            m.addConstr(
                sum(ter_vars_2) == (no_terrains - 1), "dcm2_terrain_constraint"
            )

        m.optimize()
        x_opt = m.getVars()

        t_end1 = np.log(x_opt[3].x) / self.omega
        t_end2 = np.log(x_opt[10].x) / self.omega
        if self.ter_cts != None:
            for i in range(no_terrains):
                name_1 = "ter1_" + str(i)
                a = m.getVarByName(name_1).x
                if a == 0:
                    print(
                        "Terrain selected at current timestep ............................. \n"
                    )
                    print(name_1)

        return (
            ut_x1.x + u1[0],
            ut_y1.x + u1[1],
            ut_z1.x + u1[2],
            t_end1,
            ut_x2.x + u2[0],
            ut_y2.x + u2[1],
            ut_z2.x + u2[2],
            t_end2,
        )

    def generate_foot_trajectory(
        self,
        u_t_end,
        u,
        u_old,
        t_end,
        t,
        z_max,
        z_ht,
        u_hip,
        u_old_hip,
        n,
        ctrl_timestep=0.001,
    ):
        """
        This function generates a linear trajectory from the current foot location
        to the desired step location and returns the desired location of the foot
        at the next step.

        Input :
            u_t_end : desried step location
            u : current step location
            u_old : location of the previous step
            t_end : time duration of the step
            t : current timestep
            z_max : maximum height the foot should reach(will reach at middle of the step time)
            z_ht : the height the robot must be above the ground
            u_hip : location of hip of the leg on the ground
            u_old_hip : location of the hip of the leg that is in the air
            n : 1 or 2 depending on which leg is in the air
            ctrl_timestep : the timestep at which value is recomputed
        """

        x_foot_des_air = np.zeros(3)
        x_foot_des_ground = np.zeros(3)

        u_hip[1] = 0
        u_old_hip[1] = 0

        ## for impedance the leg length has to be set to zero to move center of mass forward
        if t_end > 0.001:

            ## assumption that the center of mass is in the middle of the two legs at the contact phase
            ## This will be removed when center of mass trajectories are obtained from a trajectory planner
            if t < t_end / 2.0:
                x_foot_des_ground[0] = -1 * (u[0] - u_hip[0]) + (
                    (u[0] - u_hip[0]) / (0.5 * t_end)
                ) * (t)
                x_foot_des_ground[1] = -(u[1] - u_hip[1]) + (
                    (u[1] - u_hip[1]) / (0.5 * t_end)
                ) * (t)

                x_foot_des_air[0] = (u_old[0] - u_old_hip[0]) - (
                    (u_old[0] - u_old_hip[0]) / (0.5 * t_end)
                ) * (t)
                x_foot_des_air[1] = (u_old[1] - u_old_hip[1]) - (
                    (u_old[1] - u_old_hip[1]) / (0.5 * t_end)
                ) * (t)

            else:
                t_ = t - 0.5 * t_end
                x_foot_des_ground[0] = (
                    ((0.5 * (u_t_end[0] + u[0])) - u_hip[0]) / (0.5 * t_end)
                ) * (t_)
                x_foot_des_ground[1] = (
                    ((0.5 * (u_t_end[1] + u[1])) - u_hip[1]) / (0.5 * t_end)
                ) * (t - (0.5 * t_end))

                x_foot_des_air[0] = (
                    (u_t_end[0] - u_old_hip[0]) / (0.5 * t_end)
                ) * (t_)
                x_foot_des_air[1] = (
                    (u_t_end[1] - u_old_hip[1]) / (0.5 * t_end)
                ) * (t_)

            x_foot_des_ground[2] = z_ht
            x_foot_des_air[2] = (z_ht) + (z_max) * np.sin(
                (np.pi * t) / (t_end)
            )

        else:
            x_foot_des_air = [u_t_end[0], u_t_end[1], z_ht]
            x_foot_des_ground = [u[0], u[1], z_ht]

        return x_foot_des_air, x_foot_des_ground
