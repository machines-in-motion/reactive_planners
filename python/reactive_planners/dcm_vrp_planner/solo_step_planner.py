## This file contains code to decide where the end effectors should land to satisfy ZMP constraint
##from DCM step planner
## Author: Avadesh Meduri
## Date: 12/12/2019

import numpy as np
from gurobipy import *
from reactive_planners.utils.solo_state_estimator import SoloStateEstimator


class SoloStepPlanner:
    def __init__(self, robot, kin_min, kin_max, ht):
        """
        Input:
        kin_min = kinematic constraint min offset from hip
        kin_max = kinematic constraint max offset from hip
        ht: the height of the base above the ground
        """
        assert np.shape(kin_min) == (2,)
        assert np.shape(kin_max) == (2,)

        self.robot = robot
        self.sse = SoloStateEstimator(robot)
        self.kin_min = kin_min
        self.kin_max = kin_max
        self.ht = ht

    def compute_kinematic_constraints(self, q, dq, t, t_end, v_des, ht):
        """
        Computes the kinematic constraints at the end of the step.
        Input:
            t: current time
            t_end: step time
            v_des: desired velocity
            ht: the height of the base above the ground
        """
        hip_loc = np.array(self.sse.return_hip_locations(q, dq))
        offset = np.array(v_des) * (t_end - t)
        for i in range(4):
            hip_loc[i][0:2] = np.add(hip_loc[i][0:2], offset)
            hip_loc[i][2] = ht

        return hip_loc[0], hip_loc[1], hip_loc[2], hip_loc[3]

    def compute_next_step_locations(
        self, q, dq, t, u_t, u_t_min, u_t_max, t_end, cnt_array, v_des, W
    ):
        """
        Computes the next step locations depending on the decided contact plan
        Input:
            t: currrent step time
            u_t: next step COP location from DCM planner
            u_t_min: minimum value of viability boundary
            u_t_max: max value of viabilty boundary
            t_end: step time
            cnt_array: contact array
            v_des: desired velocity
            W: weight array
        """

        m = Model("solo_step_planner")

        FL_hip, FR_hip, HL_hip, HR_hip = self.compute_kinematic_constraints(
            q, dq, t, t_end, v_des, self.ht
        )

        ## Creating Variables
        fl_x = m.addVar(
            lb=FL_hip[0] + self.kin_min[0],
            ub=FL_hip[0] + self.kin_max[0],
            name="fl_x",
        )
        fl_y = m.addVar(
            lb=FL_hip[1] + self.kin_min[1],
            ub=FL_hip[1] + self.kin_max[1],
            name="fl_y",
        )

        fr_x = m.addVar(
            lb=FR_hip[0] + self.kin_min[0],
            ub=FR_hip[0] + self.kin_max[0],
            name="fr_x",
        )
        fr_y = m.addVar(
            lb=FR_hip[1] + self.kin_min[1],
            ub=FR_hip[1] + self.kin_max[1],
            name="fr_y",
        )

        hl_x = m.addVar(
            lb=HL_hip[0] + self.kin_min[0],
            ub=HL_hip[0] + self.kin_max[0],
            name="hl_x",
        )
        hl_y = m.addVar(
            lb=HL_hip[1] + self.kin_min[1],
            ub=HL_hip[1] + self.kin_max[1],
            name="hl_y",
        )

        hr_x = m.addVar(
            lb=HR_hip[0] + self.kin_min[0],
            ub=HR_hip[0] + self.kin_max[0],
            name="hr_x",
        )
        hr_y = m.addVar(
            lb=HR_hip[1] + self.kin_min[1],
            ub=HR_hip[1] + self.kin_max[1],
            name="hr_y",
        )

        sl_1 = m.addVar(name="slack_1")
        sl_2 = m.addVar(name="slack_2")
        sl_3 = m.addVar(name="slack_3")
        sl_4 = m.addVar(name="slack_4")
        sl_5 = m.addVar(name="slack_5")
        sl_6 = m.addVar(name="slack_6")
        sl_7 = m.addVar(name="slack_7")
        sl_8 = m.addVar(name="slack_8")
        sl_9 = m.addVar(name="slack_9")
        sl_10 = m.addVar(name="slack_10")

        # creating cost
        c1 = (
            W[0] * sl_1 * sl_1
            + W[1] * sl_2 * sl_2
            + W[2] * sl_3 * sl_3
            + W[3] * sl_4 * sl_4
            + W[4] * sl_5 * sl_5
            + W[5] * sl_6 * sl_6
            + W[6] * sl_7 * sl_7
            + W[7] * sl_8 * sl_8
            + W[8] * sl_9 * sl_9
            + W[9] * sl_10 * sl_10
        )

        m.setObjective(c1)

        ## constraints
        if cnt_array[0] == 1:
            # m.addConstr(fl_x >= u_t_max[0], "viability_max_constr_fl_x")
            # m.addConstr(fl_y >= u_t_max[1], "viability_max_constr_fl_y")
            # m.addConstr(fl_x <= u_t_min[0], "viability_min_constr_fl_x")
            # m.addConstr(fl_y <= u_t_min[1], "viability_min_constr_fl_y")

            m.addConstr(fl_x == FL_hip[0] + sl_3)
            m.addConstr(fl_y == FL_hip[1] + sl_4)

        if cnt_array[1] == 1:
            # m.addConstr(fr_x >= u_t_max[0], "viability_max_constr_fr_x")
            # m.addConstr(fr_y >= u_t_max[1], "viability_max_constr_fr_y")
            # m.addConstr(fr_x <= u_t_min[0], "viability_min_constr_fr_x")
            # m.addConstr(fr_y <= u_t_min[1], "viability_min_constr_fr_y")

            m.addConstr(fr_x == FR_hip[0] + sl_5)
            m.addConstr(fr_y == FR_hip[1] + sl_6)

        if cnt_array[2] == 1:
            # m.addConstr(hl_x >= u_t_max[0], "viability_max_constr_hl_x")
            # m.addConstr(hl_y >= u_t_max[1], "viability_max_constr_hl_y")
            # m.addConstr(hl_x <= u_t_min[0], "viability_min_constr_hl_x")
            # m.addConstr(hl_y <= u_t_min[1], "viability_min_constr_hl_y")

            m.addConstr(hl_x == HL_hip[0] + sl_7)
            m.addConstr(hl_y == HL_hip[1] + sl_8)

        if cnt_array[3] == 1:
            # m.addConstr(hr_x >= u_t_max[0], "viability_max_constr_hr_x")
            # m.addConstr(hr_y >= u_t_max[1], "viability_max_constr_hr_y")
            # m.addConstr(hr_x <= u_t_min[0], "viability_min_constr_hr_x")
            # m.addConstr(hr_y <= u_t_min[1], "viability_min_constr_hr_y")

            m.addConstr(hr_x == HR_hip[0] + sl_9)
            m.addConstr(hr_x == HR_hip[1] + sl_10)

        m.addConstr(
            cnt_array[0] * fl_x
            + cnt_array[1] * fr_x
            + cnt_array[2] * hl_x
            + cnt_array[3] * hr_x
            == np.sum(cnt_array) * (u_t[0]) + sl_1,
            "sl_constr_1",
        )
        m.addConstr(
            cnt_array[0] * fl_y
            + cnt_array[1] * fr_y
            + cnt_array[2] * hl_y
            + cnt_array[3] * hr_y
            == np.sum(cnt_array) * (u_t[1]) + sl_2,
            "sl_constr_2",
        )

        m.optimize()
        x_opt = m.getVars()

        return (
            x_opt[0].x,
            x_opt[1].x,
            x_opt[2].x,
            x_opt[3].x,
            x_opt[4].x,
            x_opt[5].x,
            x_opt[6].x,
            x_opt[7].x,
        )
