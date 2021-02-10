### Author : Avadesh Meduri
### Date : 27/09/2019
### This is the demo of the paper "walking control based on step timing Adaption" by Majid Et al.
### for the quadruped robot solo

import numpy as np

import time

import os
import rospkg
import pybullet as p
import pinocchio as se3
from pinocchio.utils import se3ToXYZQUAT

from robot_properties_solo.config import Solo12Config
from robot_properties_solo.quadruped12wrapper import Quadruped12Robot

from mim_control.solo_centroidal_controller import (
    SoloCentroidalController,
)
from mim_control.solo_impedance_controller import (
    SoloImpedanceController,
)

from pinocchio.utils import zero
from matplotlib import pyplot as plt

from reactive_planners.dcm_vrp_planner.planner import DcmVrpPlanner


#######################################################################################


# Create a robot instance. This initializes the simulator as well.
robot = Quadruped12Robot()
tau = np.zeros(12)

# Reset the robot to some initial state.
q0 = np.matrix(Solo12Config.initial_configuration).T
dq0 = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q0, dq0)

arr = lambda a: np.array(a).reshape(-1)
mat = lambda a: np.matrix(a).reshape((-1, 1))
total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])


#######################################################

x_des = 4 * [0.0, 0.0, -0.25]
xd_des = 4 * [0, 0, 0]
kp = 4 * [400, 400, 400]
kd = 4 * [20.0, 20.0, 20.0]
##################################################################################

x_com_cent = [0.0, 0.0, 0.25]
xd_com_cent = [0.0, 0.0, 0.0]

x_ori = [0.0, 0.0, 0.0, 1.0]
x_angvel = [0.0, 0.0, 0.0]
cnt_array = [1, 1, 1, 1]

solo_leg_ctrl = SoloImpedanceController(robot)
centr_controller = SoloCentroidalController(
    robot.pin_robot,
    total_mass,
    mu=0.6,
    kc=[0, 0, 0],
    dc=[0, 0, 10],
    kb=[100, 100, 100],
    db=[10.0, 10.0, 10.0],
    eff_ids=robot.pinocchio_endeff_ids,
)


#################################################################################

l_min = -0.2
l_max = 0.2
w_min = -0.1
w_max = 0.1
t_min = 0.1
t_max = 0.3
v_des = [0.1, 0.0]
l_p = 0
ht = 0.23

dcm_vrp_planner = DcmVrpPlanner(
    l_min, l_max, w_min, w_max, t_min, t_max, v_des, l_p, ht
)

W = [
    10,
    10,
    1.0,
    1000,
    1000,
    100,
]  # weight on [step length_x , step_length_y, step time, dcm_offeset_x, dcm_offeset_y]
#######################################################################################

solo_leg_ctrl = SoloImpedanceController(robot)

# Run the simulator for 100 steps
t = 0
t_gap = 0.05
t_end = t_max
n = 1
u_current_step = [0.0, 0.0]
u_old = [0.0, 0.0]
x_com = np.array([0.0, 0.0])
xd_com = np.array([0.0, 0.0])

## for plotting
plt_opt = []
plt_foot = []
plt_com = []
plt_force = []
for i in range(5000):
    p.stepSimulation()
    time.sleep(0.0001)

    # if i > 1000 and i < 1200:
    #     force = np.array([0,10,0])
    #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force, \
    #                     posObj=[0.5,0.,0], flags = p.WORLD_FRAME)

    q, dq = robot.get_state()
    x_com[0] = float(q[0])
    x_com[1] = float(q[1])
    xd_com[0] = float(dq[0])
    xd_com[1] = float(dq[1])
    plt_com.append((x_com[0], x_com[1]))

    if t < t_end:
        if t_end - t > t_gap:
            ### This if statement prevents adaptation near the end of the step to prevents jumps in desrired location.
            dcm_t = dcm_vrp_planner.compute_dcm_current(x_com, xd_com)
            alpha = dcm_vrp_planner.compute_alpha(xd_com, v_des)
            x_opt = dcm_vrp_planner.compute_adapted_step_locations(
                u_current_step, t, n, dcm_t, alpha, W
            )
            # x_opt = dcm_vrp_planner.compute_adapted_step_locations_gurobi(u_current_step, t, n, dcm_t, alpha, W)
            t_end = x_opt[2]

            if np.power(-1, n) > 0:
                (
                    x_des_fl_hr,
                    x_des_fr_hl,
                ) = dcm_vrp_planner.generate_foot_trajectory(
                    x_opt[0:2], u_current_step, u_old, t_end, t, 0.2, -0.25
                )
                x_des[0:3] = np.reshape(x_des_fl_hr, (3,))
                x_des[3:6] = np.reshape(x_des_fr_hl, (3,))
                x_des[6:9] = np.reshape(x_des_fr_hl, (3,))
                x_des[9:12] = np.reshape(x_des_fl_hr, (3,))
                cnt_array = [0, 1, 0, 1]
            else:
                (
                    x_des_fr_hl,
                    x_des_fl_hr,
                ) = dcm_vrp_planner.generate_foot_trajectory(
                    x_opt[0:2], u_current_step, u_old, t_end, t, 0.2, -0.25
                )
                x_des[0:3] = np.reshape(x_des_fl_hr, (3,))
                x_des[3:6] = np.reshape(x_des_fr_hl, (3,))
                x_des[6:9] = np.reshape(x_des_fr_hl, (3,))
                x_des[9:12] = np.reshape(x_des_fl_hr, (3,))
                cnt_array = [1, 0, 1, 0]
        t += 0.001
        # print(x_des)

    else:
        t = 0
        # n_old = n
        n += 1
        t_end = t_max
        u_old = u_current_step
        u_current_step = [x_opt[0], x_opt[1]]

    plt_opt.append(x_opt)
    plt_foot.append(
        [x_des[0], x_des[1], x_des[2], x_des[3], x_des[4], x_des[5]]
    )
    w_com = centr_controller.compute_com_wrench(
        t, q, dq, x_com_cent, xd_com_cent, x_ori, x_angvel
    )
    w_com[2] += total_mass * 9.81
    F = centr_controller.compute_force_qp(i, q, dq, cnt_array, w_com)
    plt_force.append([F[0], F[1], F[2], F[3], F[4], F[5]])
    tau = solo_leg_ctrl.return_joint_torques(q, dq, kp, kd, x_des, xd_des, F)
    robot.send_joint_command(tau)


# #### plotting

plt_opt = np.array(plt_opt)
plt_foot = np.array(plt_foot)
plt_com = np.array(plt_com)
plt_force = np.array(plt_force)
t = np.arange(0, 5, 0.001)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

ax1.plot(t, plt_opt[:, 0], label="ut_x")
ax1.plot(t, np.add(plt_com[:, 0], plt_foot[:, 0]), label="fl")
ax1.plot(t, np.add(plt_com[:, 0], plt_foot[:, 3]), label="fr")
ax1.grid()
ax1.legend()
ax2.plot(t, plt_foot[:, 2], label="fl")
ax2.plot(t, plt_foot[:, 5], label="fr")
ax2.grid()
ax2.legend()
ax3.plot(t, plt_foot[:, 0], label="fl")
ax3.plot(t, plt_foot[:, 3], label="fr")
ax3.grid()
ax3.legend()

plt.show()
