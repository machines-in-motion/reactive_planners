## Re implementation of the dcm planner for debugging
## Author : Avadesh Meduri
## Date : 20/11/ 2019

import numpy as np

import time

import os
import rospkg
import pybullet as p
import pinocchio as se3
from pinocchio.utils import se3ToXYZQUAT

from robot_properties_solo.config import Solo12Config
from robot_properties_solo.quadruped12wrapper import Quadruped12Robot

from py_blmc_controllers.solo_impedance_controller import SoloImpedanceController 
from py_blmc_controllers.solo_centroidal_controller import SoloCentroidalController

from py_dcm_vrp_planner.re_split_dcm_planner import TrajGenerator, DCMStepPlanner, SoloStateEstimator

from pinocchio.utils import zero
from matplotlib import pyplot as plt


#################################################################################################

# Create a robot instance. This initializes the simulator as well.
robot = Quadruped12Robot(ifrecord=False)
tau = np.zeros(12)

# Reset the robot to some initial state.
q0 = np.matrix(Solo12Config.initial_configuration).T
dq0 = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q0, dq0)

arr = lambda a: np.array(a).reshape(-1)
mat = lambda a: np.matrix(a).reshape((-1, 1))
total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])

################### initial Parameters of Step Planner ############################################

l_min = -0.15
l_max = 0.15
w_min = -0.1
w_max = 0.1
t_min = 0.05
t_max = 0.3
l_p = 0.0
ht = 0.25
v_des = [0.2, 0.]
W = [100., 100., 1., 1000., 1000.]

step_planner = DCMStepPlanner(l_min, l_max, w_min, w_max, t_min, t_max, l_p, ht)

#################### Impedance Control Paramters ###################################################

x_des = 4*[0.0, 0.0, -ht]
xd_des = 4*[0,0,0] 
kp = 4 * [300,300,300]
kd = 4 * [10.0,10.0,10.0]

##################### Centroidal Control Parameters ##################################################

x_com = [0.0, 0.0, ht]
xd_com = [0.0, 0.0, 0.0]

x_ori = [0., 0., 0., 1.]
x_angvel = [0., 0., 0.]
cnt_array = [1, 1, 1, 1]

solo_leg_ctrl = SoloImpedanceController(robot)
centr_controller = SoloCentroidalController(robot.pin_robot, total_mass,
        mu=0.6, kc=[100,100,100], dc=[10,10,10], kb=[200,200,200], db=[15.,15.,15.],
        eff_ids=robot.pinocchio_endeff_ids)

#################### Trajecory Generator ################################################################
trj = TrajGenerator(robot.pin_robot)
f_lift = 0.1 ## height the foot lifts of the ground
#################### initial location of the robot #######################################################

sse = SoloStateEstimator(robot.pin_robot)
q, dq = robot.get_state()
com_init = np.reshape(np.array(q[0:3]), (3,))
fl_foot, fr_foot, hl_foot, hr_foot = sse.return_foot_locations(q,dq)
fl_off, fr_off, hl_off, hr_off = sse.return_hip_offset(q, dq)
u = 0.5*(np.add(fl_foot, hr_foot))[0:2]
n = 1
t = 0
t_end = 9999
sim_time = 10000

################## For plotting ##########################################################################

plt_x_des = []

################# Running Simulation ######################################################################

for i in range(sim_time):
    p.stepSimulation()
    time.sleep(0.0001) 
    
    q, dq = robot.get_state()
    com = np.reshape(np.array(q[0:3]), (3,))
    
    if i%10 == 0: ## reducing frequency of computation of stepping to 100 hz
        if t > t_end:
            t = 0
            u = u_t
            n += 1
            fl_foot, fr_foot, hl_foot, hr_foot = sse.return_foot_locations(q, dq) ## computing current location of the feet

        psi = sse.return_dcm_location(q, dq, step_planner.omega)
        x_opt = step_planner.compute_adapted_step_location(u, t, n, psi, W, v_des)
        
        t_end = x_opt[2]
        u_t = x_opt[0:2]

    ##### generating trajectory
    fl_hip, fr_hip, hl_hip, hr_hip = sse.return_hip_locations(q, dq)
    if np.power(-1, n) < 0: ## fl, hr leave the ground
        u_t_des = [[u_t[0], u_t[1], 0.0], [fr_foot[0], fr_foot[1], 0.0], [hl_foot[0], hl_foot[1], 0.0], [u_t[0], u_t[1], 0.0]]
        u_t_des[0] = np.add(u_t_des[0], fl_off) # assumption
        u_t_des[3] = np.add(u_t_des[3], hr_off) # assumption
        x_des[0:3] = np.subtract(trj.generate_foot_traj([fl_foot[0], fl_foot[1],0.0], u_t_des[0], [0.0, 0.0, f_lift], t_end,t), [fl_hip[0], fl_hip[1], ht])
        x_des[3:6] = np.subtract(trj.generate_foot_traj([fr_foot[0], fr_foot[1],0.0], u_t_des[1], [0.0, 0.0, 0.0], t_end,t), [fr_hip[0], fr_hip[1], ht])
        x_des[6:9] = np.subtract(trj.generate_foot_traj([hl_foot[0], hl_foot[1],0.0], u_t_des[2], [0.0, 0.0, 0.0], t_end,t), [hl_hip[0], hl_hip[1], ht])
        x_des[9:12] = np.subtract(trj.generate_foot_traj([hr_foot[0], hr_foot[1],0.0], u_t_des[3], [0.0, 0.0, f_lift], t_end,t), [hr_hip[0], hr_hip[1], ht])
        cnt_array = [0, 1, 1, 0]

    elif np.power(-1, n) > 0: ## fr and hl leave the ground
        u_t_des = [[fl_foot[0], fl_foot[1], 0.0], [u_t[0], u_t[1], 0.0], [u_t[0], u_t[1], 0.0], [hr_foot[0], hr_foot[1], 0.0]]
        u_t_des[1] = np.add(u_t_des[1], fr_off) # assumption
        u_t_des[2] = np.add(u_t_des[2], hl_off) # assumption
        x_des[0:3] = np.subtract(trj.generate_foot_traj([fl_foot[0], fl_foot[1],0.0], u_t_des[0], [0.0, 0.0, 0.0], t_end,t), [fl_hip[0], fl_hip[1], ht])
        x_des[3:6] = np.subtract(trj.generate_foot_traj([fr_foot[0], fr_foot[1],0.0], u_t_des[1], [0.0, 0.0, f_lift], t_end,t), [fr_hip[0], fr_hip[1], ht])
        x_des[6:9] = np.subtract(trj.generate_foot_traj([hl_foot[0], hl_foot[1],0.0], u_t_des[2], [0.0, 0.0, f_lift], t_end,t), [hl_hip[0], hl_hip[1], ht])
        x_des[9:12] = np.subtract(trj.generate_foot_traj([hr_foot[0], hr_foot[1],0.0], u_t_des[3], [0.0, 0.0, 0.0], t_end,t), [hr_hip[0], hr_hip[1], ht])
        cnt_array = [1, 0, 0, 1]


    ### Appending plot array ###########
    plt_x_des.append([u_t_des[0][0], u_t_des[0][1]])

    ## computing desired center of mass location and velocity
    x_com[0:2] = com_init[0:2] +np.array(v_des)*i*0.001
    xd_com[0:2] = v_des
    
    ### plugging Torques
    w_com = centr_controller.compute_com_wrench(i, q, dq, x_com, xd_com, x_ori, x_angvel)
    w_com[2] += total_mass * 9.81
    F = centr_controller.compute_force_qp(i, q, dq, cnt_array, w_com)
    tau = solo_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,F)
    robot.send_joint_command(tau)
    t += 0.001


################### 

plt_x_des = np.array(plt_x_des)

plt.plot(plt_x_des[:,0])
plt.plot(plt_x_des[:,1])
# plt.plot(plt_x_des[:,3])
plt.show()
