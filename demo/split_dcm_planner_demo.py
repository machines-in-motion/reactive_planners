### Author : Avadesh Meduri
### Date : 30/10/2019
### Extension of DCM VRP planner in 2d but split of DCMs for solo. 

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

from pinocchio.utils import zero
from matplotlib import pyplot as plt


from py_dcm_vrp_planner.split_dcm_planner import DcmContactPlanner

# Create a robot instance. This initializes the simulator as well.
robot = Quadruped12Robot(ifrecord=False)
tau = np.zeros(12)

# Reset the robot to some initial state.
q0 = np.matrix(Solo12Config.initial_configuration).T
dq0 = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q0, dq0)

#######################################################

x_des = 4*[0.0, 0.0, -0.25]
xd_des = 4*[0,0,0] 
kp = 4 * [200,200,500]
kd = 4 * [10.0,10.0,10.0]
f = np.zeros(18)
f = 4*[0.0, 0.0, (2.2*9.8)/4]

l_min = -0.15
l_max = 0.15
w_min = -0.05
w_max = 0.15
t_min = 0.01
t_max = 0.3
v_des = [1.0,1.0]
l_p = 0
ht = 0.25
ht_foot = 0.2
dcm_contact_planner = DcmContactPlanner(l_min, l_max, w_min, w_max, t_min, t_max, v_des, l_p, ht)

W = 2*[10, 10, 1, 1000, 1000] # weight on [step length_x , step_length_y, step time, dcm_offeset_x, dcm_offeset_y]

########################################################################

solo_leg_ctrl = SoloImpedanceController(robot)

t1 = 0
t2 = 0
t_gap = 0.05
t1_end = t_max
t2_end = t_max
n1 = 1
n2 = 2
u1_current_step = [0.2, 0.0]
u2_current_step = [-0.2, 0.0]
x_com = np.array([0.0, 0.0]) 
xd_com = np.array([0.0, 0.0]) 


### test 

for i in range(10000):
    p.stepSimulation()
    time.sleep(0.0001) 
    
    q, dq = robot.get_state()
    x_com[0] = float(q[0])
    x_com[1] = float(q[1])
    xd_com[0] = float(dq[0])
    xd_com[1] = float(dq[1])
    
    if t1 > t1_end:
        t1 = 0
        u1_current_step = x_opt[0:2]
        n1 += 1
    if t2 > t2_end:
        t2 = 0
        u2_current_step = x_opt[3:5]
        n2 += 1

    u1_current_step_eff = np.add(u1_current_step, np.subtract(u2_current_step, x_com))
    u2_current_step_eff = np.add(u2_current_step, np.subtract(u1_current_step, x_com))

    # print(u1_current_step_eff[0], u2_current_step_eff[0])
    ### This if statement prevents adaptation near the end of the step to prevents jumps in desrired location.
    dcm_t = dcm_contact_planner.compute_dcm_current(x_com,xd_com)
    x_opt = dcm_contact_planner.compute_adapted_step_locations(u1_current_step_eff, u2_current_step_eff, t1, t2, n1, n2, dcm_t, W)
    t1_end = x_opt[2]
    t2_end = x_opt[5]
    if np.power(-1, n1) > 0:
        x_des_fl, x_des_fr = dcm_contact_planner.generate_foot_trajectory(x_opt[0:2], u1_current_step, t1_end, t1, ht_foot,-ht)
        
    elif np.power(-1, n1) < 0:
        x_des_fr, x_des_fl = dcm_contact_planner.generate_foot_trajectory(x_opt[0:2], u1_current_step, t1_end, t1, ht_foot,-ht)
    
    if np.power(-1, n2) > 0:    
        x_des_hl, x_des_hr = dcm_contact_planner.generate_foot_trajectory(x_opt[3:5], u2_current_step, t2_end, t2, ht_foot,-ht)
    
    elif np.power(-1, n2) < 0:    
        x_des_hr, x_des_hl = dcm_contact_planner.generate_foot_trajectory(x_opt[3:5], u2_current_step, t2_end, t2, ht_foot,-ht)
        
    x_des[0:3] = np.reshape(x_des_fl, (3,))
    x_des[3:6] = np.reshape(x_des_fr, (3,))
    x_des[6:9] = np.reshape(x_des_hl, (3,))
    x_des[9:12] = np.reshape(x_des_hr, (3,))
    
    
    t1+=0.001
    t2+=0.001
    tau = solo_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,f)
    robot.send_joint_command(tau)