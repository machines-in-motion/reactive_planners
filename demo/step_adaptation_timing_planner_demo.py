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

from py_impedance_control.solo_impedance_controller import solo_impedance_controller 

from pinocchio.utils import zero
from matplotlib import pyplot as plt

from py_dcm_vrp_planner.planner import dcm_vrp_planner


#######################################################################################


l_min = 0.0
l_max = 0.2
w_min = 0.0
w_max = 0.2
t_min = 0.01
t_max = 0.6
v_des = [0.0,0]
l_p = 0
ht = 0.22


#######################################################################################


dcm_vrp_planner = dcm_vrp_planner(l_min, l_max, w_min, w_max, t_min, t_max, v_des, l_p, ht)

# Create a robot instance. This initializes the simulator as well.
robot = Quadruped12Robot()
tau = np.zeros(12)

# Reset the robot to some initial state.
q0 = np.matrix(Solo12Config.initial_configuration).T
dq0 = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q0, dq0)

###################### impedance controller demo #################################

x_des = 4*[0.0, 0.0, -0.25]
xd_des = 4*[0,0,0] 
kp = 4 * [200,200,500]
kd = 4 * [10.0,10.0,10.0]
f = np.zeros(18)
f = 4*[0.0, 0.0, (2.2*9.8)/4]
##################################################################################

solo_leg_ctrl = solo_impedance_controller(robot)

# Run the simulator for 100 steps
t = 0
t_gap = 0.05
t_end = t_max
n = 1
u_current_step = [0.0, 0.0]
x_com = np.array([0.0, 0.0]) 
xd_com = np.array([0.0, 0.0]) 

x_bias = [0.2, 0.1, 0.0, 0.2, -0.1, 0.0, -0.2, 0.1, 0.0, -0.2, -0.1, 0.0]

tmp = []

for i in range(6000):
    # TODO: Implement a controller here.    
    # Step the simulator.
    p.stepSimulation()
    time.sleep(0.0001) # You can sleep here if you want to slow down the replay
    # Read the final state and forces after the stepping.

    q, dq = robot.get_state()
    x_com[0] = float(q[0])
    x_com[1] = float(q[1])
    xd_com[0] = float(dq[0])
    xd_com[1] = float(dq[1])

    ## computing the current location of the legs FL, FR
    fl_location = np.array(solo_leg_ctrl.FL_imp.compute_distance_between_frames(q))
    fl_location = np.reshape(fl_location, (3,))
    fr_location = np.array(solo_leg_ctrl.FR_imp.compute_distance_between_frames(q))
    fr_location = np.reshape(fr_location, (3,))
    if t < t_end:
        if t_end - t > t_gap: 
            ### This if statement prevents adaptation near the end of the step to prevents jumps in desrired location.
            dcm_t = dcm_vrp_planner.compute_dcm_current(x_com,xd_com)
            x_opt = dcm_vrp_planner.compute_adapted_step_locations(u_current_step, t, n, dcm_t)
            t_end = x_opt[2]
            if np.power(-1, n) > 0:
                x_des_fl_hr,xd_des_fl_hr = dcm_vrp_planner.generate_foot_trajectory(x_opt[0:2], t_end, t, fl_location, -.1, -0.26)
                x_des[0:3] = np.reshape(x_des_fl_hr, (3,))
                x_des[3:5] = u_current_step
                x_des[6:8] = u_current_step
                x_des[9:12] = np.reshape(x_des_fl_hr, (3,))
                x_des += x_bias
                tmp.append(x_des[2])
                # xd_des = 4*[0,0,0] 
                # xd_des[0:3] = xd_des_fl_hr 
                # xd_des[6:9] = xd_des_fl_hr
            else:
                x_des_fr_hl, xd_des_fr_hl = dcm_vrp_planner.generate_foot_trajectory(x_opt[0:2], t_end, t, fr_location, -0.1, -0.26)
                x_des[0:2] = u_current_step 
                x_des[3:6] = np.reshape(x_des_fr_hl, (3,))
                x_des[6:9] = np.reshape(x_des_fr_hl, (3,))
                x_des[9:11] = u_current_step
                x_des += x_bias
                
                # xd_des = 4*[0,0,0] 
                # xd_des[3:6] = xd_des_fr_hl
                # xd_des[6:9] = xd_des_fr_hl
        t+=0.001
    
    else:
        t = 0
        u_current_step = [x_opt[0],x_opt[1]]
        # print(u_current_step)
        n += 1
    tau = solo_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,f)
    robot.send_joint_command(tau)




# plt.plot(tmp)
# plt.show()