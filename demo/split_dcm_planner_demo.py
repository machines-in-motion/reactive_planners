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
from py_blmc_controllers.solo_centroidal_controller import SoloCentroidalController


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


#####################################################

# uneven_terrain = ("/home/ameduri/py_devel/workspace/src/catkin/reactive_planners/python/py_dcm_vrp_planner/terrains/stairs.urdf")
# uneven_terrain_id = p.loadURDF(uneven_terrain)

#######################################################

x_des = 4*[0.0, 0.0, -0.25]
xd_des = 4*[0,0,0] 
kp = 4 * [400,400,300]
kd = 4 * [10.0,10.0,20.0]
f = 4*[0.0, 0.0, (2.2*9.8)/4]

###########################################################



ht = 0.25
ht_foot = 0.2
l_min = -0.15
l_max = 0.15
w_min = -0.05
w_max = 0.15
h_min = -0.1
h_max = .1 
t_min = 0.00001
t_max = 0.3
v_des = [2.0, 0.0, 0.0]
l_p = 0.0
dcm_contact_planner = DcmContactPlanner(l_min, l_max, w_min, w_max, h_min, h_max, t_min, t_max, v_des, l_p, ht)

 # weight on [step length_x , step_length_y, step_length_z, step time, dcm_offeset_x, dcm_offeset_y, dcm_offeset_z, ground_slack, apha_slack]
W = 2*[100, 100, 100, 10, 10000, 10000, 10000, 10000, 100]
########################################################################

x_com = np.array([0.0, 0.0, 0.0]) 
xd_com = np.array([0.0, 0.0, 0.0]) 

## Bring the vales from the config file and remove hardcoding when refactoring
FL_HFE_idx = robot.pin_robot.model.getFrameId("FL_HFE")
FR_HFE_idx = robot.pin_robot.model.getFrameId("FR_HFE")
HL_HFE_idx = robot.pin_robot.model.getFrameId("HL_HFE")
HR_HFE_idx = robot.pin_robot.model.getFrameId("HR_HFE")
## for plotting
plt_opt = []
plt_foot = []
plt_com = []

q, dq = robot.get_state()
robot.pin_robot.framesForwardKinematics(q)
FL_HFE = np.reshape(np.array(robot.pin_robot.data.oMf[FL_HFE_idx].translation), (3,))
FR_HFE = np.reshape(np.array(robot.pin_robot.data.oMf[FR_HFE_idx].translation), (3,))
HL_HFE = np.reshape(np.array(robot.pin_robot.data.oMf[HL_HFE_idx].translation), (3,))
HR_HFE = np.reshape(np.array(robot.pin_robot.data.oMf[HR_HFE_idx].translation), (3,))

solo_leg_ctrl = SoloImpedanceController(robot)

t1 = 0
t2 = 0
t_gap = 0.05
t1_end = t_max
t2_end = t_max
n1 = 1
n2 = 2
vrp1_current = 0.5*np.add(FL_HFE, FR_HFE)
vrp2_current = 0.5*np.add(HL_HFE, HR_HFE)
u1_current_step = np.subtract(vrp1_current, [0, 0, ht])
u2_current_step = np.subtract(vrp2_current, [0, 0, ht])
u1_old = u1_current_step
u2_old = u2_current_step

### test 

for i in range(5000):
    p.stepSimulation()
    time.sleep(0.001) 
    
    # if i > 1000 and i < 2200:
    #     force = np.array([0,8,0])
    #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force, \
    #                     posObj=[0.25,0.,0], flags = p.WORLD_FRAME)

    ## Robot State estimation
    q, dq = robot.get_state()
    x_com[0] = float(q[0])
    x_com[1] = float(q[1])
    x_com[2] = float(q[2])
    xd_com[0] = float(dq[0])
    xd_com[1] = float(dq[1])
    xd_com[2] = float(dq[2])

    ## taking fixed offeset to prevent the base from rotating since VRP does not allow rotation
    off_dcm1 = np.add(x_com[0:2], 0.5*np.add(FL_HFE, FR_HFE)[0:2])
    off_dcm2 = np.add(x_com[0:2], 0.5*np.add(HL_HFE, HR_HFE)[0:2])    
    plt_com.append(off_dcm1)

    ## Step adaption planner

    if t1 > t1_end:
        t1 = 0
        n1 += 1
        t1_end = t_max
        u1_old = np.subtract(vrp1_current, [0, 0, ht])
        vrp1_current = x_opt[0:3]
        u1_current_step = np.subtract(vrp1_current, [0, 0, ht])
    if t2 > t2_end:
        t2 = 0
        n2 += 1
        t2_end = t_max
        u2_old = np.subtract(vrp2_current, [0, 0, ht])
        vrp2_current = x_opt[4:7]
        u2_current_step = np.subtract(vrp2_current, [0, 0, ht])
        
    vrp1_current_eff = np.add(vrp1_current, np.subtract(vrp2_current, x_com))
    vrp2_current_eff = np.add(vrp2_current, np.subtract(vrp1_current, x_com))

    ### This if statement prevents adaptation near the end of the step to prevents jumps in desrired location.
    dcm_t = dcm_contact_planner.compute_dcm_current(x_com,xd_com)
    alpha = dcm_contact_planner.compute_alpha(xd_com, v_des)
    x_opt = dcm_contact_planner.compute_adapted_step_locations(vrp1_current_eff, vrp2_current_eff, t1, t2, n1, n2, dcm_t,alpha, W)
    t1_end = x_opt[3]
    t2_end = x_opt[7]
    ### bringing the effective next step to the inertial frame
    x_opt = np.array(x_opt)
    x_opt[0:3] = np.add(x_opt[0:3], np.subtract(x_com, vrp2_current)) 
    x_opt[4:7] = np.add(x_opt[4:7], np.subtract(x_com, vrp1_current))
        
    u1_t_end = np.subtract(x_opt[0:3], [0,0,ht])
    u2_t_end = np.subtract(x_opt[4:7], [0,0,ht])
        
    if np.power(-1, n1) > 0:
        x_des_fl, x_des_fr = dcm_contact_planner.generate_foot_trajectory(u1_t_end, u1_current_step, u1_old, t1_end, t1, ht_foot,-ht, off_dcm1, n1)

    elif np.power(-1, n1) < 0:
        x_des_fr, x_des_fl = dcm_contact_planner.generate_foot_trajectory(u1_t_end, u1_current_step, u1_old, t1_end, t1, ht_foot,-ht, off_dcm1, n1)

    if np.power(-1, n2) > 0:    
        x_des_hl, x_des_hr = dcm_contact_planner.generate_foot_trajectory(u2_t_end, u2_current_step, u2_old, t2_end, t2, ht_foot,-ht, off_dcm2, n2)
    
    elif np.power(-1, n2) < 0:    
        x_des_hr, x_des_hl = dcm_contact_planner.generate_foot_trajectory(u2_t_end, u2_current_step, u2_old, t2_end, t2, ht_foot,-ht, off_dcm2, n2)
    
    x_des[0:3] = np.reshape(x_des_fl, (3,))
    x_des[3:6] = np.reshape(x_des_fr, (3,))
    x_des[6:9] = np.reshape(x_des_hl, (3,))
    x_des[9:12] = np.reshape(x_des_hr, (3,))
    
    plt_opt.append(x_opt)
    plt_foot.append([x_des[0],x_des[1],x_des[2],x_des[3],x_des[4],x_des[5]])
    t1+=0.001
    t2+=0.001
    
    tau = solo_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,f)
    robot.send_joint_command(tau)

# #### plotting

plt_opt = np.array(plt_opt)
plt_foot = np.array(plt_foot)
plt_com = np.array(plt_com)
t = np.arange(0,5, 0.001)

fig, (ax1, ax2,ax3) = plt.subplots(3,1, sharex=True)

ax1.plot(t, plt_opt[:,0], label='ut_x')
ax1.plot(t,np.add(plt_com[:,0],plt_foot[:,0]), label = 'fl')
# ax1.plot(t,np.add(plt_com[:,0],plt_foot[:,3]), label = 'fr')
ax1.grid()
ax1.legend()
# ax2.plot(t, plt_opt[:,4], label='ut_x_2')
# ax2.plot(t, plt_opt[:,0], label='ut_x_1')
ax2.plot(t,plt_foot[:,0], label = 'fl')
# ax2.plot(t,np.add(plt_com[:,0],plt_foot[:,3]), label = 'fr')
ax2.grid()
ax3.plot(t,plt_foot[:,2])
ax3.grid()
plt.show()