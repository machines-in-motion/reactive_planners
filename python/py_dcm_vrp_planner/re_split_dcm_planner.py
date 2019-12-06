## Re implementation of the split dcm planner for debugging
## Author : Avadesh Meduri
## Date : 20/11/ 2019

import numpy as np
from gurobipy import *
import pinocchio as pin


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
        self.omega = np.sqrt(9.81/self.ht)
        self.bx_max = self.l_max / (np.power(np.e, self.omega*self.t_min) - 1)
        self.bx_min = self.l_min / (np.power(np.e, self.omega*self.t_max) - 1)
        self.by_max_out =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_max - self.w_min * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) 
   
        self.by_max_in =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_min - self.w_max * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) 


    def compute_nominal_values(self, n, v_des):
        
        if v_des[0] == 0 or v_des[1] == 0:
            B_l = self.t_min
            B_u = self.t_max
        else:
            B_l = np.max([self.l_min/abs(v_des[0]), self.w_min/abs(v_des[1]), self.t_min])
            B_u = np.min([self.l_max/abs(v_des[0]), self.w_max/abs(v_des[1]), self.t_max])        

        t_nom = (B_l + B_u)/2.0
        t_nom = np.power(np.e, self.omega*t_nom) ### take exp as T is considered as e^wt in qp
        
        l_nom = v_des[0] * t_nom
        w_nom = v_des[1] * t_nom

        bx_nom = l_nom / (np.power(np.e, self.omega*t_nom) - 1)
        by_nom = (np.power(-1, n) * (self.l_p/(1 + np.power(np.e, self.omega*t_nom)))) - \
                        w_nom / (1 - np.power(np.e, self.omega*t_nom))
        
        return l_nom, w_nom, t_nom, bx_nom, by_nom
    
    def compute_adapted_step_location(self, u1, t1, n1, psi, W, v_des):
        
        l_nom1, w_nom1, t_nom1, bx_nom1, by_nom1 = self.compute_nominal_values(n1, v_des)
                
        m = Model("split_dcm")
        
        # Creating Variables
        ut1_x = m.addVar(lb=self.l_min + u1[0], ub=self.l_max + u1[0], name = "ut1_x")
        ut1_y = m.addVar(lb=self.w_min + u1[1], ub=self.w_max + u1[1], name = "ut1_y")    
        t1_step = m.addVar(lb=np.power(np.e, self.omega*self.t_min), ub=np.power(np.e, self.omega*self.t_max), name="t1_step")
        b1_x = m.addVar(lb=self.bx_min, ub=self.bx_max, name="b1_x")
        b1_y = m.addVar(lb=self.by_max_out, ub=self.by_max_in, name="b1_y")
        
        # Creating Cost
        c1 = W[0]*(ut1_x - u1[0] - l_nom1)*(ut1_x - u1[0] - l_nom1) + W[1]*(ut1_y - u1[1] - w_nom1)*(ut1_y - u1[1] - w_nom1) \
            + W[2]*(t1_step - t_nom1)*(t1_step - t_nom1) + W[3]*(b1_x - bx_nom1)*(b1_x - bx_nom1) + W[4]*(b1_y - by_nom1)*(b1_y - by_nom1) 
        
        m.setObjective(c1)
        
        ## Constraints
        tmp1 = np.subtract(psi[0:2],u1)*np.power(np.e, -1*self.omega*t1)
        m.addConstr(ut1_x == tmp1[0]*t1_step + u1[0] - b1_x, "dyn_1_x")
        m.addConstr(ut1_y == tmp1[1]*t1_step + u1[1] - b1_y, "dyn_1_y")

        m.optimize()
        
        x_opt = m.getVars()
        
        t1_end = np.log(x_opt[2].x)/self.omega
    
        return x_opt[0].x, x_opt[1].x, t1_end 

    def compute_com_trajectory(self, x_com, u1, horizon, dt = 0.001):
        """
        Computes the desired center of mass trajectory using LIPM dynamics
        psi_d = omega(psi - u)
        x_comd = omega(psi - x)
        Input:
            psi: current location of DCM
            x_com: current location of COM
            u1: current location of step
            t: step time
            horizon: the number of steps to which com trajectory is to be computed
        """

        assert np.shape(psi) == (3,)
        assert np.shape(x_com) == (3,)

        des_x_com = []
        des_xd_com = []
        des_x_com.append(x_com[0:2])
        des_xd_com.append(xd_com[0:2])
        for i in range(0, int(horizon)):
            psi_t = np.add((np.subtract(psi[0:2], u1)*np.power(np.e, self.omega*t)), u1)
            xd_com = self.omega*np.subtract(psi_t, des_x_com[-1])
            des_x_com.append(np.add(des_x_com[-1], xd_com*dt))
            des_xd_com.append(xd_com)
            t += dt
        return np.array(des_x_com), np.array(des_xd_com)

class TrajGenerator:
    
    def __init__(self, robot):
        
        self.robot = robot
        
    def get_frame_location(self, q, dq, frame_idx):
        """
            returns the global location of the frame
        """
        self.robot.framesForwardKinematics(q)
        return np.reshape(np.array(self.robot.model.oMf[frame_idx].translation), (3,))
        
    def generate_traj(self, start, end, traj_time, t):
        """
            returns desired location at a given time t, 
            Generates a straight line start to end point 
        """

        slope = np.divide(np.subtract(end, start), traj_time)
        
        return np.add(start, slope*t)
    
    def generate_sin_traj(self, start, end, via_point, traj_time, t):
        """
            returns desired location at a giv        psi = sse.return_dcm_location(q, dq, step_planner.omega)
en time t,
            Generates sine trajectory from start to end point through a via point
            The trajectory passes through the via point at mid duration
        """
        assert np.shape(start) == np.shape(end)
        assert np.shape(start) == np.shape(via_point)
        
        if t < traj_time/2.0:
            amplitude = np.subtract(via_point,start)
            omega = (np.pi)/traj_time
            return np.add(start, amplitude*np.sin(omega*t))
        elif t == traj_time/2.0:
            return via_point
        else:
            amplitude = np.subtract(via_point, end)
            omega = (np.pi)/traj_time
            return np.add(end, amplitude*np.sin(omega*t))
        
    def generate_foot_traj(self,start, end, via_point, traj_time,t):
        """
            Generates foot trajecotry for solo
            Input:
                Start: Current location of the foot 3d
                end: desired location of the of the foot at the end of the step
                via_point: the hieght in the z axis the leg has to rise
                traj_time: step time
                t: current time
        """
 
        assert np.shape(start) == (3,)
        assert np.shape(end) == (3,)
 
        x_des = np.zeros(3)
        x_des[0:2] = self.generate_traj(start[0:2], end[0:2], traj_time, t)
        x_des[2] = self.generate_sin_traj(start[2], end[2], via_point[2], traj_time, t)
             
        return x_des
        
class SoloStateEstimator:
    
    def __init__(self, robot):
        self.robot = robot
        self.x = np.zeros(3)
        self.xd = np.zeros(3)
        
    def get_frame_location(self, q, dq, frame_idx):
        """
            returns the global location of the frame
        """
        self.robot.framesForwardKinematics(q)
        return np.reshape(np.array(self.robot.data.oMf[frame_idx].translation), (3,))
        
    def return_foot_locations(self, q, dq):
        
        fl_foot = self.get_frame_location(q, dq, self.robot.model.getFrameId("FL_FOOT"))
        fr_foot = self.get_frame_location(q, dq, self.robot.model.getFrameId("FR_FOOT"))
        hl_foot = self.get_frame_location(q, dq, self.robot.model.getFrameId("HL_FOOT"))
        hr_foot = self.get_frame_location(q, dq, self.robot.model.getFrameId("HR_FOOT"))
        
        return fl_foot, fr_foot, hl_foot, hr_foot
    
    def return_hip_locations(self, q, dq):
        
        fl_hip = self.get_frame_location(q, dq, self.robot.model.getFrameId("FL_HFE"))
        fr_hip = self.get_frame_location(q, dq, self.robot.model.getFrameId("FR_HFE"))
        hl_hip = self.get_frame_location(q, dq, self.robot.model.getFrameId("HL_HFE"))
        hr_hip = self.get_frame_location(q, dq, self.robot.model.getFrameId("HR_HFE"))
        
        return fl_hip, fr_hip, hl_hip, hr_hip
    
    def return_dcm_location(self, q, dq, omega):
        self.x = np.reshape(np.array(q[0:3]), (3,))
        self.xd = np.reshape(np.array(dq[0:3]), (3,))
        return self.x + self.xd/omega
    
    def return_hip_offset(self, q, dq):
        """
            Returns the distance between the hip and the center of mass
        """
        fl_hip, fr_hip, hl_hip, hr_hip = self.return_hip_locations(q, dq)
        com = np.reshape(np.array(q[0:3]), (3,))
        return np.subtract(fl_hip, com), np.subtract(fr_hip, com), np.subtract(hl_hip, com), np.subtract(hr_hip, com)
        