### Author : Avadesh Meduri
### Date : 30/10/2019
### Extension of DCM VRP planner in 2d but split of DCMs for solo. 

import numpy as np
from py_dcm_vrp_planner.qp_solver import quadprog_solve_qp


class DcmContactPlanner:
    
    def __init__(self, l_min, l_max, w_min, w_max, t_min, t_max, v_des, l_p, ht):
        
        '''
            Input:
                l_min : minimum step length in the x direction (in the direction of forward motion)
                l_max : maximum step length in the x direction (in the direction of forward motion)
                w_min : minimum step length in the y direction (in the lateral direction)
                w_max : maximum steo lenght in the y direction (in the lateratl direction)
                t_min : minimum step time
                t_max : maximum step time
                v_des : desired average velocity in the x and y ([v_x, v_y]) 2d vector
                l_p : default step width
                ht : average desired height of the com above the ground
        '''
        
        assert(np.shape(v_des) == (2,))
        
        self.l_min = l_min
        self.l_max = l_max
        self.w_min = w_min
        self.w_max = w_max
        self.t_min = t_min
        self.t_max = t_max
        self.v_des = v_des
        self.l_p = l_p
        self.ht = ht
        
        self.omega = np.sqrt(9.81/self.ht)
        self.bx_max = self.l_max / (np.power(np.e, self.omega*self.t_min) - 1)
        self.bx_min = self.l_min / (np.power(np.e, self.omega*self.t_max) - 1)
        self.by_max_out =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_max - self.w_min * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) 
   
        self.by_max_in =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_min - self.w_max * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) 
                             
    
    def compute_nominal_step_values(self, n):
        
        '''
            computes nominal step location and step length at each time step
            input : 
                n : 1 if left leg and 2 if right le is in contact
        '''
        
        if self.v_des[0] == 0 or self.v_des[1] == 0:
            B_l = self.t_min
            B_u = self.t_max
        else:
            B_l = np.max([self.l_min/abs(self.v_des[0]), self.w_min/abs(self.v_des[1]), self.t_min])
            B_u = np.min([self.l_max/abs(self.v_des[0]), self.w_max/abs(self.v_des[1]), self.t_max])
            
        t_nom = (B_l + B_u)/2.0
        l_nom = self.v_des[0] * t_nom
        w_nom = self.v_des[1] * t_nom
                        
        bx_nom = l_nom / (np.power(np.e, self.omega*t_nom) - 1)
        by_nom = (np.power(-1, n) * (self.l_p/(1 + np.power(np.e, self.omega*t_nom)))) - \
                        w_nom / (1 - np.power(np.e, self.omega*t_nom))
        
        return l_nom, w_nom, t_nom, bx_nom, by_nom
    
    def compute_dcm_current(self, x, xd):
        '''
            computes the current location of the dcm
            input :
                x : center of mass location at current time step
                xd : center of mass velocity of current time step
        '''    

        return (xd/self.omega) + x 
    
    
    def compute_adapted_step_locations(self, u1, u2, t1, t2, n1, n2, psi_current, W):
        '''
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
        '''

        l_nom1, w_nom1, t_nom1, bx_nom1, by_nom1 = self.compute_nominal_step_values(n1)
        l_nom2, w_nom2, t_nom2, bx_nom2, by_nom2 = self.compute_nominal_step_values(n2)
        
        t_nom1 = np.power(np.e, self.omega*t_nom1) ### take exp as T is considered as e^wt in qp
        t_nom2 = np.power(np.e, self.omega*t_nom2) ### take exp as T is considered as e^wt in qp
        
        P = np.identity(10) ## quadratic cost matrix
        for k in range(10):
            P[k][k] = W[k]
            
        q = np.array([-W[0]*(l_nom1),
                      -W[1]*(w_nom1),
                      -W[2]*t_nom1,
                      -W[3]*bx_nom1,
                      -W[4]*by_nom1,
                      -W[5]*(l_nom2),
                      -W[6]*(w_nom2),
                      -W[7]*t_nom2,
                      -W[8]*bx_nom2,
                      -W[9]*by_nom2]) ## quadratic cost vector
         
        G = np.matrix([ [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, -1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, -1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, -1, 0, 0, 0, 0, 0], 
                        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0], ## this for dcm 2
                        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, -1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, -1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, -1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -1, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1]])
        
        h = np.array([self.l_max,
                      self.w_max,
                     -1*(self.l_min),
                     -1*(self.w_min),
                     np.power(np.e, self.omega*self.t_max),
                     -1*np.power(np.e, self.omega*self.t_min),
                     self.bx_max,
                     -1*self.bx_min,
                     self.by_max_in,
                     -1*self.by_max_out, 
                     self.l_max,
                      self.w_max,
                     -1*(self.l_min),
                     -1*(self.w_min),
                     np.power(np.e, self.omega*self.t_max),
                     -1*np.power(np.e, self.omega*self.t_min),
                     self.bx_max,
                     -1*self.bx_min,
                     self.by_max_in,
                     -1*self.by_max_out])   
         
        tmp = [0.,0., 0., 0.]
        tmp[0] = (psi_current[0] - u1[0])*np.power(np.e, -1*self.omega*t1)
        tmp[1] = (psi_current[1] - u1[1])*np.power(np.e, -1*self.omega*t1)
        tmp[2] = (psi_current[0] - u2[0])*np.power(np.e, -1*self.omega*t2)
        tmp[3] = (psi_current[1] - u2[1])*np.power(np.e, -1*self.omega*t2)
        A = np.matrix([[1, 0, -1*tmp[0], 1, 0, 0, 0, 0, 0, 0],
                        [0, 1, -1*tmp[1], 0, 1, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, -1*tmp[2], 1, 0],
                        [0, 0, 0, 0, 0, 0, 1, -1*tmp[3], 0, 1]])
        
        b = np.array([0.0, 0.0, 0.0, 0.0])
        
        P = P.astype(float)
        q = q.astype(float)
        G = G.astype(float)
        h = h.astype(float)
        A = A.astype(float)
        b = b.astype(float)
        
        x_opt = quadprog_solve_qp(P,q, G, h, A, b)
        t_end1 = np.log(x_opt[2])/self.omega
        t_end2 = np.log(x_opt[7])/self.omega
        
        print(self.bx_min, x_opt[3], x_opt[8], self.bx_max)    

        return (x_opt[0] + u1[0], x_opt[1] + u1[1], t_end1, x_opt[5] + u2[0], x_opt[6] + u2[1], t_end2)
    
    def generate_foot_trajectory(self, u_t_end, u, t_end, t, z_max, z_ground, ctrl_timestep = 0.001):
        '''
            This function generates a linear trajectory from the current foot location
            to the desired step location and returns the desired location of the foot 
            at the next step.
            
            Input :
                u_t_end : desried step location
                u : current step location
                t_end : time duration of the step
                t : current timestep
                z_max : maximum height the foot should reach(will reach at middle of the step time)
                z_ground : the location of the ground
                ctrl_timestep : the timestep at which value is recomputed
        '''
        
        x_foot_des_air = np.zeros(3)
        x_foot_des_ground = np.zeros(3)
        
        ## for impedance the leg length has to be set to zero to move center of mass forward
        
        x_foot_des_air[0] = (u_t_end[0] - u[0])*np.sin((np.pi*t)/(t_end))
        x_foot_des_air[1] = (u_t_end[1] - u[1])*np.sin((np.pi*t)/(t_end))
        
        # print(u_t_end[0])
        
        if t < t_end/2.0:
            x_foot_des_air[2] = z_ground + z_max*np.sin((np.pi*t)/(2.0*t_end))
        else:
            x_foot_des_air[2] = z_ground
        
        ## assumption that the center of mass is between the two legs 
        x_foot_des_ground[0] = 0.0
        x_foot_des_ground[1] = 0.0
        x_foot_des_ground[2] = z_ground
        
        return x_foot_des_air, x_foot_des_ground