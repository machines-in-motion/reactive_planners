### Author : Avadesh Meduri
### Date : 30/10/2019
### Extension of DCM VRP planner in 3d but split of DCMs for solo. 

import numpy as np
from py_dcm_vrp_planner.qp_solver import quadprog_solve_qp


class DcmContactPlanner:
    
    def __init__(self, l_min, l_max, w_min, w_max, h_min, h_max, t_min, t_max, v_des, l_p, ht):
        
        '''
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
                l_p : default step width
                ht : average desired height of the com above the ground
        '''
        
        assert(np.shape(v_des) == (3,))
        
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
        
        self.omega = np.sqrt(9.81/self.ht)
        self.bx_max = self.l_max / (np.power(np.e, self.omega*self.t_min) - 1)
        self.bx_min = self.l_min / (np.power(np.e, self.omega*self.t_max) - 1)
        
        self.by_max_out =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_max - self.w_min * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) 
   
        self.by_max_in =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_min - self.w_max * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) 
        
        self.bz_max = self.h_max / (np.power(np.e, self.omega*self.t_min) - 1)
        self.bz_min = self.h_min / (np.power(np.e, self.omega*self.t_max) - 1)
        
                             
    
    def compute_nominal_step_values(self, n):
        
        '''
            computes nominal step location and step length at each time step
            input : 
                n : 1 if left leg and 2 if right le is in contact
        '''
        
        if self.v_des[0] == 0 or self.v_des[1] == 0 or self.v_des[2] == 0:
            B_l = self.t_min
            B_u = self.t_max
        else:
            B_l = np.max([self.l_min/abs(self.v_des[0]), self.w_min/abs(self.v_des[1]), self.h_min/abs(self.v_des[2]), self.t_min])
            B_u = np.min([self.l_max/abs(self.v_des[0]), self.w_max/abs(self.v_des[1]), self.h_max/abs(self.v_des[2]), self.t_max])
            
        t_nom = (B_l + B_u)/2.0
        
        l_nom = self.v_des[0] * t_nom
        w_nom = self.v_des[1] * t_nom
        h_nom = self.v_des[2] * t_nom
                        
        bx_nom = l_nom / (np.power(np.e, self.omega*t_nom) - 1)
        by_nom = (np.power(-1, n) * (self.l_p/(1 + np.power(np.e, self.omega*t_nom)))) - \
                        w_nom / (1 - np.power(np.e, self.omega*t_nom))
        bz_nom = h_nom / (np.power(np.e, self.omega*t_nom) - 1)
        
        return l_nom, w_nom, h_nom, t_nom, bx_nom, by_nom, bz_nom
    
    def compute_dcm_current(self, x, xd):
        '''
            computes the current location of the dcm
            input :
                x : center of mass location at current time step
                xd : center of mass velocity of current time step
        '''    
        assert np.shape(x) == (3,)
        self.x = x
        return (xd/self.omega) + x 
    
    def compute_alpha(self, xd_com, v_des):
        '''
            computes the current value of alpha. Alpha takes a value of 1 if v_des and
            xd_com are zero. When alpha is one t has to be zero to satisfy complementary constraint.
            Otherwise it takes a value of 0.
        '''
                    
        if max(v_des) == 0 and np.square(max(xd_com)) < 0.001:
            return 1
        else:
            return 0
    
    def compute_adapted_step_locations(self, u1, u2, t1, t2, n1, n2, psi_current, alpha, W):
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
                alpha : variable that sets step time to zero if robot is not moving and should not move
        '''

        ##### To improve speed this can be defined at the initialisation so that this is not recomputed 


        l_nom1, w_nom1, h_nom1, t_nom1, bx_nom1, by_nom1, bz_nom1 = self.compute_nominal_step_values(n1)
        l_nom2, w_nom2, h_nom2, t_nom2, bx_nom2, by_nom2, bz_nom2 = self.compute_nominal_step_values(n2)
        
        t_nom1 = np.power(np.e, self.omega*t_nom1) ### take exp as T is considered as e^wt in qp
        t_nom2 = np.power(np.e, self.omega*t_nom2) ### take exp as T is considered as e^wt in qp
        
        P = np.identity(18) ## quadratic cost matrix
        for k in range(18):
            P[k][k] = W[k]
            
        q = np.array([-W[0]*(l_nom1),
                      -W[1]*(w_nom1),
                      -W[2]*(h_nom1), 
                      -W[3]*t_nom1,
                      -W[4]*bx_nom1,
                      -W[5]*by_nom1,
                      -W[6]*bz_nom1,
                      0,
                      0,
                      -W[9]*(l_nom2),
                      -W[10]*(w_nom2),
                      -W[11]*(h_nom2),
                      -W[12]*t_nom2,
                      -W[13]*bx_nom2,
                      -W[14]*by_nom2,
                      -W[15]*bz_nom2,
                      0,
                      0] ) ## quadratic cost vector
                
        ## constaint matrix for one mass
        G_dcm = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0, 0, 0, 0],
                          [-1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, -1, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, -1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0, 0, 0, 0],
                          [0, 0, 0, -1, 0, 0, 0, 0, 0],
                          [0, 0, 0, alpha, 0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 1, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, -1, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, -1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, -1, 0, 0],
                          [0, 0, 1, 0, 0, 0, 0, 1, 0]])
          
        h_dcm = np.array([  self.l_max,
                            self.w_max,
                            self.h_max,  
                            -1*(self.l_min),
                            -1*(self.w_min),
                            -1*(self.h_min),
                            np.power(np.e, self.omega*self.t_max),
                            -1*np.power(np.e, self.omega*self.t_min),
                            alpha,
                            self.bx_max,
                            self.by_max_in,
                            self.bz_max,
                            -1*self.bx_min,
                            -1*self.by_max_out,
                            -1*self.bz_min,
                            0]) ## ground constant 
                          
        G = np.block([[G_dcm, np.zeros(np.shape(G_dcm))],
                      [np.zeros(np.shape(G_dcm)), G_dcm]])  
        
        h = np.hstack((h_dcm, h_dcm))
                
        tmp = [0., 0., 0., 0., 0., 0.]
        tmp[0] = (psi_current[0] - u1[0])*np.power(np.e, -1*self.omega*t1)
        tmp[1] = (psi_current[1] - u1[1])*np.power(np.e, -1*self.omega*t1)
        tmp[2] = (psi_current[2] - u1[2])*np.power(np.e, -1*self.omega*t1)
        tmp[3] = (psi_current[0] - u2[0])*np.power(np.e, -1*self.omega*t2)
        tmp[4] = (psi_current[1] - u2[1])*np.power(np.e, -1*self.omega*t2)
        tmp[5] = (psi_current[2] - u2[2])*np.power(np.e, -1*self.omega*t2)
        A = np.matrix([ [1, 0, 0, -1*tmp[0], 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, -1*tmp[1], 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 1, -1*tmp[2], 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1*tmp[3], 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, -1*tmp[4], 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1*tmp[5], 0, 0, 1, 0, 0]])
        
        b = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        P = P.astype(float)
        q = q.astype(float)
        G = G.astype(float)
        h = h.astype(float)
        A = A.astype(float)
        b = b.astype(float)
                
        x_opt = quadprog_solve_qp(P,q, A = A, b = b, G=G, h=h)
    
        t_end1 = np.log(x_opt[3])/self.omega
        t_end2 = np.log(x_opt[12])/self.omega
        
        return (x_opt[0] + u1[0], x_opt[1] + u1[1], x_opt[2] + u1[2], t_end1, x_opt[9] + u2[0], x_opt[10] + u2[1], x_opt[11] + u2[2], t_end2)
     

    def generate_foot_trajectory(self, u_t_end, u, u_old, t_end, t, z_max, z_ht, offset, n, ctrl_timestep = 0.001):
        '''
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
                offsset : The distance between the leg hip frame and COM
                n : 1 or 2 depending on which leg is in the air
                ctrl_timestep : the timestep at which value is recomputed
        '''
        
        x_foot_des_air = np.zeros(3)
        x_foot_des_ground = np.zeros(3)
        
        ## for impedance the leg length has to be set to zero to move center of mass forward
        if t_end > 0.001:
            
            ## assumption that the center of mass is in the middle of the two legs at the contact phase
            ## This will be removed when center of mass trajectories are obtained from a trajectory planner 
            if t < t_end/2.0 :            
                # x_foot_des_ground[0:2] = np.subtract(0.5*np.add(u[0:2], u_old[0:2]), self.x[0:2])*(-1 + np.sin((np.pi*t)/t_end))
        
                x_foot_des_ground[0] = -((0.5*(u_old[0] + u[0])) - offset[0]) + (((0.5*(u_old[0] + u[0])) - offset[0])/(0.5*t_end))*(t)
                x_foot_des_ground[1] = -((0.5*(u_old[1] + u[1])) - offset[1]) + (((0.5*(u_old[1] + u[1])) - offset[1])/(0.5*t_end))*(t)
        
                x_foot_des_air[0] =  u_old[0] - offset[0] - ((u_old[0] - offset[0])/(0.5*t_end))*(t)
                x_foot_des_air[1] =  u_old[1] - offset[1] - ((u_old[1] - offset[1])/(0.5*t_end))*(t)
                
            else:
                x_foot_des_ground[0] = (((0.5*(u_t_end[0] + u[0])) - offset[0])/(0.5*t_end))*(t - (0.5*t_end))
                x_foot_des_ground[1] = (((0.5*(u_t_end[1] + u[1])) - offset[1])/(0.5*t_end))*(t - (0.5*t_end))
        
                x_foot_des_air[0] =  ((u_t_end[0] - offset[0])/(0.5*t_end))*(t - (0.5*t_end))
                x_foot_des_air[1] =  ((u_t_end[1] - offset[1])/(0.5*t_end))*(t - (0.5*t_end))

                
            x_foot_des_ground[2] = z_ht
            x_foot_des_air[2] = (z_ht) +  (z_max) *np.sin((np.pi*t)/(t_end))
            
        
        else:
            x_foot_des_air = [u_t_end[0], u_t_end[1], z_ht]
            x_foot_des_ground = [u[0], u[1],z_ht]
        
        return x_foot_des_air, x_foot_des_ground
