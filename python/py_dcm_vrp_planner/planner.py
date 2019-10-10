### Author : Avadesh Meduri
### Date : 27/09/2019
### This is the implementation of the paper "walking control based on step timing Adaption" by Majid Et al.


import numpy as np
from py_dcm_vrp_planner.qp_solver import quadprog_solve_qp




class dcm_vrp_planner:
    
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
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) ### should it t_max?
   
        self.by_max_in =  (self.l_p/(1 + np.power(np.e, self.omega*t_min))) + \
                        (self.w_min - self.w_max * np.power(np.e, self.omega*self.t_min))/ \
                             (1 - np.power(np.e, 2*self.omega*self.t_min)) ### should it t_max?
                             
                                
         
        
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
        
    def compute_adapted_step_locations(self,u, t, n, psi_current, W):
        '''
            computes adapted step location after solving QP
            Input:
                u : the location of the previous step (2d vector) [ux, uy]
                t : time elapsed after the previous step has occured
                n : 1 if left leg and 2 if right le is in contact
                psi_current : current dcm location [psi_x, psi_y]
                W : wieght array 5d
        '''    
    
        assert(np.shape(u) == (2,))
        assert(np.shape(psi_current) == (2,))

        l_nom, w_nom, t_nom, bx_nom, by_nom = self.compute_nominal_step_values(n)
        
        t_nom = np.power(np.e, self.omega*t_nom) ### take exp as T is considered as e^wt in qp
        

        ## note : add the weight values in the p and q array corectly
        P = np.identity(5) ## quadratic cost matrix
        P[0][0], P[1][1], P[2][2], P[3][3], P[4][4] = W 
        q = np.array([-2*W[0]*l_nom,
                      -2*W[1]*w_nom,
                      -2*W[2]*t_nom,
                      -2*W[3]*bx_nom,
                      -2*W[4]*by_nom ]) ## quadratic cost vector
    
        G = np.matrix([ [1, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0],
                        [-1, 0, 0, 0, 0],
                        [0, -1, 0, 0, 0],
                        [0, 0, 1, 0, 0],
                        [0, 0, -1, 0, 0],
                        [0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 1],
                        [0, 0, 0, -1, 0],
                        [0, 0, 0, 0, -1]])
    
        
        h = np.array([self.l_max,
                      self.w_max,
                     -1*self.l_min,
                     -1*self.w_min,
                     np.power(np.e, self.omega*self.t_max),
                     -1*np.power(np.e, self.omega*self.t_min),
                     self.bx_max,
                     self.by_max_in,
                     -1*self.bx_min,
                     -1*self.by_max_out]) ## could be a problem if qp doesn't solve correctly
        
        tmp = [0.,0.]
        tmp[0] = (psi_current[0] - u[0])*np.power(np.e, -1*self.omega*t)
        tmp[1] = (psi_current[1] - u[1])*np.power(np.e, -1*self.omega*t)
        
        A = np.matrix([[1, 0, -1*tmp[0], 1, 0],
                        [0, 1, -1*tmp[1], 0, 1]])
        
        b = np.array([0, 0])
    
        
        P = P.astype(float)
        q = q.astype(float)
        G = G.astype(float)
        h = h.astype(float)
        A = A.astype(float)
        b = b.astype(float)
        
        x_opt = quadprog_solve_qp(P,q, G, h, A, b)
        t_end = np.log(x_opt[2])/self.omega
        
        return (x_opt[0] + u[0], x_opt[1] + u[1], t_end)


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
        x_foot_des_ground[0] = -1*(x_foot_des_air[0] - u[0])/2.0
        x_foot_des_ground[1] = -1*(x_foot_des_air[1] - u[1])/2.0
        x_foot_des_ground[2] = z_ground
        
        return x_foot_des_air, x_foot_des_ground
    
    
    
    

    