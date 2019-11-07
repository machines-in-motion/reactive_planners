### This file contains utility functions for the planner
## Author : Avadesh Meduri
## Date : 7/11/2019

import numpy as np




def create_terrain_constraints(file_name):
    '''
     creates the contraints for the stepping QP based on the terrain details in the urdf 
     Gives details of where the next step location can exist
     returns the constraints as numpy matrix.

     Input :
        file_name : name of the urdf file that describes the terrain   
    '''
    
