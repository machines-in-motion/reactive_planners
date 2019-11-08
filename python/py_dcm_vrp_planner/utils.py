### This file contains utility functions for the planner
## Author : Avadesh Meduri
## Date : 7/11/2019

import numpy as np

from urdf_parser_py.urdf import URDF


def create_terrain_constraints(file_dir):
    '''
     creates the contraints for the stepping QP based on the terrain details in the urdf 
     Gives details of where the next step location can exist
     returns the constraints as numpy matrix.

     Input :
        file_name : name of the urdf file that describes the terrain   
    '''
    
    terrain = URDF.from_xml_file(file_dir)
    # print(terrain.links[0].visuals[0].geometry)    

    parent_map = terrain.parent_map()
    
    
    for link in terrain.links:
        name = link.name
        # Note: The assumption with origin is that every thing is in the global frame. 
        origin_xyz = np.array(link.visuals[0].origin.xyz)
        origin_rpy = np.array(link.visuals[0].origin.rpy)        
        geometry = np.array(link.visuals[0].geometry.size)


#### test


terrain_dir = './terrains/stairs.urdf'
create_terrain_constraints(terrain_dir)