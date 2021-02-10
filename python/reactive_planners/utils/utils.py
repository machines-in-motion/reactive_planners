### This file contains utility functions for the planner
## Author : Avadesh Meduri
## Date : 7/11/2019

import numpy as np

from urdf_parser_py.urdf import URDF


def create_terrain_constraints(file_dir):
    """
    creates the contraints for the stepping QP based on the terrain details in the urdf
    Gives details of where the next step location can exist
    returns the constraints as numpy matrix.

    Input :
       file_name : name of the urdf file that describes the terrain
    """

    terrain = URDF.from_xml_file(file_dir)
    # print(terrain.links[0].visuals[0].geometry)

    b = []
    link_no = 0
    for link in terrain.links:
        name = link.name
        # Note: The assumption with origin is that everything is in the global frame.
        origin_xyz = np.array(link.visual.origin.xyz)
        geometry = np.array(link.visual.geometry.size)

        for i in range(2):
            b.append(origin_xyz[i] + geometry[i] / 2.0 - 0.05)
            b.append(origin_xyz[i] - geometry[i] / 2.0 + 0.05)

        b.append(
            origin_xyz[2] + 0.5 * geometry[2] + 0.001
        )  ## ground constraint
        b.append(
            origin_xyz[2] + 0.5 * geometry[2] - 0.001
        )  ## ground constraint

        link_no += 1

    return b


# terrain_dir = './terrains/stairs.urdf'
# G, b = create_terrain_constraints(terrain_dir)
