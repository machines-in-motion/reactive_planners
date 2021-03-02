"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date:   Feb 26, 2021
"""

import numpy as np
np.set_printoptions(suppress=True, precision=3)

import pinocchio as pin

import dynamic_graph as dg

from robot_properties_solo.config import Solo12Config

from reactive_planners.dynamic_graph.solo12_stepper import Solo12WBCStepper

from dg_tools.utils import (
    stack_two_vectors, selec_vector, subtract_vec_vec
)
from dg_tools.dynamic_graph.dg_tools_entities import CreateWorldFrame


if "robot" in globals():
    from dg_demos.solo12.controllers.pd_controller import get_controller as get_pd_controller
    from dg_vicon_sdk.dynamic_graph.entities import ViconClientEntity

    # Init vicon.
    vicon = ViconClientEntity('vicon_entity')
    vicon.connect_to_vicon('172.24.117.119:801') # NYU MIM vicon.
    vicon.add_object_to_track('solo12/solo12')

    # Create a PD controller to setup the robot at the beginning.
    pd_ctrl = get_pd_controller()

    # Setup the main controller.
    ctrl = Solo12WBCStepper('solo12_wbc_stepper', 0.2)

    # Zero the initial position from the vicon signal.
    base_posture_sin = vicon.signal('solo12_position')

    op = CreateWorldFrame('wf')
    dg.plug(base_posture_sin, op.frame_sin)
    op.update()
    op.set_which_dofs(np.array([1., 1., 0., 0., 0., 0.]))

    base_posture_local_sin = stack_two_vectors(
        selec_vector(subtract_vec_vec(base_posture_sin, op.world_frame_sout), 0, 3),
        selec_vector(base_posture_sin, 3, 7), 3, 4)

    # Create the base velocity using the IMU.
    base_velocity_sin = stack_two_vectors(
        selec_vector(vicon.signal('solo12_velocity_body'), 0, 3),
        robot.device.imu_gyroscope, 3, 3)

    # Set desired base rotation and velocity.
    des_yaw = 0.
    ctrl.des_ori_pos_rpy_sin.value = np.array([0., 0., des_yaw])
    ctrl.des_com_vel_sin.value = np.array([0.0, 0.0, 0.])

    def go_pd():
        pd_ctrl.plug(robot)

    def go_stepper():
        op.update()
        ctrl.plug(robot, base_posture_local_sin, base_velocity_sin)
