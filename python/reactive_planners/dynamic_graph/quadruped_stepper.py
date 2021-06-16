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

import dynamic_graph.sot.dynamic_pinocchio as dp
from dynamic_graph.sot.core.math_small_entities import (
    Selec_of_matrix,
    Stack_of_vector,
)

from mim_control.dynamic_graph.wbc_graph import WholeBodyController

from reactive_planners.dynamic_graph.walking import QuadrupedDcmReactiveStepper

from dg_tools.dynamic_graph.dg_tools_entities import PoseRPYToPoseQuaternion
from dg_tools.utils import (
    constVectorOp,
    subtract_vec_vec,
    hom2pos,
    add_vec_vec,
    stack_two_vectors,
    selec_vector,
    zero_vec,
    basePoseQuat2PoseRPY,
    multiply_mat_vec,
)

from robot_properties_solo.config import Solo12Config


class QuadrupedStepper:
    def __init__(self, prefix, pin_robot, endeff_names):
        self.prefix = prefix
        self.pin_robot = pin_robot
        self.endeff_names = endeff_names
        self.nv = pin_robot.model.nv

        self.dg_robot = dp.DynamicPinocchio(prefix + "_pinocchio")
        self.dg_robot.setModel(self.pin_robot.model)
        self.dg_robot.setData(self.pin_robot.data)

        self.sig_eff_pos = []
        self.sig_eff_vel = []

        # Create the objects of interests from pinocchio.
        self.com = self.dg_robot.signal("com")
        self.vcom = multiply_mat_vec(
            self.dg_robot.signal("Jcom"), self.dg_robot.signal("velocity")
        )

        for endeff_name in endeff_names:
            self.dg_robot.createPosition("pos_" + endeff_name, endeff_name)
            self.dg_robot.createJacobianEndEffWorld(
                "jac_" + endeff_name, endeff_name
            )

            # Store the endeffector position signal.
            self.sig_eff_pos.append(
                hom2pos(self.dg_robot.signal("pos_" + endeff_name))
            )

            # Compute the endeffector velocity signal.
            sel_linear = Selec_of_matrix(prefix + "_pinocchio_" + endeff_name)
            sel_linear.selecRows(0, 3)
            sel_linear.selecCols(0, self.nv + 6)
            dg.plug(self.dg_robot.signal("jac_" + endeff_name), sel_linear.sin)

            self.sig_eff_vel.append(
                multiply_mat_vec(
                    sel_linear.sout, self.dg_robot.signal("velocity")
                )
            )

        ###
        # Create the actual stepper object.
        self.stepper = QuadrupedDcmReactiveStepper(
            prefix + "_quadruped_stepper"
        )

        # Setup the pinocchio input quantities for the stepper.
        dg.plug(self.com, self.stepper.com_position_sin)
        dg.plug(self.vcom, self.stepper.com_velocity_sin)

        dg.plug(
            self.sig_eff_pos[0],
            self.stepper.current_front_left_foot_position_sin,
        )
        dg.plug(
            self.sig_eff_vel[0],
            self.stepper.current_front_left_foot_velocity_sin,
        )
        dg.plug(
            self.sig_eff_pos[1],
            self.stepper.current_front_right_foot_position_sin,
        )
        dg.plug(
            self.sig_eff_vel[1],
            self.stepper.current_front_right_foot_velocity_sin,
        )
        dg.plug(
            self.sig_eff_pos[2],
            self.stepper.current_hind_left_foot_position_sin,
        )
        dg.plug(
            self.sig_eff_vel[2],
            self.stepper.current_hind_left_foot_velocity_sin,
        )
        dg.plug(
            self.sig_eff_pos[3],
            self.stepper.current_hind_right_foot_position_sin,
        )
        dg.plug(
            self.sig_eff_vel[3],
            self.stepper.current_hind_right_foot_velocity_sin,
        )

        self.stepper.is_closed_loop_sin.value = 0.0

    def start(self):
        self.stepper.start()

    def stop(self):
        self.stepper.stop()

    def plug(self, robot, base_position, base_velocity):
        # Args:
        #   robot; DGM robot device
        #   base_position: The base position as a 7 dim vector signal
        #   base_velocity: The base velocity as a 6 dim vector signal

        ###
        # Plug the pinocchio entity.
        base_pose_rpy = basePoseQuat2PoseRPY(base_position)
        position = stack_two_vectors(
            base_pose_rpy, robot.device.joint_positions, 6, self.nv - 6
        )
        velocity = stack_two_vectors(
            base_velocity, robot.device.joint_velocities, 6, self.nv - 6
        )

        dg.plug(position, self.dg_robot.signal("position"))
        dg.plug(velocity, self.dg_robot.signal("velocity"))
        self.dg_robot.signal("acceleration").value = np.array(
            self.nv
            * [
                0.0,
            ]
        )

        ###
        # Plug the stepper base position.
        dg.plug(base_position, self.stepper.xyzquat_base_sin)

    def trace(self, robot):
        robot.add_trace(self.stepper.name, "front_left_foot_position_sout")
        robot.add_trace(self.stepper.name, "front_right_foot_position_sout")
        robot.add_trace(self.stepper.name, "hind_left_foot_position_sout")
        robot.add_trace(self.stepper.name, "hind_right_foot_position_sout")
        robot.add_trace(self.stepper.name, "front_left_foot_velocity_sout")
        robot.add_trace(self.stepper.name, "front_right_foot_velocity_sout")
        robot.add_trace(self.stepper.name, "hind_left_foot_velocity_sout")
        robot.add_trace(self.stepper.name, "hind_right_foot_velocity_sout")
        robot.add_trace(self.stepper.name, "contact_array_sout")
        robot.add_trace(self.stepper.name, "swing_foot_forces_sout")
        robot.add_trace(self.stepper.name, "step_duration_sout")
