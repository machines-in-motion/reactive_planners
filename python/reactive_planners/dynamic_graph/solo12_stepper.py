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
from dynamic_graph.sot.core.math_small_entities import (Selec_of_matrix, Stack_of_vector)

from mim_control.dynamic_graph.wbc_graph import WholeBodyController

from reactive_planners.dynamic_graph.walking import QuadrupedDcmReactiveStepper

from dg_tools.dynamic_graph.dg_tools_entities import PoseRPYToPoseQuaternion
from dg_tools.utils import (
    constVectorOp, subtract_vec_vec, hom2pos, add_vec_vec, stack_two_vectors,
    selec_vector, zero_vec, basePoseQuat2PoseRPY, multiply_mat_vec
)

from robot_properties_solo.config import Solo12Config


class QuadrupedStepper:
    def __init__(self, prefix, pin_robot, endeff_names):
        self.prefix = prefix
        self.pin_robot = pin_robot
        self.endeff_names = endeff_names
        self.nv = pin_robot.model.nv

        self.dg_robot = dp.DynamicPinocchio(prefix + '_pinocchio')
        self.dg_robot.setModel(self.pin_robot.model)
        self.dg_robot.setData(self.pin_robot.data)

        self.sig_eff_pos = []
        self.sig_eff_vel = []

        # Create the objects of interests from pinocchio.
        self.com = self.dg_robot.signal('com')
        self.vcom = multiply_mat_vec(
            self.dg_robot.signal('Jcom'), self.dg_robot.signal('velocity'))

        for endeff_name in endeff_names:
            self.dg_robot.createPosition('pos_' + endeff_name, endeff_name)
            self.dg_robot.createJacobianEndEffWorld('jac_' + endeff_name, endeff_name)

            # Store the endeffector position signal.
            self.sig_eff_pos.append(hom2pos(self.dg_robot.signal('pos_' + endeff_name)))

            # Compute the endeffector velocity signal.
            sel_linear = Selec_of_matrix(prefix + '_pinocchio_' + endeff_name)
            sel_linear.selecRows(0, 3)
            sel_linear.selecCols(0, self.nv + 6)
            dg.plug(self.dg_robot.signal('jac_' + endeff_name), sel_linear.sin)

            self.sig_eff_vel.append(
                multiply_mat_vec(sel_linear.sout, self.dg_robot.signal('velocity'))
            )

        ###
        # Create the actual stepper object.
        self.stepper = QuadrupedDcmReactiveStepper(prefix + '_quadruped_stepper')

        # Setup the pinocchio input quantities for the stepper.
        dg.plug(self.com, self.stepper.com_position_sin)
        dg.plug(self.vcom, self.stepper.com_velocity_sin)

        dg.plug(self.sig_eff_pos[0], self.stepper.current_front_left_foot_position_sin)
        dg.plug(self.sig_eff_vel[0], self.stepper.current_front_left_foot_velocity_sin)
        dg.plug(self.sig_eff_pos[1], self.stepper.current_front_right_foot_position_sin)
        dg.plug(self.sig_eff_vel[1], self.stepper.current_front_right_foot_velocity_sin)
        dg.plug(self.sig_eff_pos[2], self.stepper.current_hind_left_foot_position_sin)
        dg.plug(self.sig_eff_vel[2], self.stepper.current_hind_left_foot_velocity_sin)
        dg.plug(self.sig_eff_pos[3], self.stepper.current_hind_right_foot_position_sin)
        dg.plug(self.sig_eff_vel[3], self.stepper.current_hind_right_foot_velocity_sin)

        self.stepper.is_closed_loop_sin.value = 0.

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
        position = stack_two_vectors(base_pose_rpy, robot.device.joint_positions, 6, self.nv - 6)
        velocity = stack_two_vectors(base_velocity, robot.device.joint_velocities, 6, self.nv- 6)

        dg.plug(position, self.dg_robot.signal('position'))
        dg.plug(velocity, self.dg_robot.signal('velocity'))
        self.dg_robot.signal('acceleration').value = np.array(self.nv * [0.,])

        ###
        # Plug the stepper base position.
        dg.plug(base_position, self.stepper.xyzquat_base_sin)


class Solo12WBCStepper:
    def __init__(self, prefix, friction_coeff):
        pin_robot = Solo12Config.buildRobotWrapper()
        end_effector_names = Solo12Config.end_effector_names

        ###
        # Create the whole body controller.
        qp_penalty_weights = np.array([1e0, 1e0, 1e6, 1e6, 1e6, 1e6])
        self.wbc = wbc = WholeBodyController(
            prefix + '_wbc', pin_robot, end_effector_names, friction_coeff, qp_penalty_weights)

        ###
        # Specify gains for the controller.

        # For the centroidal controllers.
        wbc.kc_sin.value = np.array([0., 0., 200.])
        wbc.dc_sin.value = np.array([10., 10., 10.])
        wbc.kb_sin.value = np.array([25., 25., 25.])
        wbc.db_sin.value = np.array([10., 10., 10.])

        wbc.des_com_pos_sin.value = np.array([0., 0., 0.25])
        wbc.des_com_vel_sin.value = np.zeros(3)
        wbc.des_ori_vel_sin.value = np.zeros(3)

        # Make it possible to specify desired orientation as RPY.
        op = Stack_of_vector(prefix + '_stack_pose_rpy')
        op.selec1(0, 3)
        op.selec2(0, 3)
        op.sin1.value = np.array([0., 0., 0.])

        op_rpy2quat = PoseRPYToPoseQuaternion('')
        dg.plug(op.sout, op_rpy2quat.sin)
        dg.plug(selec_vector(op_rpy2quat.sout, 3, 7), wbc.des_ori_pos_sin)

        # Expose the signals for easier access.
        self.des_ori_pos_rpy_sin = op.sin2
        self.des_ori_vel_sin = wbc.des_ori_vel_sin

        self.des_ori_pos_rpy_sin.value = np.array([0., 0., 0.])

        wbc.cnt_array_sin.value = np.array([1., 1., 1., 1.])

        # Impedance controllers.
        for i, imp in enumerate(wbc.imps):
            imp.gain_proportional_sin.value = np.array([50., 50., 50., 0., 0., 0.])
            imp.gain_derivative_sin.value = np.array([0.7, 0.7, 0.7, 0., 0., 0.])
            imp.gain_feed_forward_force_sin.value = 1.0

        wbc.w_com_ff_sin.value = np.array([0., 0., 9.81 * 2.5, 0., 0., 0.])


        ###
        # Create the stepper.
        self.stepper = stepper = QuadrupedStepper(prefix + '_stepper', pin_robot, end_effector_names)

        ###
        # Connect the stepper with the wbc.
        dg.plug(stepper.stepper.contact_array_sout, wbc.cnt_array_sin)

        def plug_des_pos(stepper_pos, imp):
            dg.plug(
                stack_two_vectors(
                    stepper_pos,
                    zero_vec(4, ''), 3, 4),
                imp.desired_end_frame_placement_sin
            )

        def plug_des_vel(stepper_pos, imp):
            dg.plug(
                stack_two_vectors(
                    stepper_pos,
                    zero_vec(3, ''), 3, 3),
                imp.desired_end_frame_velocity_sin
            )

        plug_des_pos(stepper.stepper.front_left_foot_position_sout, wbc.imps[0])
        plug_des_vel(stepper.stepper.front_left_foot_velocity_sout, wbc.imps[0])

        plug_des_pos(stepper.stepper.front_right_foot_position_sout, wbc.imps[1])
        plug_des_vel(stepper.stepper.front_right_foot_velocity_sout, wbc.imps[1])

        plug_des_pos(stepper.stepper.hind_left_foot_position_sout, wbc.imps[2])
        plug_des_vel(stepper.stepper.hind_left_foot_velocity_sout, wbc.imps[2])

        plug_des_pos(stepper.stepper.hind_right_foot_position_sout, wbc.imps[3])
        plug_des_vel(stepper.stepper.hind_right_foot_velocity_sout, wbc.imps[3])

        self.initialize()

    def initialize(self):
        # PART 1: Positions
        # Because this controller is specific for solo12, we can hard
        # code the values here.
        self.stepper.stepper.initialize_placement(
            np.array([0., 0., 0.238, 0, 0, 0., 1.]),
            np.array([ 0.195,  0.147, 0.015]),
            np.array([ 0.195, -0.147, 0.015]),
            np.array([-0.195,  0.147, 0.015]),
            np.array([-0.195, -0.147, 0.015])
        )

        # PART 2: Parameters
        is_left_leg_in_contact = True
        l_min = -0.1
        l_max = 0.1
        w_min = -0.08
        w_max = 0.2
        t_min = 0.1
        t_max = 1.0
        l_p = 0.00  # Pelvis width
        com_height = 0.25
        weight = np.array([1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000])
        mid_air_foot_height = 0.05
        control_period = 0.001
        planner_loop = 0.010

        self.stepper.stepper.initialize_stepper(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            com_height,
            weight,
            mid_air_foot_height,
            control_period,
            planner_loop,
        )

        ###
        # Let the quadruped step in place for now.
        self.des_com_vel_sin = self.stepper.stepper.desired_com_velocity_sin
        self.des_com_vel_sin.value = np.array([0., 0., 0.])

    def start(self):
        self.stepper.start()

    def stop(self):
        self.stepper.stop()


    def plug(self, robot, base_position, base_velocity):
        self.wbc.plug(robot, base_position, base_velocity)
        self.stepper.plug(robot, base_position, base_velocity)
