#!/usr/bin/env python

""" @namespace Controller using the dcm_vrp_planner.
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
"""

import numpy as np
from reactive_planners_cpp import (
    StepperHead,
    DcmVrpPlanner,
    EndEffectorTrajectory3D,
)
import reactive_planners_cpp
from enum import Enum
from math import exp


class contact(Enum):
    right = 0
    left = 1
    double_support = 2
    flight_r = 3
    flight_l = 4

EPSILON = 1e-6

class DcmReactiveStepper(object):
    def __init__(
        self,
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
        t_s,
        omega,
        previous_support_foot=None,
        current_support_foot=None,
    ):
        if previous_support_foot is None:
            previous_support_foot = np.zeros((3, 1))
            previous_support_foot[:] = [[0.0], [0.0], [0.0]]
        if current_support_foot is None:
            current_support_foot = np.zeros((3, 1))
            current_support_foot[:] = [[0.0], [0.0], [0.0]]
        self.control_period = control_period
        # Create the stepper head.
        self.stepper_head = StepperHead()
        self.previous_support_foot = np.zeros((3, 1))
        self.current_support_foot = np.zeros((3, 1))
        self.previous_support_foot[:] = previous_support_foot
        self.current_support_foot[:] = current_support_foot
        self.stepper_head.set_support_feet_pos(
            self.previous_support_foot, self.current_support_foot
        )
        # Create the dcm vrp planner and initialize it.
        self.omega = omega
        self.t_s = t_s
        self.g = 9.81
        self.dcm_vrp_planner = DcmVrpPlanner()
        self.t_f = 2 * self.omega* (exp(self.omega * self.t_s) - 1)/ \
                   (self.g * (exp(self.omega * t_s) + 1)) * \
                   (com_height - self.g / (self.omega ** 2))
        self.dcm_vrp_planner.initialize(
            l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, self.omega, self.t_s, weight,
        )

        # Parameters
        self.is_left_leg_in_contact = is_left_leg_in_contact
        self.duration_of_stance_phase = 0.0
        self.duration_of_flight_phase = 0.0
        self.time_from_last_step_touchdown = self.t_s + self.t_f#non_zero
        self.des_com_vel = np.zeros((3, 1))
        self.right_foot_position = np.zeros((3, 1))
        self.right_foot_position[:] = previous_support_foot
        self.right_foot_velocity = np.zeros((3, 1))
        self.right_foot_acceleration = np.zeros((3, 1))
        self.flying_foot_position = np.zeros((3, 1))
        self.flying_foot_position[:] = previous_support_foot
        self.left_foot_position = np.zeros((3, 1))
        self.left_foot_position[:] = current_support_foot
        self.left_foot_velocity = np.zeros((3, 1))
        self.left_foot_acceleration = np.zeros((3, 1))
        self.feasible_velocity = np.zeros((3, 1))
        self.swing_foot = np.zeros((3, 1))
        self.b = np.zeros((3, 1))
        self.stepper_head.set_is_left_leg_in_contact(self.is_left_leg_in_contact)

    def set_end_eff_traj_costs(
        self, cost_x, cost_y, cost_z, hess_regularization
    ):
        self.end_eff_traj3d.set_costs(
            cost_x, cost_y, cost_z, hess_regularization
        )

    def set_des_com_vel(self, des_com_vel):
        self.des_com_vel = des_com_vel.copy()

    def run(
        self,
        time,
        current_flying_foot_position,
        current_support_foot_position,
        com_position,
        com_velocity,
        base_yaw,
    ):
        # print("IS_SSSSSSSSS/FFFFFFFFF", self.is_left_leg_in_contact)
        if current_support_foot_position is not None:
            self.stepper_head.set_support_feet_pos(
                self.stepper_head.get_previous_support_location(),
                current_support_foot_position,
            )
        self.stepper_head.run(
            self.duration_of_stance_phase,
            self.duration_of_flight_phase,
            com_velocity[2],
            com_position[2] + com_velocity[2] / self.omega,
            current_flying_foot_position,
            time,
        )

        self.time_from_last_step_touchdown = (
            self.stepper_head.get_time_from_last_step_touchdown()
        )
        self.current_support_foot[
            :
        ] = self.stepper_head.get_current_support_location().reshape((3, 1))
        self.previous_support_foot[
            :
        ] = self.stepper_head.get_previous_support_location().reshape((3, 1))

        self.is_left_leg_in_contact = (
            self.stepper_head.get_is_left_leg_in_contact()
        )
        if self.time_from_last_step_touchdown - EPSILON <= self.duration_of_stance_phase:
            # print("SSSSSSSSSSSSSSSSS", self.is_left_leg_in_contact)
            self.dcm_vrp_planner.update(
                self.stepper_head.get_current_support_location(),
                self.stepper_head.get_time_from_last_step_touchdown(),
                reactive_planners_cpp.contact(self.is_left_leg_in_contact),
                self.des_com_vel,
                com_position,
                com_velocity,
                base_yaw,
                self.omega
            )
            self.projected_com = com_position.copy()
            self.projected_vcom = com_velocity.copy()
        else:
            # print("FFFFFFFFFFFFFFFFFFFFFFFFF", self.is_left_leg_in_contact)
            self.dcm_vrp_planner.update(
                self.stepper_head.get_current_support_location(),
                self.stepper_head.get_time_from_last_step_touchdown(),
                reactive_planners_cpp.contact(self.is_left_leg_in_contact + 3),
                self.des_com_vel,
                self.projected_com,
                self.projected_vcom,
                base_yaw,
                self.omega
            )
        print("is_left_leg_in_contact", self.is_left_leg_in_contact)
        assert self.dcm_vrp_planner.solve()

        self.duration_of_stance_phase = (
            self.dcm_vrp_planner.get_duration_of_stance_phase()
        )
        self.duration_of_flight_phase = (
            self.dcm_vrp_planner.get_duration_of_flight_phase()
        )
        self.swing_foot = self.dcm_vrp_planner.get_next_step_location().copy()
        # self.swing_foot[1] = 0.1235 / 2 * ((-1) ** (self.is_left_leg_in_contact + 1))
        # print("self.dcm_vrp_planner.get_next_step_location() ", self.dcm_vrp_planner.get_next_step_location())

        # Compute the feasible velocity.
        self.feasible_velocity = (
            self.dcm_vrp_planner.get_next_step_location()
            - self.stepper_head.get_previous_support_location()
        )
        self.feasible_velocity[2] = 0.0
        return


if __name__ == "__main__":

    is_left_leg_in_contact = True
    l_min = -0.5
    l_max = 0.5
    w_min = -0.5
    w_max = 0.5
    t_min = 0.1
    t_max = 0.2
    l_p = 0.1235 * 2
    com_height = 0.26487417
    weight = [1, 1, 5, 100, 100, 100, 100, 100, 100]
    mid_air_foot_height = 0.05
    control_period = 0.001
    dcm_reactive_stepper = DcmReactiveStepper(
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
    )

    time = 0.0
    current_support_foot = np.array([0, 0, 0])
    com_position = np.array([0, 0, com_height])
    com_velocity = np.array([0, 0, 0])
    base_yaw = 0.0

    dcm_reactive_stepper.run(
        time, current_support_foot, com_position, com_velocity, base_yaw
    )

    dcm_reactive_stepper.right_foot_position
    dcm_reactive_stepper.right_foot_velocity
    dcm_reactive_stepper.right_foot_acceleration
    dcm_reactive_stepper.left_foot_position
    dcm_reactive_stepper.left_foot_velocity
    dcm_reactive_stepper.left_foot_acceleration
