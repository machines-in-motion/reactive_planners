"""Demo file for running the reactive planner on SOLO 12
The logic is the same as the demo_reactive_planners_solo12_step_adjustment_walk.ipynb
This file is used for ROS 2. An example of the usage of this class is here: 
https://github.com/swuuu/ros2_controllers_solo12/blob/master/ros2_control_test_nodes/ros2_control_test_nodes/node.py

License BSD-3-Clause
Copyright (c) 2023, New York University and Max Planck Gesellschaft.

Author: Stanley Wu
Date:   Jan 31, 2023
"""

import numpy as np
from robot_properties_solo.solo12wrapper import Solo12Config
from robot_properties_solo.solo12wrapper import Solo12RobotWithoutPybullet
from mim_control.robot_impedance_controller import RobotImpedanceController
from mim_control.robot_centroidal_controller import RobotCentroidalController
from reactive_planners_cpp import QuadrupedDcmReactiveStepper

import pinocchio as pin
from scipy.spatial.transform import Rotation
np.set_printoptions(suppress=True, precision=2)

class Demo:
    """
    Based on https://github.com/machines-in-motion/reactive_planners/blob/master/demos/demo_reactive_planners_solo12_step_adjustment_walk.ipynb
    The 4 parameters to control the direction of SOLO are yaw_velocity_des, yaw_des, com_des, and v_des.
    """

    def __init__(self):
        pin.switchToNumpyArray()

        # Create a robot instance.
        self.robot = Solo12RobotWithoutPybullet()

        # Initialize RobotImpedanceController
        self.kp = np.array(12 * [100.0])
        self.kd = 12 * [5.0]
        robot_config = Solo12Config()
        config_file = robot_config.ctrl_path
        self.solo_leg_ctrl = RobotImpedanceController(self.robot, config_file)

        # Initialize RobotCentroidalController
        self.centr_controller = RobotCentroidalController(
            robot_config,
            mu=0.6,
            kc=[0, 0, 200],
            dc=[10, 10, 10],
            kb=[25, 25, 25.],
            db=[22.5, 22.5, 22.5],
            qp_penalty_lin=[1e0, 1e0, 1e6],
            qp_penalty_ang=[1e6, 1e6, 1e6],
        )

        # Parameters for the reactive stepper
        self.is_left_leg_in_contact = True
        self.l_min = -0.1
        self.l_max = 0.1
        self.w_min = -0.08
        self.w_max = 0.2
        self.t_min = 0.1
        self.t_max = 1.0
        self.l_p = 0.00  # Pelvis width
        self.weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
        self.mid_air_foot_height = 0.05
        self.control_period = 0.001
        self.planner_loop = 0.010

        # Initialize the reactive stepper
        self.quadruped_dcm_reactive_stepper = QuadrupedDcmReactiveStepper()

        # More fields
        self.cnt_array = [1, 1]
        self.open_loop = True
        self.dcm_force = np.array([0.0, 0.0, 0.0])
        self.offset = 0.015  # foot radius

        # For tracking feet positions
        self.x_curr_local = None
        self.x_des_local = None
        self.feet_pos_des = None
        self.com_pos = None

        # For tracking the feet position
        self.print_once = True
        self.curr_quat = None

    def zero_cnt_gain(self, kp, cnt_array):
        gain = np.array(kp).copy()
        for i, v in enumerate(cnt_array):
            if v == 1:
                gain[3 * i: 3 * (i + 1)] = 0.0
        return gain

    def yaw(self, q):
        return np.array(
            Rotation.from_quat([np.array(q)[3:7]]).as_euler("xyz", degrees=False)
        )[0, 2]

    def initialize_quadruped_dcm_reactive_stepper(self, q, action="forward"):
        """
        Initializes the quadruped_dcm_reactive_stepper
        :param q: joint configuration
        :param action: either "forward", "right", "left", "turn_right", "turn_left", "stay" (anything else will be equivalent to stay)
        """
        self.com_height = q[2]

        # init poses
        self.robot.pin_robot.framesForwardKinematics(q)
        base_pose = q[:7]
        front_left_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[0].frame_end_idx].translation
        front_right_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[1].frame_end_idx].translation
        hind_left_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[2].frame_end_idx].translation
        hind_right_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[3].frame_end_idx].translation

        # initialize quadruped_dcm_reactive_stepper
        self.quadruped_dcm_reactive_stepper.initialize(
            self.is_left_leg_in_contact,
            self.l_min,
            self.l_max,
            self.w_min,
            self.w_max,
            self.t_min,
            self.t_max,
            self.l_p,
            self.com_height,
            self.weight,
            self.mid_air_foot_height,
            self.control_period,
            self.planner_loop,
            base_pose,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position,
        )

        # default parameters related to the direction of SOLO
        self.yaw_velocity_des = 0.0 # default yaw speed
        self.yaw_des = self.yaw(q) # default desired yaw
        self.com_des = np.array([q[0], q[1]]) # default desired com position
        self.v_des = np.array([0.0, 0.0, 0.0])
        if action == "forward":
            self.v_des[0] = 0.2
        elif action == "left":
            self.v_des[1] = 0.2
        elif action == "right":
            self.v_des[1] = -0.2

        self.quadruped_dcm_reactive_stepper.set_desired_com_velocity(self.v_des)
        self.quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory()

    def quadruped_dcm_reactive_stepper_start(self):
        self.quadruped_dcm_reactive_stepper.start()

    def quadruped_dcm_reactive_stepper_stop(self):
        self.quadruped_dcm_reactive_stepper.stop()

    def compute_torques(self, q, qdot, control_time, action=""):
        """
        Reactive planner script
        :param q: vector of 19 elements composed on [base position (3 elements), base orientation (4 elements), joint configurations (12 elements)]
        :param qdot: vector of 18 elements composed of [base velocity (3), base ang vel (3), joint velocities (12)]
        :param control_time: parameter for self.quadruped_dcm_reactive_stepper
        :param action: either "forward", "right", "left", "turn_right", "turn_left", "stay" (anything else will be equivalent to stay)
        :return: torques to send to SOLO
        """
        # normalize the quat (maybe not needed)
        normalized_quat = pin.Quaternion(
            q[6], q[3], q[4], q[5]
        ).normalized()
        q[3] = normalized_quat[0]
        q[4] = normalized_quat[1]
        q[5] = normalized_quat[2]
        q[6] = normalized_quat[3]

        # transform the base velocity to the local frame
        curr_orientation = normalized_quat.matrix()
        qdot[:3] = curr_orientation.T.dot(qdot[:3])
        qdot[3:6] = curr_orientation.T.dot(qdot[3:6])

        # update pinocchio
        self.robot.pin_robot.com(q, qdot)
        self.robot.update_pinocchio(q, qdot)
        x_com = self.robot.pin_robot.com(q, qdot)[0]
        xd_com = self.robot.pin_robot.com(q, qdot)[1]

        # update the relevant params for the centroidal controller depending on the command
        if action == "forward":
            self.com_des[0] = q[0] + self.v_des[0] * 0.001
        elif action == "left" or action == "right":
            self.com_des[1] = q[1] + self.v_des[1] * 0.001
        elif action == "turn_right":
            self.yaw_velocity_des = 0.2
            self.yaw_des += 0.001 * self.yaw_velocity_des
        elif action == "turn_left":
            self.yaw_velocity_des = 0.2
            self.yaw_des -= 0.001 * self.yaw_velocity_des

        FL = self.solo_leg_ctrl.imp_ctrl_array[0]
        FR = self.solo_leg_ctrl.imp_ctrl_array[1]
        HL = self.solo_leg_ctrl.imp_ctrl_array[2]
        HR = self.solo_leg_ctrl.imp_ctrl_array[3]

        # Define left as front left and back right leg
        front_left_foot_position = self.robot.pin_robot.data.oMf[FL.frame_end_idx].translation
        front_right_foot_position = self.robot.pin_robot.data.oMf[FR.frame_end_idx].translation
        hind_left_foot_position = self.robot.pin_robot.data.oMf[HL.frame_end_idx].translation
        hind_right_foot_position = self.robot.pin_robot.data.oMf[HR.frame_end_idx].translation
        front_left_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, FL.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        front_right_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, FR.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        hind_left_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, HL.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        hind_right_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, HR.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear

        self.quadruped_dcm_reactive_stepper.run(
            control_time,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position,
            front_left_foot_velocity,
            front_right_foot_velocity,
            hind_left_foot_velocity,
            hind_right_foot_velocity,
            x_com,
            xd_com,
            self.yaw(q),
            not self.open_loop,
        )

        x_des_local = []
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_front_left_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_front_right_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_hind_left_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_hind_right_foot_position())

        cnt_array = self.quadruped_dcm_reactive_stepper.get_contact_array()

        for j in range(4):
            imp = self.solo_leg_ctrl.imp_ctrl_array[j]
            x_des_local[3 * j: 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation

        # compute the wrench
        w_com = self.centr_controller.compute_com_wrench(
            q.copy(),
            qdot.copy(),
            [self.com_des[0], self.com_des[1], self.com_height],
            self.v_des,
            pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., self.yaw_des)).coeffs(),
            [0.0, 0.0, self.yaw_velocity_des], # angular velocity desired
        )

        # # for tracking the feet position
        # if self.x_curr_local is None:
        #     self.x_curr_local = np.array([front_left_foot_position.copy()])
        #     self.x_des_local = np.array([self.quadruped_dcm_reactive_stepper.get_front_left_foot_position().copy()])
        #     self.feet_pos_des = np.array([self.get_desired_next_step_pos(q)[0].copy()])
        #     # self.com_pos = np.array([[q[0], q[1], q[2]]])
        #     # self.curr_quat = np.array([q[3:7].copy()])
        #     self.com_pos = np.array([[w_com[0], w_com[1], w_com[2]]])
        # elif self.x_des_local.shape[0] < 3000:
        #     self.x_curr_local = np.concatenate((self.x_curr_local, np.array([front_left_foot_position.copy()])))
        #     self.x_des_local = np.concatenate((self.x_des_local, np.array([self.quadruped_dcm_reactive_stepper.get_front_left_foot_position().copy()])))
        #     self.feet_pos_des = np.concatenate((self.feet_pos_des, np.array([self.get_desired_next_step_pos(q)[0].copy()])))
        #     # self.com_pos = np.concatenate((self.com_pos, np.array([[q[0], q[1], q[2]]])))
        #     self.com_pos = np.concatenate((self.com_pos, np.array([[w_com[0], w_com[1], w_com[2]]])))
        #     # self.curr_quat = np.concatenate((self.curr_quat, np.array([q[3:7]])))

        # compte the distributing forces
        F = self.centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

        des_vel = np.concatenate(
            (
                self.quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity(),
            )
        )

        if cnt_array[0] == 1:
            F[3:6] = -self.dcm_force[:3]
            F[6:9] = -self.dcm_force[:3]
        else:
            F[0:3] = -self.dcm_force[:3]
            F[9:12] = -self.dcm_force[:3]

        # compute the torques
        tau = self.solo_leg_ctrl.return_joint_torques(
            q.copy(),
            qdot.copy(),
            self.zero_cnt_gain(self.kp, cnt_array),
            self.zero_cnt_gain(self.kd, cnt_array),
            x_des_local,
            des_vel,
            F,
        )

        return tau

    # for tracking the feet position
    def get_pos_feet(self):
        return self.x_curr_local, self.x_des_local, self.feet_pos_des, self.com_pos

    def get_desired_next_step_pos(self, q):
        return self.quadruped_dcm_reactive_stepper.get_next_support_feet_positions(self.yaw(q))

