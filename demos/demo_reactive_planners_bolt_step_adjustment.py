""" @namespace Demos of Bolt step adjustment
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import numpy as np
import pybullet as p
from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot
from mim_control.robot_centroidal_controller import RobotCentroidalController
from mim_control.robot_impedance_controller import RobotImpedanceController
from reactive_planners.lipm_simulator import LipmSimpulator
from reactive_planners_cpp import DcmReactiveStepper
import pinocchio as se3
from scipy.spatial.transform import Rotation as R
from bullet_utils.env import BulletEnvWithGround


def zero_cnt_gain(kp, cnt_array):
    gain = np.array(kp).copy()
    for i, v in enumerate(cnt_array):
        if v == 1:
            gain[3 * i : 3 * (i + 1)] = 0.0
    return gain


def yaw(q):
    return np.array(
        R.from_quat([np.array(q)[3:7]]).as_euler("xyz", degrees=False)
    )[0, 2]


if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround()
    robot = env.add_robot(BoltRobot())
    tau = np.zeros(6)

    time = 0
    sim_freq = 10000  # Hz
    ctrl_freq = 1000
    plan_freq = 1000

    p.resetDebugVisualizerCamera(1.6, 50, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(1.0 / sim_freq)
    p.setRealTimeSimulation(0)
    for ji in range(8):
        p.changeDynamics(
            robot.robotId,
            ji,
            linearDamping=0.04,
            angularDamping=0.04,
            restitution=0.0,
            lateralFriction=4.0,
            spinningFriction=5.6,
        )

    q = np.matrix(BoltConfig.initial_configuration).T
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    warmup = 5
    kp = np.array([150.0, 150.0, 150.0, 150.0, 150.0, 150.0])
    kd = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    robot_config = BoltConfig()
    config_file = robot_config.ctrl_path
    bolt_leg_ctrl = RobotImpedanceController(robot, config_file)
    centr_controller = RobotCentroidalController(
        robot_config,
        mu=1,
        kc=[0, 0, 100],
        dc=[0, 0, 10],
        kb=[100, 100, 100],
        db=[10.0, 10, 10],
        qp_penalty_lin=[1, 1, 1e6],
        qp_penalty_ang=[1e6, 1e6, 1],
    )
    is_left_leg_in_contact = True
    l_min = -0.1
    l_max = 0.1
    w_min = -0.08
    w_max = 0.2
    t_min = 0.1
    t_max = 0.8
    l_p = 0.1035  # Pelvis width
    com_height = 0.36487417
    weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
    mid_air_foot_height = 0.05
    control_period = 0.001
    planner_loop = 0.010
    x_des_local = [
        q[0].item(),
        q[1].item() + 0.02,
        0.0,
        q[0].item(),
        q[1].item() - 0.02,
        0.0,
    ]
    past_x = x_des_local.copy()
    v_des = [0.0, 0.0, 0.0]
    sim = LipmSimpulator(com_height)
    dcm_reactive_stepper = DcmReactiveStepper()
    dcm_reactive_stepper.initialize(
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
        x_des_local[:3],
        x_des_local[3:],
    )

    dcm_reactive_stepper.set_desired_com_velocity(v_des)

    x_com = [[0.0], [0.0], [com_height]]
    cnt_array = [1, 1]
    time = 0
    control_time = 0
    open_loop = True
    dcm_force = [0.0, 0.0, 0.0]
    offset = 0.0171  # foot radius
    dcm_reactive_stepper.start()

    for i in range(5005):
        last_qdot = qdot
        q, qdot = robot.get_state()
        robot.pin_robot.com(q, qdot)
        x_com = robot.pin_robot.com(q, qdot)[0]
        xd_com = robot.pin_robot.com(q, qdot)[1]

        if warmup <= i:
            left = bolt_leg_ctrl.imp_ctrl_array[0]
            right = bolt_leg_ctrl.imp_ctrl_array[1]
            left_foot_location = np.array(
                left.pin_robot.data.oMf[left.frame_end_idx].translation
            ).reshape(-1)
            right_foot_location = np.array(
                right.pin_robot.data.oMf[right.frame_end_idx].translation
            ).reshape(-1)
            left_foot_vel = np.array(
                se3.SE3(
                    left.pin_robot.data.oMf[left.frame_end_idx].rotation,
                    np.zeros((3, 1)),
                )
                * se3.computeFrameJacobian(
                    robot.pin_robot.model,
                    robot.pin_robot.data,
                    q,
                    left.frame_end_idx,
                ).dot(qdot)[0:3]
            )
            right_foot_vel = np.array(
                se3.SE3(
                    right.pin_robot.data.oMf[right.frame_end_idx].rotation,
                    np.zeros((3, 1)),
                )
                * se3.computeFrameJacobian(
                    robot.pin_robot.model,
                    robot.pin_robot.data,
                    q,
                    right.frame_end_idx,
                ).dot(qdot)[0:3]
            )
            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                pos_for_plotter = (
                    dcm_reactive_stepper.get_right_foot_position().copy()
                )
                vel_for_plotter = (
                    dcm_reactive_stepper.get_right_foot_velocity().copy()
                )
            else:
                pos_for_plotter = (
                    dcm_reactive_stepper.get_left_foot_position().copy()
                )
                vel_for_plotter = (
                    dcm_reactive_stepper.get_left_foot_velocity().copy()
                )

            dcm_reactive_stepper.run(
                time,
                [
                    left_foot_location[0],
                    left_foot_location[1],
                    left_foot_location[2] - offset,
                ],
                [
                    right_foot_location[0],
                    right_foot_location[1],
                    right_foot_location[2] - offset,
                ],
                left_foot_vel,
                right_foot_vel,
                x_com,
                xd_com,
                yaw(q),
                not open_loop,
            )
            dcm_force = dcm_reactive_stepper.get_forces().copy()

            x_des_local = []
            x_des_local.extend(
                dcm_reactive_stepper.get_left_foot_position().copy()
            )
            x_des_local.extend(
                dcm_reactive_stepper.get_right_foot_position().copy()
            )

            x_des_local[2] += offset
            x_des_local[5] += offset

            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                cnt_array = [1, 0]
            else:
                cnt_array = [0, 1]
            time += 0.001

        for j in range(2):
            imp = bolt_leg_ctrl.imp_ctrl_array[j]
            x_des_local[3 * j : 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation
        w_com = centr_controller.compute_com_wrench(
            q.copy(),
            qdot.copy(),
            [0.0, 0.0, com_height],
            [0.0, 0.0, 0.0],
            [0, 0.0, 0, 1.0],
            [0.0, 0.0, 0.0],
        )
        w_com[0] = 0.0
        w_com[1] = 0.0

        F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

        des_vel = np.concatenate(
            (
                dcm_reactive_stepper.get_left_foot_velocity()
                - [qdot[0].item(), qdot[1].item(), qdot[2].item()],
                dcm_reactive_stepper.get_right_foot_velocity()
                - [qdot[0].item(), qdot[1].item(), qdot[2].item()],
            )
        )

        if cnt_array[0] == 1 and cnt_array[1] == 0:
            F[3:] = -dcm_force[:3]
        elif cnt_array[0] == 0 and cnt_array[1] == 1:
            F[:3] = -dcm_force[:3]
        tau = bolt_leg_ctrl.return_joint_torques(
            q.copy(),
            qdot.copy(),
            zero_cnt_gain(kp, cnt_array),
            zero_cnt_gain(kd, cnt_array),
            x_des_local,
            des_vel,
            F,
        )
        control_time += 0.001

        for j in range(10):
            robot.send_joint_command(tau)
            p.stepSimulation()

    dcm_reactive_stepper.stop()
