""" @namespace Demos of Bolt step adjustment
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import numpy as np
import pybullet as p
from matplotlib import pyplot as plt
from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot
from mim_control.robot_centroidal_controller import RobotCentroidalController
from mim_control.robot_impedance_controller import RobotImpedanceController
from mim_control.qp_solver import quadprog_solve_qp
from reactive_planners_cpp import DcmReactiveStepper
import pinocchio as se3
from pinocchio import RobotWrapper
from pinocchio.utils import zero, eye
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv, pinv
from math import sqrt
from random import random
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


def plot(f):
    if is_left_leg_in_contact:
        M = [[0.045, -0.0, 0.0], [-0.0, 0.045, -0.0], [0.0, -0.0, 0.09]]
    else:
        M = [[0.045, 0.0, 0.0], [0.0, 0.045, 0.0], [0.0, 0.0, 0.09]]
    M_inv = inv(M)
    x2 = []
    x3 = []
    v = []
    time = 0.010
    A = np.matrix(
        [
            [1.0, time, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, time, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, time],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]
    )
    B = np.matrix(
        [
            [time * time / 2, 0.0, 0.0],
            [time, 0.0, 0.0],
            [0.0, time * time / 2, 0.0],
            [0.0, time, 0.0],
            [0.0, 0.0, time * time / 2],
            [0.0, 0.0, time],
        ]
    )
    x0 = pos_for_plotter
    v0 = vel_for_plotter
    x2.append(x0)
    x3.append(x0)
    h = np.array([-0.4, 0.0, 0.8])
    for i in range(len(f) / 3):
        x2.append(
            0.5 * (f[i * 3 : i * 3 + 3] - h).dot(M_inv) * time * time
            + x0
            + v0 * time
        )
        sum = pos_for_plotter + vel_for_plotter * (i + 1) * time
        final = B
        for k in range(i + 1):
            sum[:] += np.array(
                final
                * np.matrix(
                    f[(i - k) * 3 : (i - k) * 3 + 3].dot(M_inv)
                ).transpose()
            )[::2, 0]
            sum[:] += np.array(final * np.matrix(-h.dot(M_inv)).transpose())[
                      ::2, 0
                      ]
            final = A * final
        x3.append(sum)
        x0 = (
                0.5 * (f[i * 3 : i * 3 + 3] - h).dot(M_inv) * time * time
                + x0
                + v0 * time
        )
        v0 = v0 + (f[i * 3 : i * 3 + 3] - h).dot(M_inv) * time
        v.append(v0)
    plt.plot(x2, label="x2")
    plt.plot(x3, label="x3")
    plt.axhline(
        y=dcm_reactive_stepper.get_next_support_foot_position()[0],
        linestyle="-",
    )
    plt.axhline(
        y=dcm_reactive_stepper.get_next_support_foot_position()[1],
        linestyle="-",
    )
    plt.axhline(
        y=dcm_reactive_stepper.get_next_support_foot_position()[2],
        linestyle="-",
    )
    # plt.plot(v, label="v")
    plt.legend()
    plt.grid()
    plt.show()

def dist(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

def closed_loop():
    global open_loop
    open_loop = False
    if dcm_reactive_stepper.get_is_left_leg_in_contact():
        dcm_reactive_stepper.set_right_foot_position(right_foot_location)
        dcm_reactive_stepper.set_right_foot_velocity(right_foot_vel)
    else:
        dcm_reactive_stepper.set_left_foot_position(left_foot_location)
        dcm_reactive_stepper.set_left_foot_velocity(left_foot_vel)

def detect_contact():
    for contact in p.getContactPoints():
        if dist(left_foot_location, contact[5]) < 0.02 and dist(
                right_foot_location, contact[5]
        ) > dist(left_foot_location, contact[5]):
            contact_array[0] = 1
        if dist(right_foot_location, contact[5]) < 0.02 and dist(
                left_foot_location, contact[5]
        ) > dist(right_foot_location, contact[5]):
            contact_array[1] = 1

def create_box(
        halfExtents, collisionFramePosition, collisionFrameOrientation=[0, 0, 0, 1]
):
    cuid = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=halfExtents,
        collisionFramePosition=collisionFramePosition,
        collisionFrameOrientation=collisionFrameOrientation,
    )
    mass = 0  # static box
    p.createMultiBody(mass, cuid)
    p.changeDynamics(
        cuid,
        -1,
        linearDamping=0.04,
        angularDamping=0.04,
        restitution=0.0,
        lateralFriction=2.0,
    )


def plot_all_contact_points():
    plt_next_support_foot_position = []
    for j in range(100):
        if j / 100.0 + t_min >= t_max:
            break
        dcm_reactive_stepper.dcm_vrp_planner_initialization(
            l_min,
            l_max,
            w_min,
            w_max,
            t_min + j / 100.0,
            t_max,
            l_p,
            com_height,
            weight,
            )
        contact_array = [0, 0]
        print(left_foot_vel)
        dcm_reactive_stepper.run(
            time,
            [left_foot_location[0], left_foot_location[1], 0.],
            [right_foot_location[0], right_foot_location[1], 0.],
            left_foot_vel,
            right_foot_vel,
            x_com,
            xd_com,
            yaw(q),
            contact_array,
            not open_loop,
        )
        plt_next_support_foot_position.append(
            dcm_reactive_stepper.get_next_support_foot_position().copy()
        )
    plt.figure("dcm")
    plt.plot(np.array(plt_next_support_foot_position)[:, 0], label="x")
    plt.plot(np.array(plt_next_support_foot_position)[:, 1], label="y")
    plt.legend()
    plt.show()
    dcm_reactive_stepper.dcm_vrp_planner_initialization(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight
    )


def external_force(com):
    force = np.array(
        [
            (random() - 0.5) * 7000,
            (random() - 0.5) * 7000,
            (random() - 0.5) * 2500,
            ]
    )
    p.applyExternalForce(
        objectUniqueId=robot.robotId,
        linkIndex=-1,
        forceObj=force,
        posObj=[com[0], com[1], com[2]],
        flags=p.WORLD_FRAME,
    )

if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround()
    robot = env.add_robot(BoltRobot())
    tau = np.zeros(6)
    p.resetDebugVisualizerCamera(1.6, 50, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(0.0001)
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

    # p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "Running.mp4")
    q = np.matrix(BoltConfig.initial_configuration).T
    q[2] -= 0.005
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    warmup = 10
    kp = np.array([150.0, 150.0, 150.0, 150.0, 150.0, 150.0])
    kd = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0.0, 0.0, 0]
    robot_config = BoltConfig()
    config_file = robot_config.paths["imp_ctrl_yaml"]
    bolt_leg_ctrl = RobotImpedanceController(robot, config_file)
    centr_controller = RobotCentroidalController(
        robot_config,
        mu=1,
        kc=[0, 0, 200],
        dc=[0, 0, 20],
        kb=[30, 5, 0],
        db=[40, 20, 0],
        qp_penalty_lin=[1, 1, 1e6],
        qp_penalty_ang=[1e6, 1e6, 1],
    )

    omega = 10.18
    l_min = -0.1 - 0.5
    l_max = 0.1 + 0.5
    w_min = -0.1 - 0.5
    w_max = 0.1 + 0.5
    t_min = 0.1
    t_max = 0.15 + 0.5
    l_p = 0.1235
    is_left_leg_in_contact = True
    com_height = 0.3
    weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
    mid_air_foot_height = 0.05
    control_period = 0.001
    planner_loop = 0.001
    x_des_local = [
        q[0].item(),
        q[1].item() + 0.02,
        0.0,
        q[0].item(),
        q[1].item() - 0.02,
        0.0,
        ]
    past_x = [
        q[0].item(),
        q[1].item() + 0.02,
        0.0,
        q[0].item(),
        q[1].item() - 0.02,
        0.0,
        ]
    v_des = [0.0, 0.0, 0.0]
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
       v_des,
    )

    dcm_reactive_stepper.set_desired_com_velocity(v_des)

    x_com = np.zeros((3, 1))
    x_com[:] = [[0.0], [0.0], [com_height]]
    xd_com = np.zeros((3, 1))
    x_com_cent = x_com.copy()
    xd_com_cent = xd_com.copy()
    # omega = 7.2#np.sqrt(9.8 / com_height) #Lhum Running
    cnt_array = [1, 1]
    time = 0
    control_time = 0
    open_loop = True
    duration_before_step_landing = 0.0

    # plot
    plt_time = []
    plt_control_time = []
    plt_time_r = []
    plt_time_l = []
    plt_time_all = []
    plt_x_com = []
    plt_xd_com = []
    plt_right_foot_position = []
    plt_right_foot_velocity = []
    plt_right_foot_acceleration = []
    plt_left_foot_position = []
    plt_left_foot_velocity = []
    plt_left_foot_acceleration = []
    plt_time_from_last_step_touchdown = []
    # plt_duration_before_step_landing = []
    plt_current_support_foot = []
    plt_step_time = []
    plt_dcm_local = []
    plt_left_eef_real_pos = []
    plt_right_eef_real_pos = []
    plt_r = []
    plt_is_left_in_contact = []
    plt_pos_des_local = []
    plt_q_com = []
    plt_qdot_com = []
    plt_F = []
    plt_x = []
    plt_q = []
    plt_qdot = []
    plt_qdot2 = []
    plt_tau = []
    plt_euler_angles = []
    plt_dcm = []
    plt_next_step_location = []
    plt_foot_mass_r = []
    plt_foot_mass_l = []
    plt_eq_11_r = []
    plt_eq_11_l = []
    plt_eq_h = []
    plt_eq_g = []
    plt_eq_qdot = []
    plt_eq_qddot = []
    plt_F_M_new = []
    plt_eq_fifteen = []
    plt_F_M = []
    plt_d_com = []
    plt_d_v_com = []
    dcm_force = [0.0, 0.0, 0.0]
    offset = 0.025
    dcm_reactive_stepper.start()
    plt_q_lokesh = []
    plt_dq_lokesh = []

    for i in range(4000):
        last_qdot = qdot
        q, qdot = robot.get_state()
        plt_q_lokesh.append(q)
        plt_dq_lokesh.append(qdot)
        robot.pin_robot.com(q, qdot)
        x_com = robot.pin_robot.com(q, qdot)[0]
        xd_com = robot.pin_robot.com(q, qdot)[1]
        # robot.forward_robot(q, qdot)
        # if i > 600 and i < 670:
        #     print("External Force")
        #     force = np.array([-68, 0, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        # if i > 1000 and i < 1070:
        #     print("External Force")
        #     force = np.array([0, -68, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        # if i > 1600 and i < 1670:
        #     print("External Force")
        #     force = np.array([0, 0, 160])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        if warmup <= i:
            ###### mass matrix
            m_q = np.matrix(q).transpose()
            m_qdot = np.matrix(qdot).transpose()

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

            # closed_loop()
            contact_array = [0, 0]
            # detect_contact()

            is_left_leg_in_contact = dcm_reactive_stepper.get_is_left_leg_in_contact()

            print(dcm_reactive_stepper.get_step_duration(), "    -      ", dcm_reactive_stepper.get_time_from_last_step_touchdown())
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
            duration_before_step_landing = dcm_reactive_stepper.get_step_duration() - dcm_reactive_stepper.get_time_from_last_step_touchdown()
            is_left_leg_in_contact = dcm_reactive_stepper.get_is_left_leg_in_contact()
            swing_foot = dcm_reactive_stepper.get_next_support_foot_position()

            # dcm_force = (
            #     dcm_reactive_stepper.get_forces().copy()
            # ) # feed forward
            # if (i + 5) % 1 == 0 and i > 85:# and int(dcm_reactive_stepper.get_time_from_last_step_touchdown() * 1000) == 0:
            #    d = dcm_reactive_stepper.get_forces().copy()
            #    plot(d)
            # if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
            #     desired_q = np.array(q.copy())[:, 0]
            # else:
            #     dcm_reactive_stepper.run(time, dcm_reactive_stepper.flying_foot_position, x_com.copy(), xd_com.copy(), 0)  # q[5])
            x_des_local = []
            x_des_local.extend(
                dcm_reactive_stepper.get_left_foot_position().copy()
            )
            x_des_local.extend(
                dcm_reactive_stepper.get_right_foot_position().copy()
            )
            t_s = 0.1
            cnt_array = dcm_reactive_stepper.get_contact_phase()
            print(cnt_array)
            print("XYZZZ", cnt_array, dcm_reactive_stepper.get_time_from_last_step_touchdown() , dcm_reactive_stepper.get_step_duration())
            print("XYZ",(dcm_reactive_stepper.get_step_duration() - dcm_reactive_stepper.get_time_from_last_step_touchdown()), (dcm_reactive_stepper.get_step_duration() - t_s))
            current_kessay = x_com[2] + (xd_com[2] / 10.18)
            if cnt_array[0] == cnt_array[1]: #== False
                print("KESSAY", (current_kessay - 0.2) / (take_off_kessay - 0.2))
                if dcm_reactive_stepper.get_is_left_leg_in_contact():
                    x_des_local[:3] = x_com
                    x_des_local[2] = (x_com[2] - 0.2) * (-current_kessay + take_off_kessay) / (take_off_kessay - 0.2)
                    x_des_local[3:] = [dcm_reactive_stepper.get_next_support_foot_position()[0] + (x_com[0] - dcm_reactive_stepper.get_next_support_foot_position()[0])
                                       * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                                       dcm_reactive_stepper.get_next_support_foot_position()[1] + (x_com[1] - dcm_reactive_stepper.get_next_support_foot_position()[1])
                                       * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                                       (0.1) * (current_kessay - 0.2) / (take_off_kessay - 0.2)]#dcm_reactive_stepper.get_next_support_foot_position()[2] +  x_com[2] - 0.3]

                    kp = np.array([10.0, 10.0, 75.0, 250.0, 150.0, 150.0])
                    kd = [.5, .5, 5., 5.0, 5.0, 5.0]
                else:
                    x_des_local[3:] = x_com
                    x_des_local[5] = (x_com[2] - 0.2) * (-current_kessay + take_off_kessay) / (take_off_kessay - 0.2)
                    x_des_local[:3] = [dcm_reactive_stepper.get_next_support_foot_position()[0] + (x_com[0] - dcm_reactive_stepper.get_next_support_foot_position()[0])
                                       * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                                       dcm_reactive_stepper.get_next_support_foot_position()[1] + (x_com[1] - dcm_reactive_stepper.get_next_support_foot_position()[1])
                                       * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                                       (0.1) * (current_kessay - 0.2) / (take_off_kessay - 0.2)]#dcm_reactive_stepper.get_next_support_foot_position()[2] +  x_com[2] - 0.3]

                    kp = np.array([250.0, 150.0, 150.0, 10.0, 10.0, 75.0])
                    kd = [5.0, 5.0, 5.0, .5, .5, 5.]
            elif cnt_array[0] == 1:
                x_des_local[:2] = x_com[:2]
                take_off_kessay = x_com[2] + (xd_com[2] / 10.18)
                x_des_local[3:] = [x_com[0],
                                   x_com[1],
                                   0.1]
                kp = np.array([0.0, 0.0, 0.0, 10.0, 10.0, 75.0])
                kd = [.0, .0, .0, 1., 1., 10.]

            else:
                x_des_local[3:5] = x_com[:2]
                take_off_kessay = x_com[2] + (xd_com[2] / 10.18)
                x_des_local[:3] = [x_com[0],
                                   x_com[1],
                                   0.1]
                kp = np.array([10.0, 10.0, 75.0, 0.0, 0.0, 0.0])
                kd = [1., 1., 10., .0, .0, .0]


            if open_loop:
                x_des_local[2] += offset
                x_des_local[5] += offset


            plt_time.append(time)
            plt_right_foot_position.append(x_des_local[3:6])
            plt_right_foot_velocity.append(dcm_reactive_stepper.get_right_foot_velocity().copy())
            plt_right_foot_acceleration.append(dcm_reactive_stepper.get_right_foot_acceleration().copy())
            plt_left_foot_position.append(x_des_local[:3])
            plt_left_foot_velocity.append(dcm_reactive_stepper.get_left_foot_velocity().copy())
            plt_left_foot_acceleration.append(dcm_reactive_stepper.get_left_foot_acceleration().copy())
            plt_time_from_last_step_touchdown.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
            # plt_duration_before_step_landing.append(dcm_reactive_stepper.duration_before_step_landing)
            plt_current_support_foot.append(dcm_reactive_stepper.get_current_support_foot_position().copy())
            # plt_dcm.append(dcm_reactive_stepper.dcm_vrp_planner.get_dcm_local().copy())
            plt_is_left_in_contact.append(dcm_reactive_stepper.get_is_left_leg_in_contact())
            plt_next_step_location.append(dcm_reactive_stepper.get_next_support_foot_position().copy())
            plt_dcm_local.append(x_com + (xd_com / 10.18))

            if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
                plt_step_time.append(int(i))
            time += 0.001

        for j in range(2):
            imp = bolt_leg_ctrl.imp_ctrl_array[j]
            x_des_local[3 * j : 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation
            if j == 0:
                plt_left_eef_real_pos.append(
                    np.array(
                        imp.pin_robot.data.oMf[imp.frame_end_idx].translation
                    ).reshape(-1)
                )
            else:
                plt_right_eef_real_pos.append(
                    np.array(
                        imp.pin_robot.data.oMf[imp.frame_end_idx].translation
                    ).reshape(-1)
                )
        if warmup <= i:
            com = dcm_reactive_stepper.get_com()#[0.0, 0.0, com_height + h_bais], [0.0, 0.0, 0.0]
            v_com = dcm_reactive_stepper.get_v_com()#[0.0, 0.0, com_height + h_bais], [0.0, 0.0, 0.0]
            a_com = dcm_reactive_stepper.get_a_com()
        else:
            com = [0.0, 0.0, com_height]
            v_com = [0.0, 0.0, 0.0]
            a_com = [0.0, 0.0, 0.0]
        plt_d_com.append(com.copy())
        plt_d_v_com.append(v_com.copy())
        w_com = centr_controller.compute_com_wrench(q.copy(), qdot.copy(), com, v_com,
                                                    [0, 0.0, 0, 1.0], [0.0, 0.0, 0.0],)
        w_com[0] = 0.0
        w_com[1] = 0.0
        w_com[2] += a_com[2] * total_mass#Lhum TODO add it to all the directions

        F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

        if cnt_array[0] == 0 and cnt_array[1] == 0:
            des_vel = [0., 0., 0., 0., 0., 0.]
        else:
            des_vel = np.concatenate((dcm_reactive_stepper.get_left_foot_velocity() -[qdot[0].item(), qdot[1].item(), qdot[2].item()],
                                      dcm_reactive_stepper.get_right_foot_velocity() - [qdot[0].item(), qdot[1].item(), qdot[2].item()]))
        try:
            dcm_force[0] = -dcm_force[0]
            dcm_force[1] = -dcm_force[1]
            dcm_force[2] = -dcm_force[2]
            if cnt_array[0] == cnt_array[1]:
                if is_left_leg_in_contact:
                    F[3:] = dcm_force[:3]
                else:
                    F[:3] = dcm_force[:3]
        except:
            F[:] = [0., 0., 0., 0., 0., 0.,]
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
        if warmup <= i:
            plt_control_time.append(control_time)
        plt_F.append(F)
        plt_x.append(x_des_local)
        plt_tau.append(tau)
        plt_xd_com.append(xd_com.copy())
        plt_qdot_com.append(qdot[3:6])
        plt_x_com.append(x_com.copy())
        plt_pos_des_local.append([x_des_local[1], x_des_local[4]])
        # plt_euler_angles.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
        plt_q.append(q[:].copy())
        # plt_qdot.append(inv_kin.xddot(q, qdot))
        # plt_qdot2.append(MM.dot(inv_kin.xddot(q, qdot)))
        plt_q_com.append(np.array(R.from_quat([q[3:7]]).as_euler('xyz', degrees=False))[0, :])
        # plt_desired_q.append(desired_q[7:].copy())

        for j in range(10):
            robot.send_joint_command(tau)
            p.stepSimulation()

    dcm_reactive_stepper.stop()
    #
    # FIGSIZE = 3.7
    # FONT_SIZE = 8
    # FONT_WEIGHT = "normal"
    # # set the parameters
    # font = {'family' : 'normal',
    #         'weight' : FONT_WEIGHT,
    #         'size'   : FONT_SIZE}
    # plt.rc('font', **font)
    # FIGURE_SIZE = ( FIGSIZE , FIGSIZE * 9.0/16.0)

    # p.stopStateLogging()

    # plt.figure("com")
    # plt.plot(plt_time, np.array(plt_x_com)[:,0])
    # plt.plot(plt_time, np.array(plt_x_com)[:,1])
    # plt.plot(plt_time, np.array(plt_x_com)[:,2])

    # plt.figure("xd")
    # plt.plot(plt_time, np.array(plt_xd_com)[:,0])
    # plt.plot(plt_time, np.array(plt_xd_com)[:,1])
    # plt.plot(plt_time, np.array(plt_xd_com)[:,2])

    # plt.figure("right_foot_pos")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,0])
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,1])
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,2])

    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,0])
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,1])
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,2])

    # plt.figure("feet_pos_y")
    # plt.plot(plt_time, np.array(plt_right_foot_position)[:,2])
    # plt.plot(plt_time, np.array(plt_left_foot_position)[:,2])

    plt.figure("y")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 1], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 1], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
    plt.plot(plt_time, np.array(plt_next_step_location)[:, 1], label="next_step_location")
    # plt.plot(plt_time, np.array(plt_dcm)[:, 1], label="dcm")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 1], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 1], label="right_eef_real_pos")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 1], label="current_support_foot")
    # plt.plot(plt_time, plt_pos_des_local[warmup + 1:], label = "pos des_local_eef")
    plt.legend()
    plt.grid()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    # plt.figure("q")
    # plt.plot(np.array(plt_qdot)[:, :, 0], label="plt_qddot")
    # plt.plot(np.array(plt_qdot2)[:, :, 0], label="plt_M*qddot")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("x")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 0], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 0], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    plt.plot(plt_time, np.array(plt_next_step_location)[:, 0], label="next_step_location")
    # plt.plot(plt_time, np.array(plt_dcm)[:, 0], label="dcm")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 0], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 0], label="right_eef_real_pos")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="des_left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="des_right")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 0], label="current_support_foot")
    # plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:], label="plt_duration_before_step_landing")
    # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
    plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    # plt.figure("tau")
    # plt.plot(np.array(plt_tau)[:, :], label="tau")
    # plt.legend()

    plt.figure("z")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 2], label="com")
    plt.plot(plt_time, plt_is_left_in_contact[:], label="is_left_in_contact")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 2], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 2], label="right")
    # plt.plot(plt_control_time, np.array(plt_x_com)[warmup:, 2], label="com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 2], label="dcm_local")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 2], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 2], label="right_eef_real_pos")
    plt.legend()


    plt.figure("z2")
    plt.plot(plt_control_time, np.array(plt_x_com)[warmup:, 2], label="com")
    plt.plot(plt_control_time, np.array(plt_xd_com)[warmup:, 2], label="xd_com")
    plt.plot(plt_control_time, np.array(plt_d_com)[warmup:, 2], label="d_com")
    plt.plot(plt_control_time, np.array(plt_d_v_com)[warmup:, 2], label="d_v_com")
    plt.legend()
    for time in plt_step_time:
        plt.axvline(1.0 * time / 1000)

    # plt.figure("rpy")
    # plt.plot(plt_control_time, np.array(plt_rpy)[:], label="rpy")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(1.0 * time / 1000)

    plt.figure("q")
    plt.plot(plt_time, np.array(plt_q_com)[warmup:, 0], label="x")
    plt.plot(plt_time, np.array(plt_q_com)[warmup:, 1], label="y")
    plt.plot(plt_time, np.array(plt_q_com)[warmup:, 2], label="z")
    plt.legend()
    for time in plt_step_time:
        plt.axvline(time / 1000)

    plt.figure("q_d")
    plt.plot(plt_time, np.array(plt_qdot_com)[warmup:, 0], label="x")
    plt.plot(plt_time, np.array(plt_qdot_com)[warmup:, 1], label="y")
    plt.plot(plt_time, np.array(plt_qdot_com)[warmup:, 2], label="z")
    plt.legend()
    for time in plt_step_time:
        plt.axvline(time / 1000)

    # plt.figure("F")
    # plt.plot(plt_time, plt_F[warmup:], label="F")
    # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
    # plt.legend()
    # np.savetxt('q' +'.txt', np.array(plt_q_lokesh)[:, :])
    # np.savetxt('q_dot' +'.txt', np.array(plt_dq_lokesh)[:, :])


    # add n e w _ = False/True
    # plt.figure("Z")
    # plt.plot(plt_time[:], np.array(plt_left_eef_real_pos)[warmup:, 2], label="left_z")
    # plt.plot(plt_time[:], np.array(plt_right_eef_real_pos)[warmup:, 2], label="right_z")
    # plt.plot(plt_time[:], np.array(plt_left_foot_position)[:, 2], label="des_left_z")
    # plt.plot(plt_time[:], np.array(plt_right_foot_position)[:, 2], label="des_right_z")
    # # plt.plot(plt_time[230:], np.array(plt_next_step_location)[230:, 2], label="next_step_location_z")
    # plt.legend()
    # np.savetxt('plt_left_eef_real_posz' + str(new_) +'.txt', np.array(plt_left_eef_real_pos)[warmup:, 2])
    # np.savetxt('plt_right_eef_real_posz' + str(new_) +'.txt', np.array(plt_right_eef_real_pos)[warmup:, 2])
    # np.savetxt('plt_left_foot_positionz' + str(new_) +'.txt', np.array(plt_left_foot_position)[:, 2])
    # np.savetxt('plt_right_foot_positionz' + str(new_) +'.txt', np.array(plt_right_foot_position)[:, 2])
    #
    # plt.figure("xy")
    # plt.plot(plt_time[:], np.array(plt_left_eef_real_pos)[warmup:, 0], label="left_x")
    # plt.plot(plt_time[:], np.array(plt_left_eef_real_pos)[warmup:, 1], label="left_y")
    # plt.plot(plt_time[:], np.array(plt_right_eef_real_pos)[warmup:, 0], label="right_x")
    # plt.plot(plt_time[:], np.array(plt_right_eef_real_pos)[warmup:, 1], label="right_y")
    # plt.plot(plt_time[:], np.array(plt_left_foot_position)[:, 0], label="des_left_x")
    # plt.plot(plt_time[:], np.array(plt_left_foot_position)[:, 1], label="des_lef_y")
    # plt.plot(plt_time[:], np.array(plt_right_foot_position)[:, 0], label="des_right_x")
    # plt.plot(plt_time[:], np.array(plt_right_foot_position)[:, 1], label="des_right_y")
    # plt.plot(plt_time[:], np.array(plt_next_step_location)[:, 0], label="next_step_location_x")
    # plt.plot(plt_time[:], np.array(plt_next_step_location)[:, 1], label="next_step_location_y")
    # # plt.plot(plt_time[230:], np.array(plt_next_step_location)[230:, 2], label="next_step_location_z")
    # plt.legend()
    # np.savetxt('plt_left_eef_real_posx' + str(new_) +'.txt', np.array(plt_left_eef_real_pos)[warmup:, 0])
    # np.savetxt('plt_left_eef_real_posy' + str(new_) +'.txt', np.array(plt_left_eef_real_pos)[warmup:, 1])
    # np.savetxt('plt_right_eef_real_posx' + str(new_) +'.txt', np.array(plt_right_eef_real_pos)[warmup:, 0])
    # np.savetxt('plt_right_eef_real_posy' + str(new_) +'.txt', np.array(plt_right_eef_real_pos)[warmup:, 1])
    # np.savetxt('plt_left_foot_positionx' + str(new_) +'.txt', np.array(plt_left_foot_position)[:, 0])
    # np.savetxt('plt_left_foot_positiony' + str(new_) +'.txt', np.array(plt_left_foot_position)[:, 1])
    # np.savetxt('plt_right_foot_positionx' + str(new_) +'.txt', np.array(plt_right_foot_position)[:, 0])
    # np.savetxt('plt_right_foot_positiony' + str(new_) +'.txt', np.array(plt_right_foot_position)[:, 1])
    # np.savetxt('plt_next_step_locationx' + str(new_) +'.txt', np.array(plt_next_step_location)[:, 0])
    # np.savetxt('plt_next_step_locationy' + str(new_) +'.txt', np.array(plt_next_step_location)[:, 1])
    # np.savetxt('plt_is_left_in_contact' + str(new_) +'.txt', np.array(plt_is_left_in_contact)[:])

    # plt.figure("last_step_touchdown")
    # plt.plot(plt_time, np.array(plt_time_from_last_step_touchdown)[:])
    # plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:])

    # plt.figure("support_foot")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:,0])
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:,1])
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:,2])

    # plt.figure("warmup2")
    # # plt.plot(np.array(plt_x)[:, :], label="des")
    # # plt.plot(np.array(plt_r)[warmup:, 2, 0], label = "real")
    # plt.plot(np.array(plt_F)[warmup:, :], label = "Force")
    # plt.legend()
    #
    # plt.figure("warm up")
    # plt.plot(np.array(plt_left_eef_real_pos)[1:, :], label="left_eef_real_pos")
    # plt.plot(np.array(plt_right_eef_real_pos)[1:, :], label="right_eef_real_pos")
    # plt.legend()
    #
    # plt.figure("warm up1")
    # plt.plot(np.array(plt_x_com)[:, :], label="com")
    # plt.plot(np.array(plt_xd_com)[:, :], label="xd_com")
    # plt.plot(np.array(plt_qdot_com)[:, :6, 0], label="qdot")
    # plt.legend()

    # fig, ax = plt.subplots(3, 2)
    # ax[0][0].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 0], 'o', markersize=1, label='$\max F_x$')
    # ax[0][0].legend()
    # ax[1][0].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 1], 'o', markersize=1, label='$\max F_y$')
    # ax[1][0].legend()
    # ax[2][0].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 2], 'o', markersize=1, label='$\max F_z$')
    # ax[2][0].legend()
    # ax[0][1].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 3], 'o', markersize=1, label='$\min F_x$')
    # ax[0][1].legend()
    # ax[1][1].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 4], 'o', markersize=1, label='$\min F_y$')
    # ax[1][1].legend()
    # ax[2][1].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 5], 'o', markersize=1, label='$\min F_z$')
    # ax[2][1].legend()
    # plt.savefig("min_max" + ".pdf")

    plt.show()
