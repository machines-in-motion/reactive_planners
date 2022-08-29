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
from reactive_planners_cpp import DcmReactiveStepper
import pinocchio as se3
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv, pinv
from math import sqrt
from random import random
from bullet_utils.env import BulletEnvWithGround
from perlin_noise import PerlinNoise


def zero_cnt_gain(kp, cnt_array):
    gain = np.array(kp).copy()
    for i, v in enumerate(cnt_array):
        if v == 1:
            gain[3 * i : 3 * (i + 1)] = 0.0
    return gain

def yaw(q):
    print(q)
    return np.array(
        R.from_quat([np.array(q)[3:7]]).as_euler("xyz", degrees=False)
    )[0, 2]

height_field_terrain_shape = None

def generate_terrain():

    global height_field_terrain_shape
    # remove the default plane
    p.removeBody(0)

    # terrain patch properties
    length_per_index = 0.05 # fixed as only then the contact forces are simulated properly
    patch_length_x = 1.5 # subject to the trajectory length
    patch_length_y = 1.5 # subject to the trajectory length
    numHeightfieldRows = int(patch_length_x / length_per_index)
    numHeightfieldColumns = int(patch_length_y / length_per_index)
    terrainMap = np.zeros((numHeightfieldRows,numHeightfieldColumns))


    # hilly terrain generated through perlin noise
    if True:
        noise = PerlinNoise(octaves=10)
        for i in range(numHeightfieldRows):
            for j in range(numHeightfieldColumns):
                h = 0.1 * noise([i/numHeightfieldRows, j/numHeightfieldColumns])
                terrainMap[i][j] = h


    heightfieldData = terrainMap.T.flatten()
    if height_field_terrain_shape == None:
        # first time, define the height field
        height_field_terrain_shape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD,
                                                                        meshScale=[ length_per_index , length_per_index ,1],
                                                                        heightfieldTextureScaling=(numHeightfieldRows-1)/2,
                                                                        heightfieldData=heightfieldData,
                                                                        numHeightfieldRows=numHeightfieldRows,
                                                                        numHeightfieldColumns=numHeightfieldColumns)
    else:
        # from second time, simply update the height field
        # to prevent memory leak
        height_field_terrain_shape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD,
                                                                        meshScale=[ length_per_index , length_per_index ,1],
                                                                        heightfieldTextureScaling=(numHeightfieldRows-1)/2,
                                                                        heightfieldData=heightfieldData,
                                                                        numHeightfieldRows=numHeightfieldRows,
                                                                        numHeightfieldColumns=numHeightfieldColumns,
                                                                        replaceHeightfieldIndex=height_field_terrain_shape)


    terrain_id  = p.createMultiBody(baseMass = 0, baseCollisionShapeIndex = height_field_terrain_shape)

    terrainTexture = p.loadTexture('wood.jpg')
    p.changeVisualShape(terrain_id, -1, rgbaColor=[.101, .67, .33, 1.0], textureUniqueId = terrainTexture)

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
    p.resetDebugVisualizerCamera(2., 50, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(0.001)
    p.setRealTimeSimulation(0)
    p.removeAllUserParameters()
    p.removeAllUserDebugItems()
    # generate_terrain()

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
    logID = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "Running_side.mp4")
    q = np.matrix(BoltConfig.initial_configuration).T
    # q[0] += 0.9
    q[2] += 0.01
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    warmup = 10
    kp = np.array([150.0, 150.0, 150.0, 150.0, 150.0, 150.0])
    kd = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0, 0.0, 0]
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
    l_min = -0.2
    l_max = 0.2
    w_min = -0.2
    w_max = 0.2
    t_min = 0.05
    t_max = 0.15 + 0.5
    l_p = 0.1235
    is_left_leg_in_contact = True
    com_height = 0.3
    weight = [1, 1, 5, 1000, 1000, 5, 10000000, 10000000, 10000000, 10000000]
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
    t_s = 0.1
    v_des = [1.8, 0.0, 0.0]
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
        t_s,
        weight,
        mid_air_foot_height,
        control_period,
        planner_loop,
        x_des_local[:3],
        x_des_local[3:],
       v_des)

    dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)
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
    duration_of_stance_phase = 0.0
    landing_kessay = 0.2

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
    plt_duration_of_stance_phase = []
    plt_current_support_foot = []
    plt_current_support_foot_stance_phase = []
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
    plt_vel_des = []
    plt_stance_phase = []
    plt_d_dcm = []
    plt_duration_of_flight_phase = []
    plt_force = []
    plt_qddot_com = []
    plt_qdot_com2 = []
    pv = [0.0, 0.0, 0.0]
    plt_T = []
    record_plot = True
    plt_tau_lokesh = []
    plt_time_reactive_planner = []
    plt_time_controller = []
    plt_time_simulation = []
    cur_time = []
    # file = open("tau.txt", "a")
    # getcontext().prec = 6


    for i in range(2820):
        last_qdot = qdot
        q, qdot = robot.get_state()
        if record_plot:
            plt_q_lokesh.append(q)
            plt_dq_lokesh.append(qdot)
        robot.pin_robot.com(q, qdot)
        # if i == 4100:
        x_com = robot.pin_robot.com(q, qdot)[0]
        xd_com = robot.pin_robot.com(q, qdot)[1]
        # # if i == 2000:
        # #     v_des = [2, 0.0, 0.0]
        # #     print(v_des)
        # #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # # if i == 4000:
        # #     v_des = [-2, 0.0, 0.0]
        # #     print(v_des)
        # #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # # if i == 6000:
        # #     v_des = [0.0, 0.5, 0.0]
        # #     print(v_des)
        # #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # # if i == 9000:
        # #     v_des = [0.0, -0.5, 0.0]
        # #     print(v_des)
        # #     dcm_reactive_stepper.set_desired_com_velocity(v_des)

        # if i < 3000:
        #     v_des = [0.0, 0.0, 0.0]
        #     dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)
        #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # elif i < 8000:
        #     v_des = [-0.8, 0.0, 0.0]
        #     dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)
        #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # elif i < 7000:
        #     v_des = [0.0, 0.0, 0.0]
        #     dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)
        #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # elif i < 8000:
        #     v_des = [0.0, 0.08, 0.0]
        #     dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)
        #     dcm_reactive_stepper.set_desired_com_velocity(v_des)
        # elif i < 11000:
        #     v_des = [0.0, 0.0, 0.0]
        #     dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)
        #     dcm_reactive_stepper.set_desired_com_velocity(v_des)

        # plt_vel_des.append(v_des)


        # if i >= 880 and i < 920:
        #     if i == 880:
        #         ball = env.add_object_from_urdf("ball.urdf", pos=[0.2, 0.2, 0.4], orn=[0, 0, 0, 1], useFixedBase=False)
        #         terrainTexture = p.loadTexture('balldimpled.png')
        #         p.changeVisualShape(ball, -1, textureUniqueId = terrainTexture)
        #     force = 800 * np.array(-np.array(p.getBasePositionAndOrientation(ball)[0]) + q[:3] + [0, 0, 0.1])
        #     p.applyExternalForce(objectUniqueId=ball, linkIndex=-1, forceObj=force,
        #                          posObj=p.getBasePositionAndOrientation(ball)[0], flags=p.WORLD_FRAME)
        #
        #     if record_plot:
        #         plt_force.append(p.getBaseVelocity(ball)[0])
        #
        # if i >= 2400 and i < 2450:
        #     if i == 2400:
        #         ball = env.add_object_from_urdf("ball.urdf", pos=[-0.6, -0.6, 0.35], orn=[0, 0, 0, 1], useFixedBase=False)
        #         terrainTexture = p.loadTexture('balldimpled.png')
        #         p.changeVisualShape(ball, -1, textureUniqueId = terrainTexture)
        #     force = 400 * np.array(-np.array(p.getBasePositionAndOrientation(ball)[0]) + q[:3] + [0, 0, 0.05])
        #     p.applyExternalForce(objectUniqueId=ball, linkIndex=-1, forceObj=force,
        #                          posObj=p.getBasePositionAndOrientation(ball)[0], flags=p.WORLD_FRAME)
        #
        #     if record_plot:
        #         plt_force.append(p.getBaseVelocity(ball)[0])
        #
        # if i == 2600 or i == 1000:
        #     p.removeBody(ball)

        # if i >= 3000 and i < 3100:
        #     if i == 3000:
        #         ball = env.add_object_from_urdf("ball.urdf", pos=[0.2, -0.2, 0.4], orn=[0, 0, 0, 1], useFixedBase=False)
        #     force = 200 * np.array(-np.array(p.getBasePositionAndOrientation(ball)[0]) + q[:3] + [0, 0, 0.1])
        #     p.applyExternalForce(objectUniqueId=ball, linkIndex=-1, forceObj=force,
        #                          posObj=p.getBasePositionAndOrientation(ball)[0], flags=p.WORLD_FRAME)
        # if i >= 4000 and i < 4100:
        #     if i == 4000:
        #         ball = env.add_object_from_urdf("ball.urdf", pos=[-0.4, -0.4, 0.4], orn=[0, 0, 0, 1], useFixedBase=False)
        #     force = 200 * np.array(-np.array(p.getBasePositionAndOrientation(ball)[0]) + q[:3] + [0, 0, 0.1])
        #     p.applyExternalForce(objectUniqueId=ball, linkIndex=-1, forceObj=force,
        #                          posObj=p.getBasePositionAndOrientation(ball)[0], flags=p.WORLD_FRAME)
        # if i >= 5000 and i < 5100:
        #     if i == 5000:
        #         ball = env.add_object_from_urdf("ball.urdf", pos=[-0.4, 0.4, 0.4], orn=[0, 0, 0, 1], useFixedBase=False)
        #     force = 200 * np.array(-np.array(p.getBasePositionAndOrientation(ball)[0]) + q[:3] + [0, 0, 0.1])
        #     p.applyExternalForce(objectUniqueId=ball, linkIndex=-1, forceObj=force,
        #                          posObj=p.getBasePositionAndOrientation(ball)[0], flags=p.WORLD_FRAME)
        # if i >= 6000 and i < 6100:
        #     if i == 6000:
        #         ball = env.add_object_from_urdf("ball.urdf", pos=[-0.4, -0.4, 0.4], orn=[0, 0, 0, 1], useFixedBase=False)
        #     force = 200 * np.array(-np.array(p.getBasePositionAndOrientation(ball)[0]) + q[:3] + [0, 0, 0.1])
        #     p.applyExternalForce(objectUniqueId=ball, linkIndex=-1, forceObj=force,
        #                          posObj=p.getBasePositionAndOrientation(ball)[0], flags=p.WORLD_FRAME)



        # robot.forward_robot(q, qdot)

        #
        # if i > 570 and i < 580:
        #     if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
        #         omega = 9
        #         dcm_reactive_stepper.set_new_motion(0.265, omega, t_s)
            # print("External Force")
            # force = np.array([0, -980, 0])
            # p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
            #                      posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        # if i > 1000 and i < 1070:
        #     if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
        #         omega = 8
        #         dcm_reactive_stepper.set_new_motion(0.2357, omega, t_s)
        #     print("External Force")
        #     force = np.array([0, -68, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)

        # if i > 6600 and i < 7100:
        #     if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0 and omega != 5.7183913822:
        #         omega = 5.7183913822
        #         dcm_reactive_stepper.set_new_motion(0.3, omega, 0.2)
                # dcm_reactive_stepper.initialize(
                #     is_left_leg_in_contact,
                #     l_min,
                #     l_max,
                #     w_min,
                #     w_max,
                #     t_min + 0.1,
                #     t_max + 0.15,
                #     l_p,
                #     com_height,
                #     weight,
                #     mid_air_foot_height,
                #     control_period,
                #     planner_loop,
                #     x_des_local[:3],
                #     x_des_local[3:],
                #     v_des,
                # )
                # dcm_reactive_stepper.set_new_motion(0.3, omega, 0.2)
        #
        # if i > 4600 and i < 5100:
        #     if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
        #         omega = 10.18
        #         dcm_reactive_stepper.set_new_motion(com_height, omega, t_s)

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
            duration_of_stance_phase = dcm_reactive_stepper.get_duration_of_stance_phase() - dcm_reactive_stepper.get_time_from_last_step_touchdown()
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
            current_kessay = x_com[2] + (xd_com[2] / omega)
            u_t = -dcm_reactive_stepper.get_dcm_offset() + x_com + (xd_com / omega)
            if cnt_array[0] == cnt_array[1]: #== False
                if dcm_reactive_stepper.get_is_left_leg_in_contact():
                    x_des_local[:3] = x_com
                    x_des_local[2] = (x_com[2] - 0.2) * (-current_kessay + take_off_kessay) / (take_off_kessay - 0.2)
                    # x_des_local[3:] = [u_t[0] + (x_com[0] - u_t[0]) * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                    #                    u_t[1] + (x_com[1] - u_t[1]) * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                    #                    (0.1) * (current_kessay - 0.2) / (take_off_kessay - 0.2)]#dcm_reactive_stepper.get_next_support_foot_position()[2] +  x_com[2] - 0.3]
                    x_des_local[3:] = [u_t[0],
                                       u_t[1],
                                       (0.1) * (current_kessay - 0.2) / (take_off_kessay - 0.2)]#dcm_reactive_stepper.get_next_support_foot_position()[2] +  x_com[2] - 0.3]

                    kp = np.array([10.0, 10.0, 75.0, 250.0, 150.0, 150.0])
                    kd = [.5, .5, 5., 5.0, 5.0, 5.0]
                else:
                    x_des_local[3:] = x_com
                    x_des_local[5] = (x_com[2] - 0.2) * (-current_kessay + take_off_kessay) / (take_off_kessay - 0.2)
                    # x_des_local[:3] = [u_t[0] + (x_com[0] - u_t[0]) * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                    #                    u_t[1] + (x_com[1] - u_t[1]) * (current_kessay - 0.2) / (take_off_kessay - 0.2),
                    #                    (0.1) * (current_kessay - 0.2) / (take_off_kessay - 0.2)]#dcm_reactive_stepper.get_next_support_foot_position()[2] +  x_com[2] - 0.3]
                    x_des_local[:3] = [u_t[0],
                                       u_t[1],
                                       (0.1) * (current_kessay - 0.2) / (take_off_kessay - 0.2)]#dcm_reactive_stepper.get_next_support_foot_position()[2] +  x_com[2] - 0.3]

                    kp = np.array([250.0, 150.0, 150.0, 10.0, 10.0, 75.0])
                    kd = [5.0, 5.0, 5.0, .5, .5, 5.]
            elif cnt_array[0] == 1:
                # x_des_local[:2] = x_com[:2]
                take_off_kessay = x_com[2] + (xd_com[2] / omega)
                # x_des_local[3:] = [x_com[0],
                #                    x_com[1],
                #                    0.1]
                x_des_local[3:] = [u_t[0],
                                   u_t[1],
                                   0.1]
                x_des_local[:3] = dcm_reactive_stepper.get_current_support_foot_position().copy()
                if omega == 5.7183913822:
                    x_des_local[5] = 0.1 / 0.2 * (0.2 - dcm_reactive_stepper.get_time_from_last_step_touchdown())

                if omega == 5.7183913822:
                    kp = 2 * np.array([0.0, 0.0, 0.0, 30.0, 30.0, 75.0])
                    kd = 2 * [.0, .0, .0, 4., 4., 10.]
                else:
                    kp = np.array([0.0, 0.0, 0.0, 10.0, 10.0, 75.0])
                    kd = [.0, .0, .0, 1., 1., 10.]

            else:
                # x_des_local[3:5] = x_com[:2]
                take_off_kessay = x_com[2] + (xd_com[2] / omega)
                # x_des_local[:3] = [x_com[0],
                #                    x_com[1],
                #                    0.1]
                x_des_local[:3] = [u_t[0],
                                   u_t[1],
                                   0.1]

                x_des_local[3:] = dcm_reactive_stepper.get_current_support_foot_position().copy()
                if omega == 5.7183913822:
                    x_des_local[2] = 0.1 / 0.2 * (0.2 - dcm_reactive_stepper.get_time_from_last_step_touchdown())

                if omega == 5.7183913822:
                    kp = 2 * np.array([30.0, 30.0, 75.0, 0.0, 0.0, 0.0])
                    kd = 2 * [4., 4., 10., .0, .0, .0]
                else:
                    kp = np.array([10.0, 10.0, 75.0, 0.0, 0.0, 0.0])
                    kd = [1., 1., 10., .0, .0, .0]


            if open_loop:
                x_des_local[2] += offset
                x_des_local[5] += offset

            if record_plot:
                if cnt_array[0] != cnt_array[1]:
                    plt_stance_phase.append(time)
                    plt_current_support_foot_stance_phase.append(dcm_reactive_stepper.get_current_support_foot_position().copy())
                else:
                    x=2
                plt_time.append(time)
                plt_right_foot_position.append(x_des_local[3:6])
                plt_right_foot_velocity.append(dcm_reactive_stepper.get_right_foot_velocity().copy())
                plt_right_foot_acceleration.append(dcm_reactive_stepper.get_right_foot_acceleration().copy())
                plt_left_foot_position.append(x_des_local[:3])
                plt_left_foot_velocity.append(dcm_reactive_stepper.get_left_foot_velocity().copy())
                plt_left_foot_acceleration.append(dcm_reactive_stepper.get_left_foot_acceleration().copy())
                plt_time_from_last_step_touchdown.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
                plt_duration_of_stance_phase.append(dcm_reactive_stepper.get_duration_of_stance_phase())
                plt_duration_of_flight_phase.append(dcm_reactive_stepper.get_duration_of_flight_phase())
                plt_current_support_foot.append(dcm_reactive_stepper.get_current_support_foot_position().copy())
                # plt_dcm.append(dcm_reactive_stepper.dcm_vrp_planner.get_dcm_local().copy())
                plt_is_left_in_contact.append(dcm_reactive_stepper.get_is_left_leg_in_contact())
                plt_next_step_location.append(dcm_reactive_stepper.get_next_support_foot_position().copy())
                plt_dcm_local.append(x_com + (xd_com / omega))

                if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
                    plt_step_time.append(int(i))
            time += 0.001

        for j in range(2):
            imp = bolt_leg_ctrl.imp_ctrl_array[j]
            x_des_local[3 * j : 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation

            if record_plot:
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
        if record_plot:
            plt_d_com.append(com.copy())
            plt_d_v_com.append(v_com.copy())
            plt_d_dcm.append([com[0] + v_com[0] / omega, com[1] + v_com[1] / omega, com[2] + v_com[2] / omega])
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

        if record_plot:
            if warmup <= i:
                plt_control_time.append(control_time)
            plt_F.append(F)
            plt_x.append(x_des_local)
            plt_tau.append(tau)
            plt_xd_com.append(xd_com.copy())
            if i > 0:
                # print(qdot[3:6], (np.array(R.from_quat([q[3:7]]).as_euler('xyz', degrees=False))[0, :] - plt_q_com[-1]) * 1000)
                plt_qdot_com2.append((np.array(R.from_quat([q[3:7]]).as_euler('xyz', degrees=False))[0, :] - plt_q_com[-1]) * 1000)
                plt_qddot_com.append((qdot[3:6] - plt_qdot_com[-1]) * 1000)
            plt_qdot_com.append(qdot[3:6])
            plt_x_com.append(x_com.copy())
            plt_pos_des_local.append([x_des_local[1], x_des_local[4]])
            # plt_euler_angles.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
            plt_q.append(q[:].copy())
            # plt_qdot.append(inv_kin.xddot(q, qdot))
            # plt_qdot2.append(MM.dot(inv_kin.xddot(q, qdot)))
            plt_q_com.append(np.array(R.from_quat([q[3:7]]).as_euler('xyz', degrees=False))[0, :])
            # plt_desired_q.append(desired_q[7:].copy())

        # for j in range(10):
        # np.savetxt(file, [np.around(tau, decimals=6)], fmt='%.6f')
        # file.flush()
        # lines = np.loadtxt("tau.txt")
        # print(len(lines))
        # last_line = lines[-1,:]
        # print(last_line)
        # robot.send_joint_command(last_line)
        # p.stepSimulation()
        # /
        robot.send_joint_command(tau)
        p.stepSimulation()

    #     if i % (1000 / 25) == 0:
    #         img = p.getCameraImage(1024, 768, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    #         print(str("image2/" + str(i) + ".jpg"))
    #         Image.fromarray(np.array(img[2])[:, :, :3], 'RGB').save(str("image2/" + str(i) + ".jpg"))
    #         # img_array.append(img)
    #         plt_T.append(round(T.time() * 1000))
    #
    # size = (1024, 768)
    # out = cv2.VideoWriter('Running.mp4',cv2.VideoWriter_fourcc(*'MP4V'), 25, size)
    #
    # for i in range(len(os.listdir("image"))):
    #     print(i)
    #     img = cv2.imread("image2/" + str(40 * i) + ".jpg")
    #     # print(img)
    #     out.write(img)
    # out.release()

    # dcm_reactive_stepper.stop()
    #
    FIGSIZE = 3.7
    FONT_SIZE = 8
    FONT_WEIGHT = "normal"
    # set the parameters
    font = {'family' : 'normal',
            'weight' : FONT_WEIGHT,
            'size'   : FONT_SIZE}
    plt.rc('font', **font)
    FIGURE_SIZE = ( FIGSIZE , FIGSIZE * 9.0/16.0)

    p.stopStateLogging(logID)
    print("TIME ", plt_T)

    # np.savetxt('tau' +'.txt', np.array(plt_tau_lokesh)[:, :])

    if record_plot:
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

        # plt.figure("y_com")
        # plt.plot(plt_time, np.array(plt_x_com)[warmup:, 1], label="com")
        # plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 1], label="xd_com")
        # plt.plot(plt_time, np.array(plt_vel_des)[warmup:, 1], label="des_xd_com")

        plt.figure("y")
        plt.plot(plt_time, np.array(plt_x_com)[warmup:, 1], label="com")
        plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 1], label="xd_com")
        plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
        # plt.plot(plt_time, np.array(plt_dcm)[:, 1], label="dcm")
        plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 1], label="left_eef_real_pos")
        plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 1], label="right_eef_real_pos")
        plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
        plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
        plt.plot(plt_time, np.array(plt_next_step_location)[:, 1], '-r', label="next_step_location")
        plt.plot(plt_time, np.array(plt_current_support_foot)[:, 1], label="current_support_foot")
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

        # plt.figure("x_com")
        # plt.plot(plt_time, np.array(plt_x_com)[warmup:, 0], label="com")
        # plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 0], label="xd_com")
        # plt.plot(plt_time, np.array(plt_vel_des)[warmup:, 0], label="des_xd_com")

        plt.figure("x")
        plt.plot(plt_time, np.array(plt_x_com)[warmup:, 0], label="com")
        plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 0], label="xd_com")
        plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
        # plt.plot(plt_time, np.array(plt_dcm)[:, 0], label="dcm")
        plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 0], label="left_eef_real_pos")
        plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 0], label="right_eef_real_pos")
        plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="des_left")
        plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="des_right")
        plt.plot(plt_time, np.array(plt_next_step_location)[:, 0], '-r', label="next_step_location")
        plt.plot(plt_time, np.array(plt_current_support_foot)[:, 0], label="current_support_foot")
        # plt.plot(plt_time, np.array(plt_duration_of_stance_phase)[:], label="plt_duration_of_stance_phase")
        # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
        plt.legend()
        # for time in plt_step_time:
        #     plt.axvline(time / 1000)

        plt.figure("tau")
        plt.plot(np.array(plt_tau)[:, :], label="tau")
        plt.legend()

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
        #
        #
        # plt.figure("z2")
        # plt.plot(plt_control_time, np.array(plt_x_com)[warmup:, 2], label="com")
        # plt.plot(plt_control_time, np.array(plt_xd_com)[warmup:, 2], label="xd_com")
        # plt.plot(plt_control_time, np.array(plt_d_com)[warmup:, 2], label="d_com")
        # plt.plot(plt_control_time, np.array(plt_d_v_com)[warmup:, 2], label="d_v_com")
        # plt.legend()
        # for time in plt_step_time:
        #     plt.axvline(1.0 * time / 1000)

        # plt.figure("rpy")
        # plt.plot(plt_control_time, np.array(plt_rpy)[:], label="rpy")
        # plt.legend()
        # for time in plt_step_time:
        #     plt.axvline(1.0 * time / 1000)

        # plt.figure("q")
        # plt.plot(plt_time, np.array(plt_q_com)[warmup:, 0], label="x")
        # plt.plot(plt_time, np.array(plt_q_com)[warmup:, 1], label="y")
        # plt.plot(plt_time, np.array(plt_q_com)[warmup:, 2], label="z")
        # plt.legend()
        # for time in plt_step_time:
        #     plt.axvline(time / 1000)
        #
        # plt.figure("q_d")
        # plt.plot(plt_time, np.array(plt_qdot_com)[warmup:, 0], label="x")
        # plt.plot(plt_time, np.array(plt_qdot_com)[warmup:, 1], label="y")
        # plt.plot(plt_time, np.array(plt_qdot_com)[warmup:, 2], label="z")
        # plt.plot(plt_time, np.array(plt_qdot_com2)[warmup - 1:, 0], label="x")
        # plt.plot(plt_time, np.array(plt_qdot_com2)[warmup - 1:, 1], label="y")
        # plt.plot(plt_time, np.array(plt_qdot_com2)[warmup - 1:, 2], label="z")
        # plt.legend()
        # for time in plt_step_time:
        #     plt.axvline(time / 1000)
        #
        # plt.figure("q_dd")
        # plt.plot(plt_time, np.array(plt_qddot_com)[warmup - 1:, 0], label="x")
        # plt.plot(plt_time, np.array(plt_qddot_com)[warmup - 1:, 1], label="y")
        # plt.plot(plt_time, np.array(plt_qddot_com)[warmup - 1:, 2], label="z")
        # plt.legend()
        # for time in plt_step_time:
        #     plt.axvline(time / 1000)

        # plt.figure("F")
        # plt.plot(plt_time, plt_F[warmup:], label="F")
        # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
        # plt.legend()
        # np.savetxt('q' +'.txt', np.array(plt_q_lokesh)[:, :])
        # np.savetxt('q_dot' +'.txt', np.array(plt_dq_lokesh)[:, :])
        # np.savetxt('tau' +'.txt', np.array(plt_tau_lokesh)[:, :])


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

        # plt.figure("force")
        # plt.plot(plt_force, label="vel")
        # plt.legend()
        #
        # plt.figure("last_step_touchdown")
        # plt.plot(plt_time, np.array(plt_time_from_last_step_touchdown)[:])
        # plt.plot(plt_time, np.array(plt_duration_of_stance_phase)[:])

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

        # plot 2 article
        fig, ax = plt.subplots(1, 1)
        ax.plot(plt_time, np.array(plt_duration_of_flight_phase[:]) + np.array(plt_duration_of_stance_phase[:]), 'r')
        for time in plt_step_time:
            ax.axvline((time - warmup) / 1000)

        ax.set_ylabel("T [s]")
        ax.set_xlabel("Time [s]")

        # plt.savefig("Time4" + ".pdf")
        print("stance phase", len(plt_stance_phase))
        print("time", len(plt_time))

        # plot 1 article
        # fig, ax = plt.subplots(3)
        # ax[0].plot(plt_time, np.array(plt_x_com)[warmup:,0], label="COM")
        # ax[0].plot(plt_time, np.array(plt_xd_com)[warmup:,0], label="V_CoM")
        # ax[0].plot(plt_time, np.array(plt_dcm_local)[:, 0], label="DCM")
        # # ax[0].plot(plt_time, np.array(plt_next_step_location)[:, 0], label="next_step_location")
        # ax[0].plot(plt_stance_phase, np.array(plt_current_support_foot_stance_phase)[:, 0], 'o', markersize=1, label="CoP")
        #
        # ax[1].plot(plt_time, np.array(plt_x_com)[warmup:,1], label="CoM")
        # ax[1].plot(plt_time, np.array(plt_xd_com)[warmup:,1], label="V_CoM")
        # ax[1].plot(plt_time, np.array(plt_dcm_local)[:, 1], label="DCM")
        # ax[1].plot(plt_stance_phase, np.array(plt_current_support_foot_stance_phase)[:, 1], 'o', markersize=1, label="CoP")
        #
        # ax[2].plot(plt_time, np.array(plt_x_com)[warmup:,2], label="CoM")
        # ax[2].plot(plt_time, np.array(plt_xd_com)[warmup:,2], label="V_CoM")
        # ax[2].plot(plt_time, np.array(plt_dcm_local)[:, 2],  label="DCM")
        # ax[2].plot(plt_stance_phase, np.array(plt_current_support_foot_stance_phase)[:, 2], 'o', markersize=1, label="CoP")
        # ax[0].legend()
        # ax[0].grid()
        # ax[1].grid()
        # ax[2].grid()
        # for time in plt_step_time:
        #     ax[0].axvline((time - warmup) / 1000)
        #     ax[1].axvline((time - warmup) / 1000)
        #     ax[2].axvline((time - warmup) / 1000)
        #
        # ax[0].set_ylabel("X [m]")
        # ax[1].set_ylabel("Y [m]")
        # ax[2].set_ylabel("Z [m]")
        # ax[2].set_xlabel("Time [s]")

        plt.tight_layout()
        # ax[2].grid()
        # ax[0].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 0], 'o', markersize=1, label='$\max F_x$')
        # ax[1].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 1], 'o', markersize=1, label='$\max F_y$')
        # ax[2].plot(plt_time_all, np.array(plt_eq_fifteen)[:, 2], 'o', markersize=1, label='$\max F_z$')

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
        # plt.savefig("walking_running4" + ".pdf")

        plt.show()

