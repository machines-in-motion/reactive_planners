""" @namespace Demos of solo12 step adjustment
@file
@copyright Copyright (c) 2017-2021,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import numpy as np
import pybullet as p
from robot_properties_solo.config import Solo12Config
from robot_properties_solo.solo12wrapper import Solo12Robot
from mim_control.robot_centroidal_controller import RobotCentroidalController
from mim_control.robot_impedance_controller import RobotImpedanceController
from reactive_planners.lipm_simulator import LipmSimpulator
from reactive_planners_cpp import QuadrupedDcmReactiveStepper
import pinocchio as pin
from scipy.spatial.transform import Rotation
from bullet_utils.env import BulletEnvWithGround

np.set_printoptions(suppress=True, precision=2)
pin.switchToNumpyArray()

def zero_cnt_gain(kp, cnt_array):
    gain = np.array(kp).copy()
    for i, v in enumerate(cnt_array):
        if v == 1:
            gain[3 * i : 3 * (i + 1)] = 0.0
    return gain


def yaw(q):
    return np.array(
        Rotation.from_quat([np.array(q)[3:7]]).as_euler("xyz", degrees=False)
    )[0, 2]

data_collector = None

# from RAI.data_collector import DataCollector

# data_collector = DataCollector()

# Create a robot instance. This initializes the simulator as well.
env = BulletEnvWithGround()
robot = Solo12Robot()
env.add_robot(robot)
tau = np.zeros(12)

time = 0
sim_freq = 1000  # Hz
ctrl_freq = 1000
plan_freq = 1000

p.resetDebugVisualizerCamera(1.6, 50, -35, (0.0, 0.0, 0.0))
p.setTimeStep(1.0 / sim_freq)
p.setRealTimeSimulation(0)
for ji in range(12):
    p.changeDynamics(
        robot.robotId,
        ji,
        linearDamping=0.04,
        angularDamping=0.04,
        restitution=0.0,
        lateralFriction=4.0,
        spinningFriction=0.04,
    )
q = np.array(Solo12Config.initial_configuration)
q[0] = 0.
q[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., np.pi/4)).coeffs() #
qdot = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q, qdot)
total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
warmup = 500
kp = np.array(12 * [50.0])
kd = 12 * [5.0]
robot_config = Solo12Config()
config_file = robot_config.ctrl_path
solo_leg_ctrl = RobotImpedanceController(robot, config_file)
centr_controller = RobotCentroidalController(
    robot_config,
    mu=0.6,
    kc=[0, 0, 200],
    dc=[10, 10, 10],
    kb=[25, 25, 25.],
    db=[22.5, 22.5, 22.5],
    qp_penalty_lin=[1e0, 1e0, 1e6],
    qp_penalty_ang=[1e6, 1e6, 1e6],
)
is_left_leg_in_contact = True
l_min = -0.1
l_max = 0.1
w_min = -0.08
w_max = 0.2
t_min = 0.1
t_max = 1.0
l_p = 0.00  # Pelvis width
com_height = 0.25
weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
mid_air_foot_height = 0.05
control_period = 0.001
planner_loop = 0.010
# init poses
robot.pin_robot.framesForwardKinematics(q)
base_pose = q[:7]
front_left_foot_position = robot.pin_robot.data.oMf[
    solo_leg_ctrl.imp_ctrl_array[0].frame_end_idx].translation
front_right_foot_position = robot.pin_robot.data.oMf[
    solo_leg_ctrl.imp_ctrl_array[1].frame_end_idx].translation
hind_left_foot_position = robot.pin_robot.data.oMf[
    solo_leg_ctrl.imp_ctrl_array[2].frame_end_idx].translation
hind_right_foot_position = robot.pin_robot.data.oMf[
    solo_leg_ctrl.imp_ctrl_array[3].frame_end_idx].translation

v_des = np.array([0.0, 0.0, 0.0])
y_des = 0.2 # Speed of the yaw angle

quadruped_dcm_reactive_stepper = QuadrupedDcmReactiveStepper()
quadruped_dcm_reactive_stepper.initialize(
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
    base_pose,
    front_left_foot_position,
    front_right_foot_position,
    hind_left_foot_position,
    hind_right_foot_position,
)

quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des)
quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory()

plt_timer = []
plt_front_left_foot_position = []
plt_front_right_foot_position = []
plt_hind_left_foot_position = []
plt_hind_right_foot_position = []
plt_des_front_left_foot_position = []
plt_des_front_right_foot_position = []
plt_des_hind_left_foot_position = []
plt_des_hind_right_foot_position = []


x_com = [[0.0], [0.0], [com_height]]
com_des = np.array([0., 0.0])
yaw_des = yaw(q)
cnt_array = [1, 1]
control_time = 0
open_loop = True
dcm_force = np.array([0.0, 0.0, 0.0])
offset = 0.015  # foot radius
# quadruped_dcm_reactive_stepper.start()

traj_q = np.zeros((8000 + warmup, 19))

for i in range(traj_q.shape[0]):
    if i == 1000:
        v_des = np.array([0.0, 0.0, 0.])

    if i == 4000:
        p.applyExternalForce(robot.robotId, -1, [-200., 200., 0.], [0., 0., 0.], p.LINK_FRAME)

    last_qdot = qdot
    q, qdot = robot.get_state()
    robot.pin_robot.com(q, qdot)
    robot.update_pinocchio(q, qdot)
    x_com = robot.pin_robot.com(q, qdot)[0]
    xd_com = robot.pin_robot.com(q, qdot)[1]
    traj_q[i] = q

    com_des += v_des[:2] * 0.001
    yaw_des += y_des * 0.001

    if i == warmup:
        quadruped_dcm_reactive_stepper.start()
    #     elif i > warmup :
    else:
        FL = solo_leg_ctrl.imp_ctrl_array[0]
        FR = solo_leg_ctrl.imp_ctrl_array[1]
        HL = solo_leg_ctrl.imp_ctrl_array[2]
        HR = solo_leg_ctrl.imp_ctrl_array[3]

        # Define left as front left and back right leg
        front_left_foot_position = robot.pin_robot.data.oMf[FL.frame_end_idx].translation
        front_right_foot_position = robot.pin_robot.data.oMf[FR.frame_end_idx].translation
        hind_left_foot_position = robot.pin_robot.data.oMf[HL.frame_end_idx].translation
        hind_right_foot_position = robot.pin_robot.data.oMf[HR.frame_end_idx].translation
        front_left_foot_velocity = pin.getFrameVelocity(
            robot.pin_robot.model, robot.pin_robot.data, FL.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        front_right_foot_velocity = pin.getFrameVelocity(
            robot.pin_robot.model, robot.pin_robot.data, FR.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        hind_left_foot_velocity = pin.getFrameVelocity(
            robot.pin_robot.model, robot.pin_robot.data, HL.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        hind_right_foot_velocity = pin.getFrameVelocity(
            robot.pin_robot.model, robot.pin_robot.data, HR.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear

        quadruped_dcm_reactive_stepper.run(
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
            yaw(q),
            not open_loop,
        )

        x_des_local = []
        x_des_local.extend(quadruped_dcm_reactive_stepper.get_front_left_foot_position())
        x_des_local.extend(quadruped_dcm_reactive_stepper.get_front_right_foot_position())
        x_des_local.extend(quadruped_dcm_reactive_stepper.get_hind_left_foot_position())
        x_des_local.extend(quadruped_dcm_reactive_stepper.get_hind_right_foot_position())

        cnt_array = quadruped_dcm_reactive_stepper.get_contact_array()
    #     else:
    #         cnt_array = [1, 1, 1, 1]
    #         x_des_local = np.array([
    #              0.195,  0.147, 0.015,
    #              0.195, -0.147, 0.015,
    #             -0.195,  0.147, 0.015,
    #             -0.195, -0.147, 0.015
    #         ])

    for j in range(4):
        imp = solo_leg_ctrl.imp_ctrl_array[j]
        x_des_local[3 * j : 3 * (j + 1)] -= imp.pin_robot.data.oMf[
            imp.frame_root_idx
        ].translation

    w_com = centr_controller.compute_com_wrench(
        q.copy(),
        qdot.copy(),
        [com_des[0], com_des[1], com_height],
        v_des,
        pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., yaw_des)).coeffs(),
        [0.0, 0.0, y_des],
    )

    F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

    des_vel = np.concatenate(
        (
            quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity(),
        )
    )

    if cnt_array[0] == 1:
        F[3:6] = -dcm_force[:3]
        F[6:9] = -dcm_force[:3]
    else:
        F[0:3] = -dcm_force[:3]
        F[9:12] = -dcm_force[:3]

    tau = solo_leg_ctrl.return_joint_torques(
        q.copy(),
        qdot.copy(),
        zero_cnt_gain(kp, cnt_array),
        zero_cnt_gain(kd, cnt_array),
        x_des_local,
        des_vel,
        F,
    )
    control_time += 0.001

    robot.send_joint_command(tau)
    p.stepSimulation()

    if data_collector is not None:
        data_collector.add_variable(control_time, "time", "s")
        data_collector.add_vector_3d(
            quadruped_dcm_reactive_stepper.get_front_left_foot_position(), "front_left_foot_position", "m")
        data_collector.add_vector_3d(
            quadruped_dcm_reactive_stepper.get_front_right_foot_position(), "front_right_foot_position", "m")
        data_collector.add_vector_3d(
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position(), "hind_left_foot_position", "m")
        data_collector.add_vector_3d(
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position(), "hind_right_foot_position", "m")


    plt_timer.append(control_time)
    plt_front_left_foot_position.append(
        front_left_foot_position.copy())
    plt_front_right_foot_position.append(
        front_right_foot_position.copy())
    plt_hind_left_foot_position.append(
        hind_left_foot_position.copy())
    plt_hind_right_foot_position.append(
        hind_right_foot_position.copy())
    plt_des_front_left_foot_position.append(
        quadruped_dcm_reactive_stepper.get_front_left_foot_position())
    plt_des_front_right_foot_position.append(
        quadruped_dcm_reactive_stepper.get_front_right_foot_position())
    plt_des_hind_left_foot_position.append(
        quadruped_dcm_reactive_stepper.get_hind_left_foot_position())
    plt_des_hind_right_foot_position.append(
        quadruped_dcm_reactive_stepper.get_hind_right_foot_position())

quadruped_dcm_reactive_stepper.stop()


from matplotlib import pyplot as plt

plt.figure("y")
# plt.plot(plt_timer, np.array(plt_left_foot_position)[:, 1], label="left")
# plt.plot(plt_timer, np.array(plt_right_foot_position)[:, 1], label="right")
# plt.plot(plt_timer, np.array(plt_x_com)[warmup:, 1], label="com")
# plt.plot(
#     plt_timer, np.array(plt_xd_com)[warmup:, 1], label="xd_com"
# )
# plt.plot(plt_timer, np.array(plt_dcm_local)[:, 1], label="dcm_local")
# plt.plot(
#     plt_timer,
#     np.array(plt_next_step_location)[:, 1],
#     label="next_step_location",
# )

plt.plot(
    plt_timer[warmup:],
    np.array(plt_front_left_foot_position)[warmup:, 1],
    label="left_y",
)
plt.plot(
    plt_timer[warmup:],
    np.array(plt_des_front_left_foot_position)[warmup:, 1],
    label="des_left_y",
)
# plt.plot(
#     plt_timer[:],
#     np.array(plt_right_eef_real_pos)[warmup:, 0],
#     label="right_x",
# )
plt.plot(
    plt_timer[warmup:],
    np.array(plt_hind_left_foot_position)[warmup:, 1],
    label="right_y",
)
plt.plot(
    plt_timer[warmup:],
    np.array(plt_des_hind_left_foot_position)[warmup:, 1],
    label="des_right_y",
)
plt.legend()
plt.show()