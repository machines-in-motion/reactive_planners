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
from reactive_planners.lipm_simulator import LipmSimpulator
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


def joint_controller(q, desired_q, qdot, desired_qdot, kp, kd, cnt_array):
    torque = np.zeros((6, 1))
    number_of_joints_per_leg = 3
    for i in range(7, len(q)):
        torque[i - 7] = cnt_array[int((i - 7) / number_of_joints_per_leg)] * (
            kp[i - 7] * (desired_q[i] - q[i])
        ) + cnt_array[int((i - 7) / number_of_joints_per_leg)] * (
            kd[i - 7] * (desired_qdot[i - 1] - qdot[i - 1])
        )
    return torque


class OperationalSpaceDynamics(object):
    def __init__(self, model, endeff_frame_names):
        def getFrameId(name):
            idx = model.getFrameId(name)
            if idx == len(model.frames):
                raise Exception("Unknown frame name: {}".format(name))
            return idx

        self.robot = RobotWrapper(model)
        self.model = model
        self.data = self.robot.data
        self.mass = sum([i.mass for i in self.robot.model.inertias[1:]])
        print(self.robot.model.inertias)
        self.base_id = self.robot.model.getFrameId("base_link")
        self.endeff_frame_names = endeff_frame_names
        self.endeff_ids = [getFrameId(name) for name in endeff_frame_names]
        self.is_init_time = 1

        self.ne = len(self.endeff_ids)
        self.nv = self.model.nv

        self.last_q = None
        self.last_dq = None
        self.p = eye(self.model.nv)
        self.swing_id = self.endeff_ids[1]
        self.stance_id = self.endeff_ids[0]
        self.last_J_c = self.get_world_oriented_frame_jacobian(
            q, self.stance_id
        )[:3]
        self.xdot = self.get_world_oriented_frame_jacobian(q, self.swing_id)[
            :3
        ].dot(qdot)[:3]
        self.last_xdot = self.xdot
        self.mu = 0.5

        # Allocate space for the jacobian and desired velocities.
        # Using two entires for the linear and angular velocity of the base.
        # (self.nv - 6) is the number of jointss for posture regularization
        # self.ne * 3 is the components of foot impulse
        self.J = np.zeros(
            ((self.ne + 2) * 3 + (self.nv - 6) + (self.ne * 3), self.nv)
        )
        self.vel_des = np.zeros(
            ((self.ne + 2) * 3 + (self.nv - 6) + (self.ne * 3), 1)
        )

        # full robot get_jacobian
        self.jacobian = np.zeros((self.ne * 3, self.nv))

    def foot_mass_matrix(self, q):
        foot_mass = np.zeros((self.ne * 3, 3))
        mass_matrix = se3.crba(self.model, self.data, q)
        for i, idx in enumerate(self.endeff_ids):
            self.jacobian[
                3 * i : 3 * (i + 1), :
            ] = self.get_world_oriented_frame_jacobian(q, idx)[:3]
            M = inv(
                self.jacobian[3 * i : 3 * (i + 1), :]
                .dot(inv(mass_matrix))
                .dot(self.jacobian[3 * i : 3 * (i + 1), :].T)
            )
            foot_mass[3 * i : 3 * (i + 1), :] = M
        return foot_mass

    def update_null_space_projection(self, q):
        J_c = self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        self.p = np.matrix(eye(self.nv)) - np.matrix(pinv(J_c)) * J_c
        self.pdot = (self.p - self.last_p) * 1000

    def update_constraint_consistent_inertia(self, q):
        mass_matrix = se3.crba(self.model, self.data, q)
        self.m_c = self.p * mass_matrix + eye(self.nv) - self.p

    def update_constrained_swing_foot_inertia(self, q):
        self.null_space_projection(q)
        self.constraint_consistent_inertia(q)
        J = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        self.lambda_c = inv(J * inv(self.m_c) * self.p * J.T)

    def constrained_swing_foot_inertia(self, q):
        self.update_constrained_swing_foot_inertia(q)
        return self.lambda_c

    def update_c(self, freq=1000):
        J_c = np.matrix(
            self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        )
        self.J_dot = (J_c - self.last_J_c) * freq
        self.c = -pinv(J_c) * self.J_dot

    def projected_nonlinear_terms_h(self, q, qdot):
        self.equation_eleven_mass_matrix(q)
        J = np.matrix(
            self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        )
        h = np.matrix(
            se3.nonLinearEffects(self.model, self.data, q, qdot)
        ).transpose()
        return self.lambda_c * J * inv(self.m_c) * self.p * h

    def projected_gravity(self, q, qdot):
        self.update_constrained_swing_foot_inertia(q)
        J = np.matrix(
            self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        )
        g = np.matrix(
            se3.computeGeneralizedGravity(self.model, self.data, q)
        ).transpose()
        return self.lambda_c * J * inv(self.m_c) * self.p * g

    def xddot(self, q, qdot):
        self.xdot = self.get_world_oriented_frame_jacobian(q, self.swing_id)[
            :3
        ].dot(qdot)[:3]
        return (self.xdot - self.last_xdot) * 1000

    def projected_nonlinear_terms_v(self, q, qdot):
        self.update_onstrained_swing_foot_inertia(q)
        self.update_c()
        J = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        return (
            -self.lambda_c * (self.J_dot + J * inv(self.m_c) * self.c) * qdot
        )

    def rotate_J(self, jac, index):
        world_R_joint = se3.SE3(self.data.oMf[index].rotation, zero(3))
        return world_R_joint.action.dot(jac)

    def get_world_oriented_frame_jacobian(self, q, index):
        return self.rotate_J(
            se3.getFrameJacobian(
                self.model, self.data, index, se3.ReferenceFrame.LOCAL
            ),
            index,
        )

    def swing_force_boundaries(self, h, B, t, q, qdot):  # equation 38
        I = np.matrix(eye(self.nv))
        M = np.matrix(se3.crba(self.robot.model, self.robot.data, q))
        J_c = np.matrix(
            self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        )
        J_X = np.matrix(
            self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        )

        eta = (
            -pinv(J_c.T) * (I - self.p) * (I - M * inv(self.m_c) * self.p) * B
        )
        rho = (
            pinv(J_c.T)
            * (I - self.p)
            * (
                (I - (M * inv(self.m_c) * self.p)) * h
                + M * inv(self.m_c) * self.pdot * qdot
            )
        )
        Q = np.eye(12) * 0.000001
        # p = np.zeros(12)
        p = np.array([1.0 for i in range(12)])
        G = np.zeros((4 + 2 * 12 + 1, 12))
        h = np.zeros(4 + 2 * 12 + 1)
        A = np.zeros((6, 12))
        b = np.zeros(6)
        A[0, 0] = 1
        A[1, 1] = 1
        A[2, 2] = 1
        A[3, 3] = 1
        A[4, 4] = 1
        A[5, 5] = 1

        for i in range(12):
            G[0, i] = (
                sqrt(2) / 2 * self.mu * eta[2, i]
            )  # -sqrt(2) / 2 * (eta_z + rho_z) <= eta_x + rho_x
            G[0, i] -= eta[0, i]

            G[1, i] = (
                sqrt(2) / 2 * self.mu * eta[2, i]
            )  # eta_x + rho_x <= sqrt(2) / 2 * (eta_z + rho_z)
            G[1, i] += eta[0, i]

            G[2, i] = (
                sqrt(2) / 2 * self.mu * eta[2, i]
            )  # -sqrt(2) / 2 * (eta_z + rho_z) <= eta_y + rho_y
            G[2, i] += -eta[1, i]

            G[3, i] = (
                sqrt(2) / 2 * self.mu * eta[2, i]
            )  # eta_y + rho_y <= sqrt(2) / 2 * (eta_z + rho_z)
            G[3, i] += eta[1, i]

        for i in range(12):
            G[4 + i, i] = -1
            h[4 + i] = t
            G[4 + 12 + i, i] = 1
            h[4 + 12 + i] = t

        G[4 + 12 * 2, :] = eta[2, :]
        h[4 + 12 * 2] = -rho[2]

        h[0] = rho[0] - sqrt(2) / 2 * self.mu * rho[2]
        h[1] = -rho[0] - sqrt(2) / 2 * self.mu * rho[2]
        h[2] = rho[1] - sqrt(2) / 2 * self.mu * rho[2]
        h[3] = -rho[1] - sqrt(2) / 2 * self.mu * rho[2]

        alpha = self.lambda_c * J_X * inv(self.m_c) * self.p * B

        result = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        for nb in range(3):
            for i in range(12):
                p[i] = -alpha[nb, i]
            try:
                solution = quadprog_solve_qp(Q, p, G, h, A, b)
                solution = np.array([solution])
                result[0, nb] = (alpha * solution.T)[nb, 0]
            except:
                pass

        for nb in range(3):
            for i in range(12):
                p[i] = alpha[nb, i]
            try:
                solution = quadprog_solve_qp(Q, p, G, h, A, b)
                solution = np.array([solution])
                result[0, nb + 3] = (alpha * solution.T)[nb, 0]
            except:
                pass

        return result

    def evaluate_swing_force_boundaries(self, q, qdot):
        self.update_constrained_swing_foot_inertia(q)
        self.update_c()
        J = np.matrix(
            self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        )
        h = np.matrix(
            se3.nonLinearEffects(self.model, self.data, q, qdot)
        ).transpose()
        B = np.zeros((self.nv, self.nv))
        B[6:, 6:] = eye(self.nv - 6)
        t = 2
        tau = self.swing_force_boundaries(h, B, t, q, qdot)
        tau = tau.T
        return tau

    def forward_robot(self, q, dq):
        # Update the pinocchio model.
        self.last_J_c = self.get_world_oriented_frame_jacobian(
            q, self.stance_id
        )[:3]
        self.last_xdot = self.xdot
        self.last_p = self.p
        self.robot.forwardKinematics(q, dq)
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)

        self.last_q = q.copy()
        self.last_dq = dq.copy()


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
        dcm_reactive_stepper.run(
            time,
            [left_foot_location[0], left_foot_location[1], 0],
            [right_foot_location[0], right_foot_location[1], 0],
            x_com,
            xd_com,
            yaw(q),
            contact_array,
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

    MM = np.matrix(
        [[0.045, -0.002, 0.037], [-0.002, 0.042, 0.0], [0.037, 0.0, 0.065]]
    )
    MM = np.matrix(
        [
            [0.045, 0.005, 0.043],
            [
                0.005,
                0.045,
                0.01,
            ],
            [0.043, 0.01, 0.09],
        ]
    )

    M = np.matrix([[0.045, 0.0, 0.0], [0.0, 0.045, 0.0], [0.0, 0.0, 0.09]])

    # p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "new_traj_obj_fall_2.mp4")
    q = np.matrix(BoltConfig.initial_configuration).T
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    warmup = 5
    kp = np.array([150.0, 150.0, 150.0, 150.0, 150.0, 150.0])
    kd = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0.0, 0.0, 0]
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
    l_p = 0.1035 * 1
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
    past_x = [
        q[0].item(),
        q[1].item() + 0.02,
        0.0,
        q[0].item(),
        q[1].item() - 0.02,
        0.0,
    ]
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

    x_com = np.zeros((3, 1))
    x_com[:] = [[0.0], [0.0], [com_height]]
    xd_com = np.zeros((3, 1))
    x_com_cent = x_com.copy()
    xd_com_cent = xd_com.copy()
    omega = np.sqrt(9.8 / com_height)
    cnt_array = [1, 1]
    time = 0
    control_time = 0
    open_loop = True

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
    dcm_force = [0.0, 0.0, 0.0]
    offset = 0.025
    dcm_reactive_stepper.start()
    inv_kin = OperationalSpaceDynamics(
        robot.pin_robot.model, robot.end_effector_names
    )

    for i in range(5005):
        last_qdot = qdot
        q, qdot = robot.get_state()
        robot.pin_robot.com(q, qdot)
        x_com = robot.pin_robot.com(q, qdot)[0]
        xd_com = robot.pin_robot.com(q, qdot)[1]
        # robot.forward_robot(q, qdot)
        # if i > 600 and i < 670:
        #     print("External Force")
        #     force = np.array([0, -138, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)

        if warmup <= i:
            ###### mass matrix
            m_q = np.matrix(q).transpose()
            m_qdot = np.matrix(qdot).transpose()
            if (
                i > 10
                and 0.010
                < dcm_reactive_stepper.get_step_duration()
                - dcm_reactive_stepper.get_time_from_last_step_touchdown()
            ):
                force_flag = False

            #### test
            # if  i > 10 and t_min + 0.001 > dcm_reactive_stepper.get_time_from_last_step_touchdown() \
            #         and t_min < dcm_reactive_stepper.get_time_from_last_step_touchdown() and not force_flag:
            #     # additional_time = random() * 0.15
            #     external_force(x_com)
            #     force_flag = True
            #     # dcm_reactive_stepper.dcm_vrp_planner_initialization(l_min, l_max, w_min, w_max, t_min + additional_time,
            #     #                                                     t_max, l_p, com_height, weight)

            # if dcm_reactive_stepper.get_is_left_leg_in_contact() == 1: #inv_kin
            #     # if inv_kin.endeff_ids[1] != inv_kin.swing_id:
            #     #     print(i)
            #     inv_kin.swing_id = inv_kin.endeff_ids[1]
            #     inv_kin.stance_id = inv_kin.endeff_ids[0]
            # else:
            #     # if inv_kin.endeff_ids[0] != inv_kin.swing_id:
            #         # print(i)
            #     inv_kin.swing_id = inv_kin.endeff_ids[0]
            #     inv_kin.stance_id = inv_kin.endeff_ids[1]
            # inv_kin.forward_robot(m_q, m_qdot)
            # # print(inv_kin.foot_mass_matrix(m_q))
            # if dcm_reactive_stepper.get_is_left_leg_in_contact() == 1:
            #     plt_foot_mass_r.append(inv_kin.foot_mass_matrix(m_q)[3:6])
            #     plt_eq_11_r.append(inv_kin.constrained_swing_foot_inertia(m_q))
            #     plt_time_r.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
            # else:
            #     plt_foot_mass_l.append(inv_kin.foot_mass_matrix(m_q)[:3])
            #     plt_eq_11_l.append(inv_kin.constrained_swing_foot_inertia(m_q))
            #     plt_time_l.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
            # # print(inv_kin.constrained_swing_foot_inertia(m_q).dot(inv_kin.xddot(m_q, m_qdot)))
            #
            # contact_array = [0, 0]
            # left = bolt_leg_ctrl.imps[0]
            # right = bolt_leg_ctrl.imps[1]
            # left_foot_location = np.array(left.pin_robot.data.oMf[left.frame_end_idx].translation).reshape(-1)
            # right_foot_location = np.array(right.pin_robot.data.oMf[right.frame_end_idx].translation).reshape(-1)
            # detect_contact()
            # if 0.003 < dcm_reactive_stepper.get_time_from_last_step_touchdown() and\
            #     contact_array[dcm_reactive_stepper.get_is_left_leg_in_contact()] == 0:
            #     plt_eq_h.append(inv_kin.projected_nonlinear_terms_h(m_q, m_qdot))
            #     plt_eq_g.append(inv_kin.projected_gravity(m_q, m_qdot))
            #     plt_eq_qdot.append(inv_kin.projected_nonlinear_terms_v(m_q, m_qdot))
            #     plt_eq_qddot.append(inv_kin.constrained_swing_foot_inertia(m_q).dot(inv_kin.xddot(m_q, m_qdot)))
            #     plt_F_M.append(MM.dot(inv_kin.xddot(m_q, m_qdot)))
            #     plt_F_M_new.append(M.dot(inv_kin.xddot(m_q, m_qdot)))
            #     plt_time_all.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())#if you uncomment this line, you should comment below similar line
            # else:
            #     inv_kin.projected_nonlinear_terms_h(m_q, m_qdot)
            #     inv_kin.projected_nonlinear_terms_v(m_q, m_qdot)
            #     inv_kin.constrained_swing_foot_inertia(m_q).dot(inv_kin.xddot(m_q, m_qdot))
            #     dcm_reactive_stepper.get_time_from_last_step_touchdown()
            #
            # contact_array = [0, 0]
            # s = inv_kin.evaluate_swing_force_boundaries(m_q, m_qdot)
            # if s[0].item() > 0.00001:
            #     plt_eq_fifteen.append(inv_kin.evaluate_swing_force_boundaries(m_q, m_qdot))
            #     # plt_time_all.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())#if you uncomment this line, you should comment above similar line
            #
            # # if(len(inv_kin.evaluate_swing_force_boundaries(m_q, m_qdot) != 6)):
            # #     print(inv_kin.evaluate_swing_force_boundaries(m_q, m_qdot))
            # ##### mass matrix done

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

            # if i % 100 == 0:
            #     plot_all_contact_points()
            if open_loop:
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
            else:
                if dcm_reactive_stepper.get_is_left_leg_in_contact():
                    pos_for_plotter = [
                        right_foot_location[0],
                        right_foot_location[1],
                        right_foot_location[2] - offset,
                    ]
                    vel_for_plotter = right_foot_vel
                else:
                    pos_for_plotter = [
                        left_foot_location[0],
                        left_foot_location[1],
                        left_foot_location[2] - offset,
                    ]
                    vel_for_plotter = left_foot_vel

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
            dcm_force = (
                dcm_reactive_stepper.get_forces().copy()
            )  # feed forward
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

            if open_loop:
                x_des_local[2] += offset
                x_des_local[5] += offset

            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                cnt_array = [1, 0]
            else:
                cnt_array = [0, 1]

            plt_time.append(time)
            plt_right_foot_position.append(x_des_local[3:6])
            plt_right_foot_velocity.append(
                dcm_reactive_stepper.get_right_foot_velocity().copy()
            )
            plt_right_foot_acceleration.append(
                dcm_reactive_stepper.get_right_foot_acceleration().copy()
            )
            plt_left_foot_position.append(x_des_local[:3])
            plt_left_foot_velocity.append(
                dcm_reactive_stepper.get_left_foot_velocity().copy()
            )
            plt_left_foot_acceleration.append(
                dcm_reactive_stepper.get_left_foot_acceleration().copy()
            )
            plt_time_from_last_step_touchdown.append(
                dcm_reactive_stepper.get_time_from_last_step_touchdown()
            )
            # plt_duration_before_step_landing.append(dcm_reactive_stepper.duration_before_step_landing)
            plt_current_support_foot.append(
                dcm_reactive_stepper.get_current_support_foot_position().copy()
            )
            # plt_dcm.append(dcm_reactive_stepper.dcm_vrp_planner.get_dcm_local().copy())
            plt_is_left_in_contact.append(
                dcm_reactive_stepper.get_is_left_leg_in_contact()
            )
            plt_next_step_location.append(
                dcm_reactive_stepper.get_next_support_foot_position().copy()
            )
            plt_dcm_local.append(x_com + xd_com / omega)

            if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
                plt_step_time.append(int(i) - warmup)
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
        # w_com[2] += total_mass * 9.81

        F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

        # torque = joint_controller(q, desired_q, qdot, desired_qdot, kp_joint, kd_joint, cnt_array)

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
        if warmup <= i:
            plt_control_time.append(control_time)
        # plt_r.append(r)
        plt_F.append(F)
        plt_x.append(x_des_local)
        plt_tau.append(tau)
        plt_xd_com.append(xd_com.copy())
        plt_qdot_com.append(qdot)
        plt_x_com.append(x_com.copy())
        plt_pos_des_local.append([x_des_local[1], x_des_local[4]])
        # plt_euler_angles.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
        plt_q.append(q[:].copy())
        # plt_qdot.append(inv_kin.xddot(q, qdot))
        # plt_qdot2.append(MM.dot(inv_kin.xddot(q, qdot)))
        # plt_q_com.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
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

    # np.savetxt('plt_time_all.txt', np.array(plt_time_all))
    # np.savetxt('plt_eq_fifteen0.txt', np.array(plt_eq_fifteen)[:, 0])
    # np.savetxt('plt_eq_fifteen1.txt', np.array(plt_eq_fifteen)[:, 1])
    # np.savetxt('plt_eq_fifteen2.txt', np.array(plt_eq_fifteen)[:, 2])
    # np.savetxt('plt_eq_fifteen3.txt', np.array(plt_eq_fifteen)[:, 3])
    # np.savetxt('plt_eq_fifteen4.txt', np.array(plt_eq_fifteen)[:, 4])
    # np.savetxt('plt_eq_fifteen5.txt', np.array(plt_eq_fifteen)[:, 5])

    # fig, ax = plt.subplots(2, 3)
    # ax[0][0].plot(plt_time_r, np.array(plt_foot_mass_r)[:,0,0], 'o', markersize=1, label ='0,0')
    # ax[0][0].legend()
    # ax[0][1].plot(plt_time_r, np.array(plt_foot_mass_r)[:,0,1], 'o', markersize=1, label ='0,1')
    # ax[0][1].legend()
    # ax[0][2].plot(plt_time_r, np.array(plt_foot_mass_r)[:,0,2], 'o', markersize=1, label ='0,2')
    # ax[0][2].legend()
    # ax[1][0].plot(plt_time_r, np.array(plt_foot_mass_r)[:,1,1], 'o', markersize=1, label = '1,1')
    # ax[1][0].legend()
    # ax[1][1].plot(plt_time_r, np.array(plt_foot_mass_r)[:,1,2], 'o', markersize=1, label = '1,2')
    # ax[1][1].legend()
    # ax[1][2].plot(plt_time_r, np.array(plt_foot_mass_r)[:,2,2], 'o', markersize=1, label = '2,2')
    # ax[1][2].legend()
    # fig, ax = plt.subplots(2, 3)
    # ax[0][0].plot(plt_time_l, np.array(plt_foot_mass_l)[:, 0, 0], 'o', markersize=1, label='0,0')
    # ax[0][0].legend()
    # ax[0][1].plot(plt_time_l, np.array(plt_foot_mass_l)[:, 0, 1], 'o', markersize=1, label='0,1')
    # ax[0][1].legend()
    # ax[0][2].plot(plt_time_l, np.array(plt_foot_mass_l)[:, 0, 2], 'o', markersize=1, label='0,2')
    # ax[0][2].legend()
    # ax[1][0].plot(plt_time_l, np.array(plt_foot_mass_l)[:, 1, 1], 'o', markersize=1, label='1,1')
    # ax[1][0].legend()
    # ax[1][1].plot(plt_time_l, np.array(plt_foot_mass_l)[:, 1, 2], 'o', markersize=1, label='1,2')
    # ax[1][1].legend()
    # ax[1][2].plot(plt_time_l, np.array(plt_foot_mass_l)[:, 2, 2], 'o', markersize=1, label='2,2')
    # ax[1][2].legend()
    # fig, ax = plt.subplots(2, 3)
    # ax[0][0].plot(plt_time_r, np.array(plt_eq_11_r)[:, 0, 0], 'o', markersize=1, label='0,0')
    # ax[0][0].legend()
    # ax[0][1].plot(plt_time_r, np.array(plt_eq_11_r)[:, 0, 1], 'o', markersize=1, label='0,1')
    # ax[0][1].legend()
    # ax[0][2].plot(plt_time_r, np.array(plt_eq_11_r)[:, 0, 2], 'o', markersize=1, label='0,2')
    # ax[0][2].legend()
    # ax[1][0].plot(plt_time_r, np.array(plt_eq_11_r)[:, 1, 1], 'o', markersize=1, label='1,1')
    # ax[1][0].legend()
    # ax[1][1].plot(plt_time_r, np.array(plt_eq_11_r)[:, 1, 2], 'o', markersize=1, label='1,2')
    # ax[1][1].legend()
    # ax[1][2].plot(plt_time_r, np.array(plt_eq_11_r)[:, 2, 2], 'o', markersize=1, label='2,2')
    # ax[1][2].legend()
    # fig, ax = plt.subplots(2, 3)
    # ax[0][0].plot(plt_time_l, np.array(plt_eq_11_l)[:, 0, 0], 'o', markersize=1, label='0,0')
    # ax[0][0].legend()
    # ax[0][1].plot(plt_time_l, np.array(plt_eq_11_l)[:, 0, 1], 'o', markersize=1, label='0,1')
    # ax[0][1].legend()
    # ax[0][2].plot(plt_time_l, np.array(plt_eq_11_l)[:, 0, 2], 'o', markersize=1, label='0,2')
    # ax[0][2].legend()
    # ax[1][0].plot(plt_time_l, np.array(plt_eq_11_l)[:, 1, 1], 'o', markersize=1, label='1,1')
    # ax[1][0].legend()
    # ax[1][1].plot(plt_time_l, np.array(plt_eq_11_l)[:, 1, 2], 'o', markersize=1, label='1,2')
    # ax[1][1].legend()
    # ax[1][2].plot(plt_time_l, np.array(plt_eq_11_l)[:, 2, 2], 'o', markersize=1, label='2,2')
    # ax[1][2].legend()
    # np.savetxt('plt_time_l.txt', np.array(plt_time_l))
    # np.savetxt('plt_eq_11_l0.txt', np.array(plt_eq_11_l)[:, 0, 0])
    # np.savetxt('plt_eq_11_l1.txt', np.array(plt_eq_11_l)[:, 0, 1])
    # np.savetxt('plt_eq_11_l2.txt', np.array(plt_eq_11_l)[:, 0, 2])
    # np.savetxt('plt_eq_11_l3.txt', np.array(plt_eq_11_l)[:, 1, 1])
    # np.savetxt('plt_eq_11_l4.txt', np.array(plt_eq_11_l)[:, 1, 2])
    # np.savetxt('plt_eq_11_l5.txt', np.array(plt_eq_11_l)[:, 2, 2])
    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time_all, np.array(plt_eq_h)[:, 0], 'o', markersize=1, label='h')
    # ax[0].plot(plt_time_all, np.array(plt_eq_g)[:, 0], 'o', markersize=1, label='g')
    # ax[0].plot(plt_time_all, np.array(plt_eq_qdot)[:, 0], 'o', markersize=1, label='0')
    # ax[0].legend()
    # ax[1].plot(plt_time_all, np.array(plt_eq_h)[:, 1], 'o', markersize=1, label='h')
    # ax[1].plot(plt_time_all, np.array(plt_eq_g)[:, 1], 'o', markersize=1, label='g')
    # ax[1].plot(plt_time_all, np.array(plt_eq_qdot)[:, 1], 'o', markersize=1, label='1')
    # ax[1].legend()
    # ax[2].plot(plt_time_all, np.array(plt_eq_h)[:, 2], 'o', markersize=1, label='h')
    # ax[2].plot(plt_time_all, np.array(plt_eq_g)[:, 2], 'o', markersize=1, label='g')
    # ax[2].plot(plt_time_all, np.array(plt_eq_qdot)[:, 2], 'o', markersize=1, label='2')
    # ax[2].legend()
    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time, np.array(plt_eq_qdot)[:, 0], 'o', markersize=1, label='0')
    # ax[0].legend()
    # ax[1].plot(plt_time, np.array(plt_eq_qdot)[:, 1], 'o', markersize=1, label='1')
    # ax[1].legend()
    # ax[2].plot(plt_time, np.array(plt_eq_qdot)[:, 2], 'o', markersize=1, label='2')
    # ax[2].legend()
    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time_all, np.array(plt_eq_qddot)[:, 0], 'o', color = 'red', label='Inertia')
    # ax[0].plot(plt_time_all, np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0], 'o', label='Nonlinear Terms')
    # ax[0].legend()
    # ax[1].plot(plt_time_all, np.array(plt_eq_qddot)[:, 1], 'o', color = 'red', label='Inertia')
    # ax[1].plot(plt_time_all, np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1], 'o', label='Nonlinear Terms')
    # ax[1].legend()
    # ax[2].plot(plt_time_all, np.array(plt_eq_qddot)[:, 2], 'o', color = 'red', label='Inertia')
    # ax[2].plot(plt_time_all, np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2], 'o', label='Nonlinear Terms')
    # ax[2].legend()

    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time_all, np.array(plt_F_M_new)[:, 0] - (np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0]), 'o', markersize=1 , color = 'red', label='Diagonal')
    # ax[0].plot(plt_time_all, np.array(plt_F_M)[:, 0] - (np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0]), 'o', markersize=1 , label='Non-diagonal')
    # ax[0].plot(plt_time_all, np.array(plt_F_M_new)[:, 0] - 0.4  - (np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0]), 'og', markersize=1 , label='Non-diagonal')
    # ax[0].legend()
    # ax[1].plot(plt_time_all, np.array(plt_F_M_new)[:, 1] - (np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1]), 'o', markersize=1 , color = 'red', label='Diagonal')
    # ax[1].plot(plt_time_all, np.array(plt_F_M)[:, 1] - (np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1]), 'o', markersize=1 , label='Non-diagonal')
    # ax[1].plot(plt_time_all, np.array(plt_F_M_new)[:, 1] + 0  - (np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1]), 'og', markersize=1 , label='Non-diagonal')
    # ax[1].legend()
    # ax[2].plot(plt_time_all, np.array(plt_F_M_new)[:, 2] - (np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2]), 'o', markersize=1 , color = 'red', label='Diagonal')
    # ax[2].plot(plt_time_all, np.array(plt_F_M)[:, 2] - (np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2]), 'o', markersize=1 , label='Non-diagonal')
    # ax[2].plot(plt_time_all, np.array(plt_F_M_new)[:, 2] + 0.8 - (np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2]), 'og', markersize=1 , label='Non-diagonal')
    # ax[2].legend()
    #
    #
    # np.savetxt('plt_D_error0.txt', np.array(plt_F_M_new)[:, 0] - (np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0]))
    # np.savetxt('plt_D_error1.txt', np.array(plt_F_M_new)[:, 1] - (np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1]))
    # np.savetxt('plt_D_error2.txt', np.array(plt_F_M_new)[:, 2] - (np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2]))
    # np.savetxt('plt_ND_error0.txt', np.array(plt_F_M)[:, 0] - (np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0]))
    # np.savetxt('plt_ND_error1.txt', np.array(plt_F_M)[:, 1] - (np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1]))
    # np.savetxt('plt_ND_error2.txt', np.array(plt_F_M)[:, 2] - (np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2]))
    # np.savetxt('plt_time_all_D_ND.txt', np.array(plt_time_all))
    #
    # np.savetxt('plt_F0.txt', np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0])
    # np.savetxt('plt_F1.txt', np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1])
    # np.savetxt('plt_F2.txt', np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2])
    # np.savetxt('plt_non_linear0.txt', np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0])
    # np.savetxt('plt_non_linear1.txt', np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1])
    # np.savetxt('plt_non_linear2.txt', np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2])
    # np.savetxt('plt_qddot_MassMatrix0.txt', np.array(plt_eq_qddot)[:, 0])
    # np.savetxt('plt_qddot_MassMatrix1.txt', np.array(plt_eq_qddot)[:, 1])
    # np.savetxt('plt_qddot_MassMatrix2.txt', np.array(plt_eq_qddot)[:, 2])
    # np.savetxt('plt_Error_D_M0.txt', np.array(plt_F_M_new)[:, 0] - np.array(plt_eq_qddot)[:, 0])
    # np.savetxt('plt_Error_D_M1.txt', np.array(plt_F_M_new)[:, 1] - np.array(plt_eq_qddot)[:, 1])
    # np.savetxt('plt_Error_D_M2.txt', np.array(plt_F_M_new)[:, 2] - np.array(plt_eq_qddot)[:, 2])
    # np.savetxt('plt_Error_ND_M0.txt', np.array(plt_F_M)[:, 0] - np.array(plt_eq_qddot)[:, 0])
    # np.savetxt('plt_Error_ND_M1.txt', np.array(plt_F_M)[:, 1] - np.array(plt_eq_qddot)[:, 1])
    # np.savetxt('plt_Error_ND_M2.txt', np.array(plt_F_M)[:, 2] - np.array(plt_eq_qddot)[:, 2])
    # np.savetxt('plt_h0.txt', np.array(plt_eq_h)[:, 0])
    # np.savetxt('plt_h1.txt', np.array(plt_eq_h)[:, 1])
    # np.savetxt('plt_h2.txt', np.array(plt_eq_h)[:, 2])
    # np.savetxt('plt_qdot0.txt', np.array(plt_eq_qdot)[:, 0])
    # np.savetxt('plt_qdot1.txt', np.array(plt_eq_qdot)[:, 1])
    # np.savetxt('plt_qdot2.txt', np.array(plt_eq_qdot)[:, 2])
    # np.savetxt('plt_eq_g0.txt', np.array(plt_eq_g)[:, 0])
    # np.savetxt('plt_eq_g1.txt', np.array(plt_eq_g)[:, 1])
    # np.savetxt('plt_eq_g2.txt', np.array(plt_eq_g)[:, 2])

    # ax[0].set_ylabel("Force [N]")
    # ax[1].set_ylabel("Force [N]")
    # ax[2].set_ylabel("Force [N]")
    # ax[2].set_xlabel("Time [ms]")
    # plt.tight_layout()
    # plt.savefig("eq11" + ".pdf")
    #
    # np.savetxt('plt_F_M0WF.txt', np.array(plt_F_M)[:, 0])
    # np.savetxt('plt_F_M1WF.txt', np.array(plt_F_M)[:, 1])
    # np.savetxt('plt_F_M2WF.txt', np.array(plt_F_M)[:, 2])
    #
    # np.savetxt('plt_F_M20WF.txt', np.array(plt_eq_qddot)[:, 0] + np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0])
    # np.savetxt('plt_F_M21WF.txt', np.array(plt_eq_qddot)[:, 1] + np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1])
    # np.savetxt('plt_F_M22WF.txt', np.array(plt_eq_qddot)[:, 2] + np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2])
    #
    # np.savetxt('plt_time_all_F_MWF.txt', np.array(plt_time_all))

    # print(shape(plt_eq_fifteen))

    # print(plt_eq_fifteen)

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
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 1], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 1], label="right")
    plt.plot(plt_control_time, np.array(plt_x_com)[warmup:, 1], label="com")
    plt.plot(
        plt_control_time, np.array(plt_xd_com)[warmup:, 1], label="xd_com"
    )
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
    plt.plot(
        plt_time,
        np.array(plt_next_step_location)[:, 1],
        label="next_step_location",
    )
    # plt.plot(plt_time, np.array(plt_dcm)[:, 1], label="dcm")
    plt.plot(
        plt_time,
        np.array(plt_left_eef_real_pos)[warmup:, 1],
        label="left_eef_real_pos",
    )
    plt.plot(
        plt_time,
        np.array(plt_right_eef_real_pos)[warmup:, 1],
        label="right_eef_real_pos",
    )
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 1], label="current_support_foot")
    # plt.plot(plt_time, plt_pos_des_local[warmup + 1:], label = "pos des_local_eef")
    plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    # plt.figure("q")
    # plt.plot(np.array(plt_qdot)[:, :, 0], label="plt_qddot")
    # plt.plot(np.array(plt_qdot2)[:, :, 0], label="plt_M*qddot")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("x")
    plt.plot(
        plt_time, np.array(plt_left_foot_position)[:, 0], label="des_left"
    )
    plt.plot(
        plt_time, np.array(plt_right_foot_position)[:, 0], label="des_right"
    )
    plt.plot(plt_control_time, np.array(plt_x_com)[warmup:, 0], label="com")
    plt.plot(
        plt_control_time, np.array(plt_xd_com)[warmup:, 0], label="xd_com"
    )
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    plt.plot(
        plt_time,
        np.array(plt_next_step_location)[:, 0],
        label="next_step_location",
    )
    # plt.plot(plt_time, np.array(plt_dcm)[:, 0], label="dcm")
    plt.plot(
        plt_time,
        np.array(plt_left_eef_real_pos)[warmup:, 0],
        label="left_eef_real_pos",
    )
    plt.plot(
        plt_time,
        np.array(plt_right_eef_real_pos)[warmup:, 0],
        label="right_eef_real_pos",
    )
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 0], label="current_support_foot")
    # plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:], label="plt_duration_before_step_landing")
    # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
    plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("tau")
    plt.plot(np.array(plt_tau)[:, :], label="tau")
    plt.legend()

    plt.figure("z")
    plt.plot(
        plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact"
    )
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 2], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 2], label="right")
    plt.plot(plt_control_time, np.array(plt_x_com)[warmup:, 2], label="com")
    # plt.plot(plt_time, np.array(plt_dcm_local)[:, 2], label="dcm_local")
    plt.plot(
        plt_time,
        np.array(plt_left_eef_real_pos)[warmup:, 2],
        label="left_eef_real_pos",
    )
    plt.plot(
        plt_time,
        np.array(plt_right_eef_real_pos)[warmup:, 2],
        label="right_eef_real_pos",
    )
    plt.legend()
    for time in plt_step_time:
        plt.axvline(time / 1000)

    # plt.figure("q")
    # plt.plot(plt_time, np.array(plt_q_com)[:, 3], label="x")
    # plt.plot(plt_time, np.array(plt_q_com)[:, 4], label="y")
    # plt.plot(plt_time, np.array(plt_q_com)[:, 5], label="z")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("F")
    plt.plot(plt_time, plt_F[warmup:], label="F")
    plt.plot(
        plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact"
    )
    plt.legend()

    new_ = True
    plt.figure("Z")
    plt.plot(
        plt_time[:],
        np.array(plt_left_eef_real_pos)[warmup:, 2],
        label="left_z",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_right_eef_real_pos)[warmup:, 2],
        label="right_z",
    )
    plt.plot(
        plt_time[:], np.array(plt_left_foot_position)[:, 2], label="des_left_z"
    )
    plt.plot(
        plt_time[:],
        np.array(plt_right_foot_position)[:, 2],
        label="des_right_z",
    )
    # plt.plot(plt_time[230:], np.array(plt_next_step_location)[230:, 2], label="next_step_location_z")
    plt.legend()
    # np.savetxt('plt_left_eef_real_posz' + str(new_) +'.txt', np.array(plt_left_eef_real_pos)[warmup:, 2])
    # np.savetxt('plt_right_eef_real_posz' + str(new_) +'.txt', np.array(plt_right_eef_real_pos)[warmup:, 2])
    # np.savetxt('plt_left_foot_positionz' + str(new_) +'.txt', np.array(plt_left_foot_position)[:, 2])
    # np.savetxt('plt_right_foot_positionz' + str(new_) +'.txt', np.array(plt_right_foot_position)[:, 2])

    plt.figure("xy")
    plt.plot(
        plt_time[:],
        np.array(plt_left_eef_real_pos)[warmup:, 0],
        label="left_x",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_left_eef_real_pos)[warmup:, 1],
        label="left_y",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_right_eef_real_pos)[warmup:, 0],
        label="right_x",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_right_eef_real_pos)[warmup:, 1],
        label="right_y",
    )
    plt.plot(
        plt_time[:], np.array(plt_left_foot_position)[:, 0], label="des_left_x"
    )
    plt.plot(
        plt_time[:], np.array(plt_left_foot_position)[:, 1], label="des_lef_y"
    )
    plt.plot(
        plt_time[:],
        np.array(plt_right_foot_position)[:, 0],
        label="des_right_x",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_right_foot_position)[:, 1],
        label="des_right_y",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_next_step_location)[:, 0],
        label="next_step_location_x",
    )
    plt.plot(
        plt_time[:],
        np.array(plt_next_step_location)[:, 1],
        label="next_step_location_y",
    )
    # plt.plot(plt_time[230:], np.array(plt_next_step_location)[230:, 2], label="next_step_location_z")
    plt.legend()
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
