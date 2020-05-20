""" @namespace Demos of Bolt step adjustment
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import numpy as np
# np.set_printoptions(precision=2, suppress=True)
import pybullet as p
from matplotlib import pyplot as plt
from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot
from py_blmc_controllers.bolt_centroidal_controller import BoltCentroidalController
from py_blmc_controllers.bolt_impedance_controller import BoltImpedanceController
from py_reactive_planners.lipm_simulator import LipmSimpulator
from pinocchio.utils import zero, eye
import pinocchio as se3
from pinocchio import RobotWrapper
import time as Time
from scipy.spatial.transform import Rotation as R
from reactive_planners import DcmReactiveStepper
from numpy.linalg import inv, det, pinv
from math import sqrt


def zero_cnt_gain(kp, cnt_array):
    gain = np.array(kp).copy()
    for i, v in enumerate(cnt_array):
        if v == 1:
            gain[3 * i:3 * (i + 1)] = 0.
    return gain


def joint_controller(q, desired_q, qdot, desired_qdot, kp, kd, cnt_array):
    torque = np.zeros((6, 1))
    number_of_joints_per_leg = 3
    for i in range(7, len(q)):
        torque[i - 7] = (cnt_array[int((i - 7) / number_of_joints_per_leg)] * (kp[i - 7] * (desired_q[i] - q[i])) + \
                         cnt_array[int((i - 7) / number_of_joints_per_leg)] * (
                                 kd[i - 7] * (desired_qdot[i - 1] - qdot[i - 1])))
    return torque


class PointContactInverseKinematics(object):
    def __init__(self, model, endeff_frame_names):
        def getFrameId(name):
            idx = model.getFrameId(name)
            if idx == len(model.frames):
                raise Exception('Unknown frame name: {}'.format(name))
            return idx

        self.robot = RobotWrapper(model)
        self.model = model
        self.data = self.robot.data
        self.mass = sum([i.mass for i in self.robot.model.inertias[1:]])
        self.base_id = self.robot.model.getFrameId('base_link')
        self.endeff_frame_names = endeff_frame_names
        self.endeff_ids = [getFrameId(name) for name in endeff_frame_names]
        self.is_init_time=1

        self.ne = len(self.endeff_ids)
        self.nv = self.model.nv

        self.last_q = None
        self.last_dq = None
        self.p = eye(self.model.nv)
        self.swing_id = self.endeff_ids[1]
        self.stance_id = self.endeff_ids[0]
        self.last_J_c = self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        self.xdot = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3].dot(qdot)[:3]
        self.last_xdot = self.xdot

        # Allocate space for the jacobian and desired velocities.
        # Using two entires for the linear and angular velocity of the base.
        # (self.nv - 6) is the number of jointss for posture regularization
        # self.ne * 3 is the components of foot impulse
        self.J = np.zeros(((self.ne + 2) * 3 + (self.nv - 6) + (self.ne * 3), self.nv))
        self.vel_des = np.zeros(((self.ne + 2) * 3 + (self.nv - 6) + (self.ne * 3), 1))

        #full robot get_jacobian
        self.jacobian = np.zeros((self.ne * 3, self.nv))

    def foot_mass_matrix(self, q):
        foot_mass = np.zeros((self.ne  * 3, 3))
        mass_matrix = se3.crba(self.model, self.data, q)
        for i, idx in enumerate(self.endeff_ids):
            # print(idx)
            self.jacobian[3 * i:3 * (i + 1), :] = self.get_world_oriented_frame_jacobian(q, idx)[:3]
            # print("jac", self.jacobian[3 * i:3 * (i + 1), :])
            # print("ma", mass_matrix)
            M = inv(self.jacobian[3 * i:3 * (i + 1), :].dot(inv(mass_matrix)).dot(self.jacobian[3 * i:3 * (i + 1), :].T))
            foot_mass[3 * i:3 * (i + 1), :] = M
        return foot_mass

    def update_p(self, q):
        J_c = self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        self.p = eye(self.nv) - pinv(J_c) * J_c

    def update_m_c(self, q):
        mass_matrix = se3.crba(self.model, self.data, q)
        self.m_c = self.p * mass_matrix + eye(self.nv) - self.p

    def eleven_equation_mass_matrix(self, q):
        self.update_p(q)
        self.update_m_c(q)
        J = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        self.lambda_c = inv(J * inv(self.m_c) * self.p * J.T)
        return self.lambda_c

    def update_c(self):
        J_c = self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        self.J_dot = ((J_c - self.last_J_c) * 1000)
        self.c = -pinv(J_c) * self.J_dot

    def eleven_equation_h(self, q, qdot):
        self.eleven_equation_mass_matrix(q)
        J = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        h = se3.nonLinearEffects(self.model, self.data, q, qdot)
        return self.lambda_c * J * inv(self.m_c) * self.p * h

    def xddot(self, q, qdot):
        self.xdot = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3].dot(qdot)[:3]
        return (self.xdot - self.last_xdot) * 1000

    def eleven_equation_q_dot(self, q, qdot):
        self.eleven_equation_mass_matrix(q)
        self.update_c()
        J = self.get_world_oriented_frame_jacobian(q, self.swing_id)[:3]
        return -self.lambda_c * (self.J_dot + J * inv(self.m_c) * self.c) * qdot

    def rotate_J(self, jac, index):
        world_R_joint = se3.SE3(self.data.oMf[index].rotation, zero(3))
        return world_R_joint.action.dot(jac)

    def get_world_oriented_frame_jacobian(self, q, index):
        return self.rotate_J(
            se3.getFrameJacobian(self.model, self.data, index, se3.ReferenceFrame.LOCAL),
            index)

    def forward_robot(self, q, dq):
        # Update the pinocchio model.
        self.last_J_c = self.get_world_oriented_frame_jacobian(q, self.stance_id)[:3]
        self.last_xdot = self.xdot
        self.robot.forwardKinematics(q, dq)
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)

        self.last_q = q.copy()
        self.last_dq = dq.copy()


def yaw(q):
    return np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, 2]

def plot(f):
    M = [[0.06, 0.0, 0.03],
         [0.0, 0.05, 0.01],
         [0.03, 0.01, 0.05]]
    M_inv = inv(M)
    sum_f = [0, 0, 0]
    x2 = []
    x3 = []
    v = []
    # f[0] = 10
    # f[3] = -10
    time = 0.001
    A = np.matrix([[1., time, 0., 0., 0., 0.],
                   [0., 1., 0., 0., 0., 0.],
                   [0., 0., 1., time, 0., 0.],
                   [0., 0., 0., 1., 0., 0.],
                   [0., 0., 0., 0., 1., time],
                   [0., 0., 0., 0., 0., 1.]])
    B = np.matrix([[time * time / 2, 0., 0.],
                   [time, 0., 0.],
                   [0., time * time / 2, 0.],
                   [0., time, 0.],
                   [0., 0., time * time / 2],
                   [0., 0., time]])
    x0 = pos_for_plotter
    v0 = vel_for_plotter
    x3.append(x0)
    print("plot f", f)
    for i in range(len(f) / 3):
        sum_f += f[i * 3: i * 3 + 3]
        x2.append(0.5 * f[i * 3: i * 3 + 3].dot(M_inv) * time * time + x0 + v0 * time)
        sum = pos_for_plotter + vel_for_plotter * (i + 1) * time + \
              np.array(B * np.matrix(f[i * 3: i * 3 + 3].dot(M_inv)).transpose())[::2, 0]
        final_A = A.copy()
        for k in range(1, i + 1):
            sum[:] += np.array(final_A * B * np.matrix(f[(i - k) * 3: (i - k) * 3 + 3].dot(M_inv)).transpose())[::2, 0]
            final_A *= A
        x3.append(sum)
        x0 = 0.5 * f[i * 3: i * 3 + 3].dot(M_inv) * time * time + x0 + v0 * time
        v0 = v0 + f[i * 3: i * 3 + 3].dot(M_inv) * time
        v.append(v0)
    plt.plot(x2, label="x2")
    plt.plot(x3, label="x3")
    plt.plot(v, label="v")
    plt.legend()
    plt.grid()
    plt.show()

def dist(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

def closed_loop():
    global  open_loop
    open_loop = False
    if dcm_reactive_stepper.get_is_left_leg_in_contact():
        dcm_reactive_stepper.set_right_foot_position(right_foot_location)
        dcm_reactive_stepper.set_right_foot_velocity(right_foot_vel)
    else:
        dcm_reactive_stepper.set_left_foot_position(left_foot_location)
        dcm_reactive_stepper.set_left_foot_velocity(left_foot_vel)

def detect_contact():
    for contact in p.getContactPoints():
        if dist(left_foot_location, contact[5]) < 0.02 and dist(right_foot_location, contact[5]) > dist(left_foot_location, contact[5]):
            contact_array[0] = 1
        if dist(right_foot_location, contact[5]) < 0.02 and dist(left_foot_location, contact[5]) > dist(right_foot_location, contact[5]):
            contact_array[1] = 1
    print(contact_array)

def create_box(halfExtents, collisionFramePosition):
    cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents, collisionFramePosition=collisionFramePosition)
    mass = 0  # static box
    p.createMultiBody(mass, cuid)
    p.changeDynamics(cuid, -1, linearDamping=.04,
                     angularDamping=0.04, restitution=0.0, lateralFriction=2.)

def plot_all_contact_points():
    plt_next_support_foot_position = []
    for j in range(100):
        if j / 100.0 + t_min >= t_max:
            break
        # print("j" , j)
        # print("1")
        dcm_reactive_stepper.dcm_vrp_planner_initialization(l_min, l_max, w_min, w_max, t_min + j / 100.0, t_max, l_p,
                                                            com_height, weight)
        # print("2")
        dcm_reactive_stepper.run(time, [left_foot_location[0], left_foot_location[1], 0],
                                 [right_foot_location[0], right_foot_location[1], 0], x_com, xd_com, yaw(q),
                                 contact_array)
        # print("3")
        print("Lhum next support foot position", dcm_reactive_stepper.get_next_support_foot_position())
        plt_next_support_foot_position.append(dcm_reactive_stepper.get_next_support_foot_position().copy())
        # print("4")
    print("6")
    plt.figure("dcm")
    plt.plot(np.array(plt_next_support_foot_position)[:, 0], label="x")
    plt.plot(np.array(plt_next_support_foot_position)[:, 1], label="y")
    print("7")
    plt.legend()
    plt.show()
    dcm_reactive_stepper.dcm_vrp_planner_initialization(l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight)
    print("8")

if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    robot = BoltRobot()
    tau = np.zeros(6)
    p.resetDebugVisualizerCamera(1.6, 50, -35, (0., 0., 0.))
    p.setTimeStep(0.001)
    p.setRealTimeSimulation(0)

    for ji in range(8):
        p.changeDynamics(robot.robotId, ji, linearDamping=.04,
                         angularDamping=0.04, restitution=0.0, lateralFriction=2.0)

    # p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "new_traj_obj_fall_1.mp4")

    create_box([0.05, 0.4, 0.01], [0.15, 0., 0.])
    create_box([0.05, 0.2, 0.02], [0.1, -0.1, 0.])
    create_box([0.06, 0.4, 0.03], [0.25, 0.1, 0.])
    create_box([0.25, 0.1, 0.01], [0.25, 0.15, 0.])

    # create_box([0.08, 0.4, 0.01], [0.15, 0., 0.])
    # create_box([0.05, 0.2, 0.02], [0.1, -0.1, 0.])
    # create_box([0.03, 0.4, 0.03], [0.3, 0.1, 0.])
    # create_box([0.25, 0.1, 0.01], [0.25, 0.15, 0.])

    q = np.matrix(BoltConfig.initial_configuration).T
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    total_mass = 1.13  # sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    warmup = 10
    kp = np.array([150., 150., 150., 150., 150., 150.])
    kd = [15., 15., 15., 15., 15., 15.]
    kp_joint = np.array([2., 2., 2., 2., 2., 2.])
    kd_joint = [.1, .01, .01, .1, .01, .01]
    x_ori = [0., 0., 0., 1.]
    x_angvel = [0., 0., 0]
    bolt_leg_ctrl = BoltImpedanceController(robot)
    centr_controller = BoltCentroidalController(robot.pin_robot, total_mass, mu=1, kp=[0, 0, 100], kd=[0, 0, 10],
                                                kpa=[100, 100, 100], kda=[10., 10, 10], eff_ids=robot.pinocchio_endeff_ids)

    sim = LipmSimpulator(.2)
    dcm_reactive_stepper = DcmReactiveStepper()
    is_left_leg_in_contact = True
    l_min = -0.12
    l_max = 0.12
    w_min = -0.1
    w_max = 0.3
    t_min = 0.1
    t_max = 0.3
    l_p = 0.1235 * 1
    com_height = 0.26487417
    weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
    mid_air_foot_height = .05
    control_period = 0.001
    x_des_local = [0., 0.02, 0., 0., -0.02, 0.]
    past_x = [0., 0.02, 0., 0., -0.02, 0.]
    dcm_reactive_stepper.initialize(is_left_leg_in_contact, l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height,
                                    weight, mid_air_foot_height, control_period, x_des_local[:3], x_des_local[3:])
    #previous_support_foot=[[0.0], [-0.075], [0.]],
    #current_support_foot=[[0.0], [0.075], [0.]]
    # dcm_reactive_stepper.set_end_eff_traj_costs(1e1, 1e1, 1e0, 1e-9)
    v_des = [.3, .0, .0]
    dcm_reactive_stepper.set_desired_com_velocity(v_des)

    x_com = np.zeros((3, 1))
    x_com[:] = [[.0], [.0], [com_height]]
    xd_com = np.zeros((3, 1))
    x_com_cent = x_com.copy()
    xd_com_cent = xd_com.copy()
    omega = np.sqrt(9.8 / com_height)
    cnt_array = [1, 1]
    time = 0
    open_loop = True

    #plot
    plt_time = []
    plt_time_r = []
    plt_time_l = []
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
    plt_tau = []
    plt_euler_angles = []
    plt_dcm = []
    plt_next_step_location = []
    plt_foot_mass_r = []
    plt_foot_mass_l = []
    plt_11_eq_r = []
    plt_11_eq_l = []
    plt_eq_h = []
    plt_eq_qdot = []
    plt_eq_qddot = []
    dcm_force = [0., 0., 0.]

    dcm_reactive_stepper.start()
    # torque = np.loadtxt("torque.txt")

    # f2 = open("q_qdot.txt", "w")
    # f = open("torque.txt", "w")
    # torque = np.loadtxt("torque.txt")
    h_bais = 0
    print(robot.end_effector_names)
    inv_kin = PointContactInverseKinematics(robot.pin_robot.model, robot.end_effector_names)
    for i in range(1600):#1466):#1500):
        print("%%%", i)
        last_qdot = qdot
        q, qdot = robot.get_state()
        # f2.write(" ".join( repr(e.item()) for e in q) + " " + " ".join( repr(e.item()) for e in qdot) + "\n")
        robot.pin_robot.com(q, qdot)
        x_com = robot.pin_robot.com(q, qdot)[0]
        xd_com = robot.pin_robot.com(q, qdot)[1]
        # robot.forward_robot(q, qdot)
        # External force
        # if i > 1000 and i < 1100:
        #     force = np.array([-8, 0, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        # if i > 3000 and i < 3100:
        #     force = np.array([8, 0, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        # if i > 5000 and i < 5100:
        #     force = np.array([0, -4, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)
        # if i > 7000 and i < 7100:
        #     force = np.array([0, 4, 0])
        #     p.applyExternalForce(objectUniqueId=robot.robotId, linkIndex=-1, forceObj=force,
        #                          posObj=[q[0], q[1], q[2]], flags=p.WORLD_FRAME)

        if warmup <= i:

            # if dcm_reactive_stepper.get_is_left_leg_in_contact() == 1: #inv_kin
            #     inv_kin.swing_id = inv_kin.endeff_ids[1]
            #     inv_kin.stance_id = inv_kin.endeff_ids[0]
            # else:
            #     inv_kin.swing_id = inv_kin.endeff_ids[0]
            #     inv_kin.stance_id = inv_kin.endeff_ids[1]
            # inv_kin.forward_robot(q, qdot)
            # # print(inv_kin.foot_mass_matrix(q))
            # if dcm_reactive_stepper.get_is_left_leg_in_contact() == 1:
            #     plt_foot_mass_r.append(inv_kin.foot_mass_matrix(q)[3:6])
            #     plt_11_eq_r.append(inv_kin.eleven_equation_mass_matrix(q))
            #     plt_time_r.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
            # else:
            #     plt_foot_mass_l.append(inv_kin.foot_mass_matrix(q)[:3])
            #     plt_11_eq_l.append(inv_kin.eleven_equation_mass_matrix(q))
            #     plt_time_l.append(dcm_reactive_stepper.get_time_from_last_step_touchdown())
            # plt_eq_h.append(inv_kin.eleven_equation_h(q, qdot))
            # plt_eq_qdot.append(inv_kin.eleven_equation_q_dot(q, qdot))
            # print(inv_kin.eleven_equation_mass_matrix(q).dot(inv_kin.xddot(q, qdot)))
            # plt_eq_qddot.append(inv_kin.eleven_equation_mass_matrix(q).dot(inv_kin.xddot(q, qdot)))
            left = bolt_leg_ctrl.imps[0]
            right = bolt_leg_ctrl.imps[1]
            left_foot_location = np.array(left.pin_robot.data.oMf[left.frame_end_idx].translation).\
                                  reshape(-1)
            right_foot_location = np.array(right.pin_robot.data.oMf[right.frame_end_idx].translation).\
                                    reshape(-1)
            left_foot_vel = np.array(se3.SE3(left.pin_robot.data.oMf[left.frame_end_idx].rotation, np.zeros((3,1))) *\
                            se3.frameJacobian(robot.pin_robot.model, robot.pin_robot.data, q, left.frame_end_idx).dot(qdot)[0:3])[:, 0]
            right_foot_vel = np.array(se3.SE3(right.pin_robot.data.oMf[right.frame_end_idx].rotation, np.zeros((3,1))) *\
                             se3.frameJacobian(robot.pin_robot.model, robot.pin_robot.data, q, right.frame_end_idx).dot(qdot)[0:3])[:, 0]
            #closed_loop()

            contact_array = [0, 0]
            detect_contact()

            print("cur, vel python", right_foot_location, right_foot_vel)
            # left_foot_location[2] = 0.
            # right_foot_location[2] = 0.
            print("contact", dcm_reactive_stepper.get_is_left_leg_in_contact())
            print("1", time, [left_foot_location[0], left_foot_location[1], 0],
                                     [right_foot_location[0], right_foot_location[1], 0], x_com, xd_com, yaw(q), contact_array)
            # if i % 100 == 0:
            #     plot_all_contact_points()
            print("2", time, [left_foot_location[0], left_foot_location[1], 0],
                  [right_foot_location[0], right_foot_location[1], 0], x_com, xd_com, yaw(q), contact_array)

            dcm_reactive_stepper.run(time, [left_foot_location[0], left_foot_location[1], 0],
                                     [right_foot_location[0], right_foot_location[1], 0], x_com, xd_com, yaw(q), contact_array)
            print("run")
            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                print("Lhum-3")
                pos_for_plotter = dcm_reactive_stepper.get_right_foot_position().copy()
                print("Lhum-2")
                vel_for_plotter = dcm_reactive_stepper.get_right_foot_velocity().copy()
                print("Lhum-1")
            else:
                print("Lhum--3")
                pos_for_plotter = dcm_reactive_stepper.get_left_foot_position().copy()
                print("Lhum--2")
                vel_for_plotter = dcm_reactive_stepper.get_left_foot_velocity().copy()
                print("Lhum--1")
            #dcm_force = dcm_reactive_stepper.get_forces().copy() #feed forward
             # if i % 1 == 0 and i > 1430:
            #     print(dcm_force)
            #     plot(dcm_force)
            # print("@", t)
            # print(yaw(q))
            # if dcm_reactive_stepper.time_from_last_step_touchdown == 0:
            #     desired_q = np.array(q.copy())[:, 0]
            # else:
            #     dcm_reactive_stepper.run(time, dcm_reactive_stepper.flying_foot_position, x_com.copy(), xd_com.copy(), 0)  # q[5])

            x_des_local = []
            x_des_local.extend(dcm_reactive_stepper.get_left_foot_position().copy())
            x_des_local.extend(dcm_reactive_stepper.get_right_foot_position().copy())
            print("Lhum2")

            if open_loop:
                x_des_local[2] += 0.0171
                x_des_local[5] += 0.0171

            # print("des")
            # print(x_des_local)
            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                cnt_array = [1, 0]
                x_des_local[5] += left_foot_location[2] - 0.0171
                h_bais = left_foot_location[2] - 0.0171
            else:
                cnt_array = [0, 1]
                x_des_local[2] += right_foot_location[2] - 0.0171
                h_bais = right_foot_location[2] - 0.0171

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
            print("Lhum DCM", x_com, ",", xd_com, ",", omega)
            plt_dcm_local.append(x_com + xd_com / omega)
            if dcm_reactive_stepper.get_time_from_last_step_touchdown() == 0:
                plt_step_time.append(int(i) - warmup)
            time += 0.001

        for j in range(2):
            imp = bolt_leg_ctrl.imps[j]
            # print("LhumImp", j, " ", np.array(imp.pin_robot.data.oMf[imp.frame_root_idx].translation).\
            #                                            reshape(-1))
            x_des_local[3 * j:3 * (j + 1)] -= np.array(imp.pin_robot.data.oMf[imp.frame_root_idx].translation).\
                                                       reshape(-1)
            if j == 0:
                plt_left_eef_real_pos.append(
                    np.array(imp.pin_robot.data.oMf[imp.frame_end_idx].translation).reshape(-1))
            else:
                plt_right_eef_real_pos.append(
                    np.array(imp.pin_robot.data.oMf[imp.frame_end_idx].translation).reshape(-1))

        w_com = centr_controller.compute_com_wrench(q.copy(), qdot.copy(), [0.0, 0.0, com_height + h_bais], [0.0, 0.0, 0.0],
                                                    [0, 0., 0., 1.], [0., 0., 0.])
        w_com[0] = 0.0
        w_com[1] = 0.0
        w_com[2] += total_mass * 9.81

        F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)
        # torque = joint_controller(q, desired_q, qdot, desired_qdot, kp_joint, kd_joint, cnt_array)
        # print("$$$$$$$$$$$$$")
        # print([qdot[0].item(), qdot[1].item(), qdot[2].item()])
        # print(dcm_reactive_stepper.get_left_foot_velocity() - [qdot[0].item(), qdot[1].item(), qdot[2].item()])
        des_vel = np.concatenate((dcm_reactive_stepper.get_left_foot_velocity() -[qdot[0].item(), qdot[1].item(), qdot[2].item()],
                                  dcm_reactive_stepper.get_right_foot_velocity() - [qdot[0].item(), qdot[1].item(), qdot[2].item()]))
        # print(dcm_reactive_stepper.get_left_foot_velocity() + dcm_reactive_stepper.get_right_foot_velocity())
        print("F: ", F)
        dcm_force[0] = -dcm_force[0]
        dcm_force[1] = -dcm_force[1]
        dcm_force[2] = -dcm_force[2]
        print("dcm_force", dcm_force)
        if cnt_array[0] == 1:
            print("!!!!!!!!!!!!!!!!!!!!", is_left_leg_in_contact)
            F[3:] = dcm_force[:3]
        else:
            print("@@@@@@@@@@@@@@@@@@@@", is_left_leg_in_contact)
            F[:3] = dcm_force[:3]
        print("F after adding dcm2", F)
        tau, r = bolt_leg_ctrl.return_joint_torques(q.copy(), qdot.copy(), zero_cnt_gain(kp, cnt_array),
                                                 zero_cnt_gain(kd, cnt_array),
                                                 x_des_local, des_vel, F)
        plt_r.append(r)
        plt_F.append(F)
        plt_x.append(x_des_local)
        plt_tau.append(tau)
        plt_xd_com.append(xd_com.copy())
        plt_qdot_com.append(qdot)
        plt_x_com.append(x_com.copy())
        plt_pos_des_local.append([x_des_local[1], x_des_local[4]])
        #plt_euler_angles.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
        plt_q.append(q[7:].copy())
        plt_qdot.append(qdot[6:].copy())
        # plt_q_com.append(np.array(R.from_quat([np.array(q)[3:7, 0]]).as_euler('xyz', degrees=False))[0, :])
        # plt_desired_q.append(desired_q[7:].copy())

        # for i in range(10):
        # print("tau", tau)
        # print(torque[0])
        # robot.send_joint_command(torque[i])
        robot.send_joint_command(tau)
        # for i in range(6):
        #     tau[i] = round(tau[i].item(), 10)
        # robot.send_joint_command(tau)
        # f.write(str(tau[0].item()) + " " + str(tau[1].item()) + " " + str(tau[2].item()) + " " +
        #         str(tau[3].item()) + " " + str(tau[4].item()) + " " + str(tau[5].item()) + " " + "\n")
        p.stepSimulation()
        # Time.sleep(0.01)
    # print(plt_step_time)
    dcm_reactive_stepper.stop()
    # f2.write(str(plt_foot_mass))
    # p.stopStateLogging()
    # f.close()
    # f2.close()
    print(plt_step_time)

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
    # ax[0][0].plot(plt_time_r, np.array(plt_11_eq_r)[:, 0, 0], 'o', markersize=1, label='0,0')
    # ax[0][0].legend()
    # ax[0][1].plot(plt_time_r, np.array(plt_11_eq_r)[:, 0, 1], 'o', markersize=1, label='0,1')
    # ax[0][1].legend()
    # ax[0][2].plot(plt_time_r, np.array(plt_11_eq_r)[:, 0, 2], 'o', markersize=1, label='0,2')
    # ax[0][2].legend()
    # ax[1][0].plot(plt_time_r, np.array(plt_11_eq_r)[:, 1, 1], 'o', markersize=1, label='1,1')
    # ax[1][0].legend()
    # ax[1][1].plot(plt_time_r, np.array(plt_11_eq_r)[:, 1, 2], 'o', markersize=1, label='1,2')
    # ax[1][1].legend()
    # ax[1][2].plot(plt_time_r, np.array(plt_11_eq_r)[:, 2, 2], 'o', markersize=1, label='2,2')
    # ax[1][2].legend()
    # fig, ax = plt.subplots(2, 3)
    # ax[0][0].plot(plt_time_l, np.array(plt_11_eq_l)[:, 0, 0], 'o', markersize=1, label='0,0')
    # ax[0][0].legend()
    # ax[0][1].plot(plt_time_l, np.array(plt_11_eq_l)[:, 0, 1], 'o', markersize=1, label='0,1')
    # ax[0][1].legend()
    # ax[0][2].plot(plt_time_l, np.array(plt_11_eq_l)[:, 0, 2], 'o', markersize=1, label='0,2')
    # ax[0][2].legend()
    # ax[1][0].plot(plt_time_l, np.array(plt_11_eq_l)[:, 1, 1], 'o', markersize=1, label='1,1')
    # ax[1][0].legend()
    # ax[1][1].plot(plt_time_l, np.array(plt_11_eq_l)[:, 1, 2], 'o', markersize=1, label='1,2')
    # ax[1][1].legend()
    # ax[1][2].plot(plt_time_l, np.array(plt_11_eq_l)[:, 2, 2], 'o', markersize=1, label='2,2')
    # ax[1][2].legend()
    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time, np.array(plt_eq_h)[:, 0], 'o', markersize=1, label='0')
    # ax[0].legend()
    # ax[1].plot(plt_time, np.array(plt_eq_h)[:, 1], 'o', markersize=1, label='1')
    # ax[1].legend()
    # ax[2].plot(plt_time, np.array(plt_eq_h)[:, 2], 'o', markersize=1, label='2')
    # ax[2].legend()
    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time, np.array(plt_eq_qdot)[:, 0], 'o', markersize=1, label='0')
    # ax[0].legend()
    # ax[1].plot(plt_time, np.array(plt_eq_qdot)[:, 1], 'o', markersize=1, label='1')
    # ax[1].legend()
    # ax[2].plot(plt_time, np.array(plt_eq_qdot)[:, 2], 'o', markersize=1, label='2')
    # ax[2].legend()
    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(plt_time, np.array(plt_eq_qddot)[:, 0], markersize=1, label='^xddot')
    # ax[0].plot(plt_time, np.array(plt_eq_qdot)[:, 0] + np.array(plt_eq_h)[:, 0], markersize=1, label='h - qdot')
    # ax[0].legend()
    # ax[1].plot(plt_time, np.array(plt_eq_qddot)[:, 1], markersize=1, label='^xddot')
    # ax[1].plot(plt_time, np.array(plt_eq_qdot)[:, 1] + np.array(plt_eq_h)[:, 1], markersize=1, label='h - qdot')
    # ax[1].legend()
    # ax[2].plot(plt_time, np.array(plt_eq_qddot)[:, 2], markersize=1, label='^xddot')
    # ax[2].plot(plt_time, np.array(plt_eq_qdot)[:, 2] + np.array(plt_eq_h)[:, 2], markersize=1, label='h - qdot')
    # ax[2].legend()

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
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 1], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 1], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 1], label="dcm_local")
    plt.plot(plt_time, np.array(plt_next_step_location)[:, 1], label="next_step_location")
    # plt.plot(plt_time, np.array(plt_dcm)[:, 1], label="dcm")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 1], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 1], label="right_eef_real_pos")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 1], label="current_support_foot")
    # plt.plot(plt_time, plt_pos_des_local[warmup + 1:], label = "pos des_local_eef")
    plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    # plt.figure("q")
    # plt.plot(plt_time, np.array(plt_qdot_com)[:, 6:, 0], label="plt_qdot_com")
    # plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("x")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 0], label="des_left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 0], label="des_right")
    plt.plot(plt_time, np.array(plt_x_com)[warmup:, 0], label="com")
    plt.plot(plt_time, np.array(plt_xd_com)[warmup:, 0], label="xd_com")
    plt.plot(plt_time, np.array(plt_dcm_local)[:, 0], label="dcm_local")
    plt.plot(plt_time, np.array(plt_next_step_location)[:, 0], label="next_step_location")
    # plt.plot(plt_time, np.array(plt_dcm)[:, 0], label="dcm")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 0], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 0], label="right_eef_real_pos")
    # plt.plot(plt_time, np.array(plt_current_support_foot)[:, 0], label="current_support_foot")
    # plt.plot(plt_time, np.array(plt_duration_before_step_landing)[:], label="plt_duration_before_step_landing")
    # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
    plt.legend()
    # for time in plt_step_time:
    #     plt.axvline(time / 1000)

    plt.figure("tau")
    plt.plot(plt_time[:], np.array(plt_tau)[warmup:, :, 0], label="tau")
    plt.legend()

    plt.figure("z")
    plt.plot(plt_time, np.array(plt_left_foot_position)[:, 2], label="left")
    plt.plot(plt_time, np.array(plt_right_foot_position)[:, 2], label="right")
    # plt.plot(plt_time, np.array(plt_x_com)[warmup:, 2], label="com")
    # plt.plot(plt_time, np.array(plt_dcm_local)[:, 2], label="dcm_local")
    plt.plot(plt_time, np.array(plt_left_eef_real_pos)[warmup:, 2], label="left_eef_real_pos")
    plt.plot(plt_time, np.array(plt_right_eef_real_pos)[warmup:, 2], label="right_eef_real_pos")
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

    # plt.figure("F")
    # plt.plot(plt_time, plt_F[warmup:], label="F")
    # plt.plot(plt_time[:], plt_is_left_in_contact[:], label="is_left_in_contact")
    # plt.legend()

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

    plt.show()
