""" @namespace Demos of solo12 step adjustment
@file
@copyright Copyright (c) 2017-2021,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import time

import numpy as np
import pybullet as p
from robot_properties_solo.config import Solo12Config
from robot_properties_solo.solo12wrapper import Solo12Robot
from reactive_planners_cpp import QuadrupedDcmReactiveStepper
import pinocchio as pin
from scipy.spatial.transform import Rotation

import pinocchio as pin
import mim_control_cpp

import dynamic_graph_head as dgh
from dynamic_graph_head import ThreadHead, Vicon, HoldPDController

import dynamic_graph_manager_cpp_bindings

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

import pinocchio as pin
import mim_control_cpp

class CentroidalController:
    def __init__(self, head, vicon_name, mu, kp, kd, kc, dc, kb, db, qp_weights=[5e5, 5e5, 5e5, 1e6, 1e6, 1e6]):
        self.set_k(kp, kd)
        self.config = Solo12Config
        self.robot = Solo12Config.buildRobotWrapper()
        self.vicon_name = vicon_name

        self.x_com = [0.0, 0.0, 0.20]
        self.xd_com = [0.0, 0.0, 0.0]

        self.x_des = np.array([
             0.2, 0.142, 0.015,  0.2, -0.142,  0.015,
            -0.2, 0.142, 0.015, -0.2, -0.142,  0.015
        ])
        self.xd_des = np.array(4*[0., 0., 0.])

        self.x_ori = [0., 0., 0., 1.]
        self.x_angvel = [0., 0., 0.]
        self.cnt_array = 4*[1,]

        self.w_com = np.zeros(6)

        q_init = np.zeros(19)
        q_init[7] = 1
        self.centrl_pd_ctrl = mim_control_cpp.CentroidalPDController()
        self.centrl_pd_ctrl.initialize(2.5, np.diag(self.robot.mass(q_init)[3:6, 3:6]))

        self.force_qp = mim_control_cpp.CentroidalForceQPController()
        self.force_qp.initialize(4, mu, np.array(qp_weights))

        root_name = 'universe'
        endeff_names = ['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']
        self.imp_ctrls = [mim_control_cpp.ImpedanceController() for eff_name in endeff_names]
        for i, c in enumerate(self.imp_ctrls):
            c.initialize(self.robot.model, root_name, endeff_names[i])

        self.kc = np.array(kc)
        self.dc = np.array(dc)
        self.kb = np.array(kb)
        self.db = np.array(db)

        self.joint_positions = head.get_sensor('joint_positions')
        self.joint_velocities = head.get_sensor('joint_velocities')
        self.slider_positions = head.get_sensor('slider_positions')
        self.imu_gyroscope = head.get_sensor('imu_gyroscope')

    def set_k(self, kp, kd):
        self.kp = 4 * [kp, kp, kp, 0, 0, 0]
        self.kd = 4 * [kd, kd, kd, 0, 0, 0]

    def warmup(self, thread_head):
        thread_head.vicon.bias_position(self.vicon_name)
        self.zero_sliders = self.slider_positions.copy()

    def get_base(self, thread_head):
        base_pos, base_vel = thread_head.vicon.get_state(self.vicon_name)
        base_vel[3:] = self.imu_gyroscope
        return base_pos, base_vel

    def compute_F(self, thread_head):
        ext_cnt_array = [1., 1., 1., 1.]
        self.force_qp.run(self.w_com, self.rel_eff, ext_cnt_array)
        self.F = self.force_qp.get_forces()

    def run(self, thread_head):
        base_pos, base_vel = self.get_base(thread_head)

        self.q = np.hstack([base_pos, self.joint_positions])
        self.dq = np.hstack([base_vel, self.joint_velocities])

        self.centrl_pd_ctrl.run(
            self.kc, self.dc, self.kb, self.db,
            self.q[:3], self.x_com, self.dq[:3], self.xd_com,
            self.q[3:7], self.x_ori, self.dq[3:6], self.x_angvel
        )

        self.w_com = self.centrl_pd_ctrl.get_wrench()
        self.w_com[2] += 9.81 * Solo12Config.mass

        # distrubuting forces to the active end effectors
        pin_robot = self.robot
        pin_robot.framesForwardKinematics(self.q)
        com = self.com = pin_robot.com(self.q)
        self.rel_eff = np.array([
            pin_robot.data.oMf[i].translation - com for i in Solo12Config.end_eff_ids
        ]).reshape(-1)

        self.compute_F(thread_head)

        # passing forces to the impedance controller
        self.tau = np.zeros(18)
        for i, c in enumerate(self.imp_ctrls):
            c.run(self.q, self.dq,
                 np.array(self.kp[6*i:6*(i+1)]),
                 np.array(self.kd[6*i:6*(i+1)]),
                 1.,
                 pin.SE3(np.eye(3), np.array(self.x_des[3*i:3*(i+1)])),
                 pin.Motion(self.xd_des[3*i:3*(i+1)], np.zeros(3)),
                 pin.Force(self.F[3*i:3*(i+1)], np.zeros(3))
             )

            self.tau += c.get_torques()

        head.set_control('ctrl_joint_torques', self.tau[6:])

class ReactiveStepperController(CentroidalController):
    def __init__(self, head):
        super().__init__(
            head, 'solo12/solo12', 0.6,
            50, 0.7,
            [0, 0, 200], [15, 15, 15], [25, 25, 25.], [10., 10., 10.],
            qp_weights=[1e0, 1e0, 1e6, 1e6, 1e6, 1e6]
        )

        is_left_leg_in_contact = True
        l_min = -0.1
        l_max = 0.1
        w_min = -0.10
        w_max = 0.10
        t_min = 0.1
        t_max = 0.3
        l_p = 0.00  # Pelvis width
        com_height = 0.27
        weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
        mid_air_foot_height = 0.1
        control_period = 0.001
        planner_loop = 0.010

        self.x_com[2] = com_height

        # init poses
        base_pose = np.array([0.0, 0.0, 0.238, 0, 0, 0.0, 1.0])

        front_left_foot_position = np.array([0.195, 0.147, 0.015])
        front_right_foot_position = np.array([0.195, -0.147, 0.015])
        hind_left_foot_position = np.array([-0.195, 0.147, 0.015])
        hind_right_foot_position = np.array([-0.195, -0.147, 0.015])

        self.stepper = QuadrupedDcmReactiveStepper()
        self.stepper.initialize(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            self.x_com[2],
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

        self.stepper.set_desired_com_velocity(np.array([0.0, 0.0, 0.0]))
        self.stepper.set_dynamical_end_effector_trajectory()
        self.stepper.set_steptime_nominal(0.20)

    def warmup(self, thread_head):
        super().warmup(thread_head)

        self.control_time = 0.

    def start(self):
        self.stepper.start()

    def stop(self):
        self.stepper.stop()

    def compute_F(self, thread_head):
        config = self.config
        robot = self.robot
        x_com, xd_com = self.robot.com(self.q, self.dq)

        robot.forwardKinematics(self.q, self.dq)
        robot.framesForwardKinematics(self.q)

        # Define left as front left and back right leg
        front_left_foot_position = robot.data.oMf[config.end_eff_ids[0]].translation
        front_right_foot_position = robot.data.oMf[config.end_eff_ids[1]].translation
        hind_left_foot_position = robot.data.oMf[config.end_eff_ids[2]].translation
        hind_right_foot_position = robot.data.oMf[config.end_eff_ids[3]].translation
        front_left_foot_velocity = pin.getFrameVelocity(
            robot.model, robot.data, config.end_eff_ids[0], pin.LOCAL_WORLD_ALIGNED).linear
        front_right_foot_velocity = pin.getFrameVelocity(
            robot.model, robot.data, config.end_eff_ids[1], pin.LOCAL_WORLD_ALIGNED).linear
        hind_left_foot_velocity = pin.getFrameVelocity(
            robot.model, robot.data, config.end_eff_ids[2], pin.LOCAL_WORLD_ALIGNED).linear
        hind_right_foot_velocity = pin.getFrameVelocity(
            robot.model, robot.data, config.end_eff_ids[3], pin.LOCAL_WORLD_ALIGNED).linear

        open_loop = True
        self.stepper.run(
            self.control_time,
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
            yaw(self.q),
            not open_loop,
        )

        cnt_array = self.stepper.get_contact_array()
        self.force_qp.run(self.w_com, self.rel_eff, cnt_array)
        self.F = self.force_qp.get_forces()

        # dcm_forces = self.stepper.get_forces()

        # if cnt_array[0] == 1:
        #     self.F[3:6] = -dcm_forces[6:9]
        #     self.F[6:9] = -dcm_forces[6:9]
        # else:
        #     self.F[0:3] = -dcm_forces[:3]
        #     self.F[9:12] = -dcm_forces[:3]

        self.x_des = np.hstack([
            self.stepper.get_front_left_foot_position(),
            self.stepper.get_front_right_foot_position(),
            self.stepper.get_hind_left_foot_position(),
            self.stepper.get_hind_right_foot_position()
        ])

        self.dx_des = np.hstack([
            self.stepper.get_front_left_foot_velocity(),
            self.stepper.get_front_right_foot_velocity(),
            self.stepper.get_hind_left_foot_velocity(),
            self.stepper.get_hind_right_foot_velocity(),
        ])

        self.control_time += 0.001

###
# Create the dgm communication and instantiate the controllers.
head = dynamic_graph_manager_cpp_bindings.DGMHead(Solo12Config.dgm_yaml_path)

# Create the controllers.
hold_pd_controller = HoldPDController(head, 3., 0.05, with_sliders=True)

centroidal_controller = CentroidalController(head, 'solo12/solo12', 0.2, 50., 0.7,
    [100., 100., 100.], [15., 15., 15.], [25., 25., 25.], [22.5, 22.5, 22.5]
)

ctrl = ReactiveStepperController(head)

thread_head = ThreadHead(
    0.001,
    hold_pd_controller,
    head,
    [
        ('vicon', Vicon('172.24.117.119:801', ['solo12/solo12']))
    ]
)

# Start the parallel processing.
thread_head.start()

###
# List of helper go functions.
def go_hold():
    thread_head.switch_controllers(thread_head.safety_controllers)

def go_hold_zero():
    go_hold()
    hold_pd_controller.go_zero()

def go_cent():
    thread_head.switch_controllers(centroidal_controller)

def go_stepper():
    thread_head.switch_controllers(ctrl)

# Check if the IMU is working.
imu_gyro = head.get_sensor('imu_gyroscope')
if np.all(imu_gyro == np.zeros(3)):
    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    print('!!!                                                       !!!')
    print('!!!                 IMU IS NOT WORKING                    !!!')
    print('!!!                                                       !!!')
    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

    print('imu_gyro:', imu_gyro)

go_hold_zero()
