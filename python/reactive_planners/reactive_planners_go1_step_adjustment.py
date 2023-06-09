""" @namespace Demos of Go1 step adjustment
@file
@copyright Copyright (c) 2017-2021,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
import numpy as np
from robot_properties_go1.config import Go1Config
from robot_properties_go1.go1wrapper import Go1RobotWithoutPybullet
from mim_control.robot_centroidal_controller import RobotCentroidalController
from mim_control.robot_impedance_controller import RobotImpedanceController
from reactive_planners_cpp import QuadrupedDcmReactiveStepper
import pinocchio as pin
from scipy.spatial.transform import Rotation
from colorama import Fore
from pinocchio.utils import zero
import time
import tracemalloc
import memory_profiler

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


class go1_reactive_planners():
    def __init__(self,
                 initial_configuration=Go1Config.initial_configuration,
                 ):
        """" Notes:
        1) Make sure you initialize the robot from the ground. All feet 
        should be in contact and in nominal positions from the base.
        2) The desired velocity is in the world frame not the base frame.
        3) You must build your code when you change any Python file 
        except the one you run directly. (Because We use pybind to be able
        to run the code fast.)
        """""

        # np.set_printoptions(suppress=True, precision=2)
        # pin.switchToNumpyArray()

        # Create a robot instance.
        self.robot = Go1RobotWithoutPybullet()

        q = np.array(initial_configuration)
        self.robot_config = Go1Config()
        config_file = self.robot_config.ctrl_path
        self.go1_leg_ctrl = RobotImpedanceController(self.robot, config_file)
        self.robot.pin_robot.framesForwardKinematics(q)

        ########## Parameters can  tune
        self.kp = np.array(4 * [100000, 100000, 10000])
        self.kd = 12 * [15.0]
        self.centr_controller = RobotCentroidalController(
            self.robot_config,
            mu=0.6,
            kc=[0, 0, 200],
            dc=[10, 10, 10],
            kb=[125, 25, 25.],
            db=[22.5, 22.5, 22.5],
            qp_penalty_lin=[1e0, 1e0, 1e6],
            qp_penalty_ang=[1e6, 1e6, 1e6],
        )
        l_min = -0.4
        l_max = 0.4
        w_min = -0.08
        w_max = 0.4
        t_min = 0.1
        t_max = 2.0
        self.com_height = 0.35
        weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
        mid_air_foot_height = 0.05

        #########################

        is_left_leg_in_contact = True
        l_p = 0.00  # Pelvis width
        self.dt = 0.002
        control_period = self.dt
        planner_loop = 0.010
        v_des = np.array([0.0, -0.0, 0.0])
        base_pose = q[:7]

        front_left_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[0].frame_end_idx].translation
        front_right_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[1].frame_end_idx].translation
        hind_left_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[2].frame_end_idx].translation
        hind_right_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[3].frame_end_idx].translation

        self.quadruped_dcm_reactive_stepper = QuadrupedDcmReactiveStepper()
        self.quadruped_dcm_reactive_stepper.initialize(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            self.com_height,
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

        self.quadruped_dcm_reactive_stepper.set_steptime_nominal(0.15)
        self.quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des)
        self.quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory()

        # Do not change
        # x_com = self.robot.pin_robot.com(q, qdot)[0]#Lhum update
        x_com = [[0.0], [0.0], [self.com_height]]
        self.com_des = np.array([x_com[0][0], x_com[1][0]])
        self.yaw_des = yaw(q)
        self.cnt_array = [1, 1]
        self.control_time = 0
        self.open_loop = True
        self.offset = -0.02  # foot radius

    def start(self):
        """ Warning
        1) Don't use it immediately after initialization. run at least one step before calling start.
        """
        self.quadruped_dcm_reactive_stepper.start()


    def stop(self):
        """ Warning
        1) Don't use it immediately after initialization. run at least one step before calling start.
        """
        self.quadruped_dcm_reactive_stepper.stop()

    def reset(self,
              initial_configuration=Go1Config.initial_configuration,
              ):
        # Create a robot instance.
        q = np.array(initial_configuration)
        q[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., yaw(q))).coeffs()
        self.robot.pin_robot.framesForwardKinematics(q)

        ########## Parameters you can tune
        l_min = -0.4
        l_max = 0.4
        w_min = -0.08
        w_max = 0.4
        t_min = 0.1
        t_max = 2.0
        self.com_height = 0.35
        weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
        mid_air_foot_height = 0.05

        #########################
        is_left_leg_in_contact = True
        l_p = 0.00  # Pelvis width
        control_period = self.dt
        planner_loop = 0.010
        v_des = np.array([0.0, -0.0, 0.0])
        base_pose = q[:7]


        front_left_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[0].frame_end_idx].translation
        front_right_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[1].frame_end_idx].translation
        hind_left_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[2].frame_end_idx].translation
        hind_right_foot_position = self.robot.pin_robot.data.oMf[
            self.go1_leg_ctrl.imp_ctrl_array[3].frame_end_idx].translation

        self.quadruped_dcm_reactive_stepper.initialize(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            self.com_height,
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

        self.quadruped_dcm_reactive_stepper.set_steptime_nominal(0.15)
        self.quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des)
        self.quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory()

        # Do not change
        # x_com = self.robot.pin_robot.com(q, qdot)[0]#Lhum update
        x_com = [[0.0], [0.0], [self.com_height]]
        self.com_des = np.array([x_com[0][0], x_com[1][0]])
        self.yaw_des = yaw(q)
        self.cnt_array = [1, 1]
        self.control_time = 0

    def step(self, q, qdot,
             v_des=np.array([0.0, -0.0, 0.0]),
             y_des=0.0,
             ):
        """ Warning
        1) Calling this function means one timestep moving forward. You cannot undo it afterward.
        """
        if type(q) is np.array or type(q) is list:
            temp = zero(len(q))
            temp[:] = q
            q = temp.copy()

        if type(qdot) is np.array or type(qdot) is list:
            temp = zero(len(qdot))
            temp[:] = qdot
            qdot = temp.copy()

        # self.robot.reset_state(q, qdot)
        self.robot.pin_robot.com(q, qdot)
        self.robot.update_pinocchio(q, qdot)
        x_com = self.robot.pin_robot.com(q, qdot)[0]
        xd_com = self.robot.pin_robot.com(q, qdot)[1]
        self.quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des)

        self.com_des += v_des[:2] * self.dt
        self.yaw_des += y_des * self.dt

        FL = self.go1_leg_ctrl.imp_ctrl_array[0]
        FR = self.go1_leg_ctrl.imp_ctrl_array[1]
        HL = self.go1_leg_ctrl.imp_ctrl_array[2]
        HR = self.go1_leg_ctrl.imp_ctrl_array[3]
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

        # print(Fore.WHITE + "class control_time", self.control_time)
        # print(Fore.WHITE + "class front_left_foot_position", front_left_foot_position)
        # print(Fore.WHITE + "class front_right_foot_position", front_right_foot_position)
        # print(Fore.WHITE + "class hind_left_foot_position", hind_left_foot_position)
        # print(Fore.WHITE + "class hind_right_foot_position", hind_right_foot_position)
        # print(Fore.WHITE + "class front_left_foot_velocity", front_left_foot_velocity)
        # print(Fore.WHITE + "class front_right_foot_velocity", front_right_foot_velocity)
        # print(Fore.WHITE + "class hind_left_foot_velocity", hind_left_foot_velocity)
        # print(Fore.WHITE + "class hind_right_foot_velocity", hind_right_foot_velocity)
        # print(Fore.WHITE + "class x_com", x_com)
        # print(Fore.WHITE + "class xd_com", xd_com)
        # print(Fore.WHITE + "class yaw(q)", yaw(q))
        self.quadruped_dcm_reactive_stepper.run(
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
            yaw(q),
            not self.open_loop,
        )

        x_des_local = []
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_front_left_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_front_right_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_hind_left_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_hind_right_foot_position())

        x_des_local[2] -= self.offset
        x_des_local[5] -= self.offset
        x_des_local[8] -= self.offset
        x_des_local[11] -= self.offset
        # print(Fore.RED + "class x_des_local after offset", x_des_local)

        self.cnt_array = self.quadruped_dcm_reactive_stepper.get_contact_array()

        for j in range(4):
            imp = self.go1_leg_ctrl.imp_ctrl_array[j]
            x_des_local[3 * j : 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation

        # print(Fore.WHITE + "class q", q)
        # print("class qdot", qdot)
        # print("class [self.com_des[0], self.com_des[1], self.com_height]", [self.com_des[0], self.com_des[1], self.com_height])
        # print("class v_des", v_des)
        # print("class yaw_des", self.yaw_des)
        # print("class pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., self.yaw_des)).coeffs()", pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., self.yaw_des)).coeffs())
        # print("class [0.0, 0.0, y_des]", [0.0, 0.0, y_des])
        w_com = self.centr_controller.compute_com_wrench(
            q.copy(),
            qdot.copy(),
            [self.com_des[0], self.com_des[1], self.com_height],
            v_des,
            pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., self.yaw_des)).coeffs(),
            [0.0, 0.0, y_des],
        )
        # print("class w_com", w_com)

        F = self.centr_controller.compute_force_qp(q, qdot, self.cnt_array, w_com)

        des_vel = np.concatenate(
            (
                self.quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity(),
            )
        )

        dcm_force = np.array([0.0, 0.0, 0.0])
        if self.cnt_array[1] == 0:
            F[3:6] = -dcm_force[:3]
        elif self.cnt_array[2] == 0:
            F[6:9] = -dcm_force[:3]
        elif self.cnt_array[0] == 0:
            F[0:3] = -dcm_force[:3]
        elif self.cnt_array[3] == 0:
            F[9:12] = -dcm_force[:3]

        # print("class cnt_array", self.cnt_array)
        # print("class q", q)
        # print("class qdot", qdot)
        # print("class zero_cnt_gain(self.kp, self.cnt_array)", zero_cnt_gain(self.kp, self.cnt_array))
        # print("class zero_cnt_gain(self.kd, self.cnt_array)", zero_cnt_gain(self.kd, self.cnt_array))
        # print(Fore.RED + "class x_des_local", x_des_local)
        # print(Fore.WHITE + "class des_vel", des_vel)
        # print("class F", F)
        tau = self.go1_leg_ctrl.return_joint_torques(
            q.copy(),
            qdot.copy(),
            zero_cnt_gain(self.kp, self.cnt_array),
            zero_cnt_gain(self.kd, self.cnt_array),
            x_des_local,
            des_vel,
            F,
        )
        self.control_time += self.dt

        # self.robot.send_joint_command(tau)

        return tau

if __name__ == "__main__":
    tracemalloc.start()
    num_simulation = 1
    length_simulation = 1000
    stepper = []
    start_time = time.time()

    for i in range(num_simulation):
        stepper.append(go1_reactive_planners())
        stepper[-1].start()

    initialization_time = time.time()
    print("Initialization time", initialization_time - start_time)

    for i in range(length_simulation):
        for j in range(num_simulation):
            stepper[j].step(Go1Config.initial_configuration, Go1Config.initial_velocity, 0)

    end_time = time.time()
    print("Simulating steps time", end_time - initialization_time)
    print(tracemalloc.get_tracemalloc_memory(), "bytes")
    tracemalloc.stop()
    import os, psutil
    print(psutil.Process(os.getpid()).memory_info().rss)