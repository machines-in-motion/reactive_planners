""" @namespace Demos of Bolt rerun trajectory
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
from bullet_utils.env import BulletEnvWithGround
import time
from decimal import Decimal

if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround()
    robot = env.add_robot(BoltRobot())
    p.resetDebugVisualizerCamera(2., 50, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(0.001)
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
    q[2] += 0.01
    print(q)
    qdot = np.matrix(BoltConfig.initial_velocity).T
    robot.reset_state(q, qdot)
    env.print_physics_engine_params()

    for ji in range(8):
        print(p.getDynamicsInfo(robot.robotId, ji))
    tau = np.loadtxt("tau.txt")

    for t in tau:
        # print(np.array(t))
        print([Decimal(i) for i in t])
        robot.send_joint_command(np.around(t, decimals=6))
        p.stepSimulation()
        time.sleep(0.001)
