import time

import crocoddyl
import pinocchio
import numpy as np
from crocoddyl.utils.biped import SimpleBipedGaitProblem
# import pybullet as p
# from bullet_utils.env import BulletEnvWithGround
# from robot_properties_bolt.bolt_wrapper import BoltRobot
from robot_properties_bolt.config import BoltConfig

def update_trajectory():
    CoM_trajectory = [[1.0 * i / (lengh_of_motion * frequency) , 0, 0.35487417] for i in range(lengh_of_motion * frequency)]
    # trajectory_end_eff_left =
    # trajectory_end_eff_right =
    # trajectory_contact =
    return np.array(CoM_trajectory)

if __name__ == "__main__":
    # Creating the lower-body part of Talos
    # talos_legs = example_robot_data.load('talos_legs')
    # print(talos_legs)


    # Create a robot instance. This initializes the simulator as well.
    # env = BulletEnvWithGround()
    # robot = env.add_robot(BoltRobot())
    robot = BoltConfig.buildRobotWrapper()
    viz = pinocchio.visualize.MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    lengh_of_motion = 2
    frequency = 1000

    CoM_trajectory = update_trajectory()

    state = crocoddyl.StateMultibody(robot.model)

    # setting up terminal cost model
    xRegCost = crocoddyl.CostModelState(state)
    uRegCost = crocoddyl.CostModelControl(state)
    comTrack = crocoddyl.CostModelCoMPosition(state, CoM_trajectory[-1], state.nv)

    # Create cost model per each action model
    runningCostModel = crocoddyl.CostModelSum(state)
    terminalCostModel = crocoddyl.CostModelSum(state)

    runningCostModel.addCost("stateReg", xRegCost, 1e-4)
    terminalCostModel.addCost("ctrlReg", uRegCost, 1e-7)

    # Create the actuation model
    actuationModel = crocoddyl.ActuationModelFloatingBase(state)

    # Create the action model
    runningModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, runningCostModel), 1. / frequency)
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, terminalCostModel))
    #runningModel.differential.armature = 0.2 * np.ones(state.nv)
    #terminalModel.differential.armature = 0.2 * np.ones(state.nv)

    # Create the problem
    q0 = np.array(BoltConfig.initial_configuration)
    x0 = np.concatenate([q0, pinocchio.utils.zero(robot.model.nv)])
    problem = crocoddyl.ShootingProblem(x0, [runningModel] * lengh_of_motion, terminalModel)


    # Creating the DDP solver for this OC problem, defining a logger
    ddp = crocoddyl.SolverDDP(problem)
    ddp.setCallbacks([crocoddyl.CallbackVerbose()])

    # Solving it with the DDP algorithm
    ddp.solve()

    # Visualizing the solution in gepetto-viewer

    # print("Start")
    viz.display(q0)
    time.sleep(10)
    print("start")

    for i in range(len(ddp.xs)):
        time.sleep(0.1)
        print(ddp.xs[i][:robot.model.nq])
        viz.display(ddp.xs[i][:robot.model.nq])

    print("END")

    robot_data = robot.model.createData()
    xT = ddp.xs[-1]
    pinocchio.forwardKinematics(robot.model, robot_data, xT[:state.nq])
    pinocchio.updateFramePlacements(robot.model, robot_data)
    print('Finally reached = ', robot_data.oMf[robot.model.getFrameId("gripper_left_joint")].translation.T)


