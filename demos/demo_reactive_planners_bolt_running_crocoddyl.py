import crocoddyl
import pinocchio
import numpy as np
import example_robot_data
from crocoddyl.utils.biped import SimpleBipedGaitProblem
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_bolt.bolt_wrapper import BoltRobot


if __name__ == "__main__":
    # Creating the lower-body part of Talos
    talos_legs = example_robot_data.load('talos_legs')
    print(talos_legs)


    # # Create a robot instance. This initializes the simulator as well.
    # env = BulletEnvWithGround()
    # robot = env.add_robot(BoltRobot())
    # print(robot)


    # Setting up the 3d walking problem
    rightFoot = 'right_sole_link'
    leftFoot = 'left_sole_link'
    gait = SimpleBipedGaitProblem(talos_legs.model, rightFoot, leftFoot)


    # Create the initial state
    q0 = talos_legs.q0.copy()
    v0 = pinocchio.utils.zero(talos_legs.model.nv)
    x0 = np.concatenate([q0, v0])


    # Creating the walking problem
    stepLength = 0.6 # meters
    stepHeight = 0.1 # meters
    timeStep = 0.0375 # seconds
    stepKnots = 20
    supportKnots = 10
    print(x0)
    problem = gait.createWalkingProblem(x0, stepLength, stepHeight, timeStep, stepKnots, supportKnots)


    # Solving the 3d walking problem using Feasibility-prone DDP
    ddp = crocoddyl.SolverFDDP(problem)


    # Using the meshcat displayer, you could enable gepetto viewer for nicer view
    # display = crocoddyl.GepettoDisplay(talos_legs, 4, 4, frameNames=[rightFoot, leftFoot])
    display = crocoddyl.MeshcatDisplay(talos_legs, 4, 4, False)
    ddp.setCallbacks([crocoddyl.CallbackLogger(),
                      crocoddyl.CallbackVerbose(),
                      crocoddyl.CallbackDisplay(display)])


    # Emdebbed meshcat in this cell
    display.robot.viewer.jupyter_cell()

    # Solve the optimal control problem
    ddp.th_stop = 1e-9
    init_xs = [talos_legs.model.defaultState] * (problem.T + 1)
    init_us = []
    maxiter = 1000
    regInit = 0.1
    ddp.solve(init_xs, init_us, maxiter, False, regInit)


    log = ddp.getCallbacks()[0]
    crocoddyl.plotOCSolution(log.xs, log.us)
    crocoddyl.plotConvergence(log.costs, log.u_regs, log.x_regs, log.grads, log.stops, log.steps)

    # Visualization of the DDP solution in meshcat
    display.rate = -1
    display.freq = 1
    display.displayFromSolver(ddp)