## Contains functions to generate trajectories. Not specific to any robot.
## Author: Avadesh Meduri
## Date: 9/12/2019

import numpy as np


class TrajGenerator:
    def __init__(self, robot):

        self.robot = robot

    def get_frame_location(self, q, dq, frame_idx):
        """
        returns the global location of the frame by computing forward kinematics
        Input:
            q: current joint configuration of the robot
            dq: current joint velocity of the robot
        """

        self.robot.framesForwardKinematics(q)
        return np.reshape(
            np.array(self.robot.model.oMf[frame_idx].translation), (3,)
        )

    def generate_traj(self, start, end, traj_time, t):
        """
        returns desired location at a given time t,
        Generates a straight line start to end point(uniform velocity motion)
        Input:
            start: Start point of the trajectory.
            end: End point of the trajecory.
            traj_time: Total trajectory time
            t: Current time step
        """

        slope = np.divide(np.subtract(end, start), traj_time)

        return np.add(start, slope * t)

    def generate_sin_traj(self, start, end, via_point, traj_time, t):
        """
        returns desired location at a given time t,
        psi = sse.return_dcm_location(q, dq, step_planner.omega)
        Generates sine trajectory from start to end point through a via point
        The trajectory passes through the via point at mid duration
        Input:
            start: Start point of the trajectory.
            end: End point of the trajecory.
            via_point: The point through which trajectory will go through mid trajecory
            traj_time: Total trajectory time
            t: Current time step
        """
        assert np.shape(start) == np.shape(end)
        assert np.shape(start) == np.shape(via_point)

        if t < traj_time / 2.0:
            amplitude = np.subtract(via_point, start)
            omega = (np.pi) / traj_time
            return np.add(start, amplitude * np.sin(omega * t))
        elif t == traj_time / 2.0:
            return via_point
        else:
            amplitude = np.subtract(via_point, end)
            omega = (np.pi) / traj_time
            return np.add(end, amplitude * np.sin(omega * t))

    def generate_foot_traj(self, start, end, via_point, traj_time, t):
        """
        Generates foot trajectory for walking. A uniform velocity is tracked in x,y direction.
        A sine trajectory is generated in the z direction.
        Input:
            Start: Current location of the foot 3d
            end: desired location of the of the foot at the end of the step
            via_point: the hieght in the z axis the leg has to rise
            traj_time: step time
            t: current time
        """

        assert np.shape(start) == (3,)
        assert np.shape(end) == (3,)

        x_des = np.zeros(3)
        x_des[0:2] = self.generate_traj(start[0:2], end[0:2], traj_time, t)
        x_des[2] = self.generate_sin_traj(
            start[2], end[2], via_point[2], traj_time, t
        )

        return x_des
