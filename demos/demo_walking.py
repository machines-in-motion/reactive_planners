from py_reactive_planners.dcm_reactive_stepper import DcmReactiveStepper
from py_reactive_planners.lipm_simulator import LipmSimpulator
import numpy as np
from RAI.data_collector import DataCollector

if __name__ == "__main__":
    dc = DataCollector()
    sim = LipmSimpulator(.2)

    dcm_reactive_stepper = DcmReactiveStepper(is_left_leg_in_contact=True, l_min=-0.5, l_max=0.5, w_min=-0.5, w_max=0.5, t_min=0.1,
                             t_max=0.2, l_p=0.1235 * 2, com_height=0.26487417,
                             weight=[1, 1, 5, 100, 100, 100, 100, 100, 100], mid_air_foot_height=.05,
                            control_period= 0.001)
    u_current_step=np.array([.0, 0.1235, .0])
    v_des=np.array([.0, .0, .0])
    x_com=np.array([.0, .0, .2])
    xd_com=np.array([.0, .0, .0])
    time = 0

    for i in range(1000):
        time += 0.001
        dcm_reactive_stepper.run(time, dcm_reactive_stepper.flying_foot_position, x_com, xd_com, 0)
        x_com, xd_com , _ = sim.step(dcm_reactive_stepper.time_from_last_step_touchdown,
                                     dcm_reactive_stepper.current_support_foot, x_com, xd_com)
        dc.add_variable(time, "time", "s")
        dc.add_vector_3d(x_com, "x_com", "m")
        dc.add_vector_3d(xd_com, "xd_com", "m/s")
        dc.add_vector_3d(dcm_reactive_stepper.right_foot_position, "right_foot_position", "m")
        dc.add_vector_3d(dcm_reactive_stepper.right_foot_velocity, "right_foot_velocity", "m/s")
        dc.add_vector_3d(dcm_reactive_stepper.right_foot_acceleration, "right_foot_acceleration", "m/s^2")
        dc.add_vector_3d(dcm_reactive_stepper.left_foot_position, "left_foot_position", "m")
        dc.add_vector_3d(dcm_reactive_stepper.left_foot_velocity, "left_foot_velocity", "m/s")
        dc.add_vector_3d(dcm_reactive_stepper.left_foot_acceleration, "left_foot_acceleration", "m/s^2")
    dc.dump('/tmp/demo_walking.npy')
