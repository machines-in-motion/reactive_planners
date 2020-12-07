/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Demo of the stepper_head + dcm_vrp_planner +
 * end_effector_trajectory_3d
 */

#include "reactive_planners/stepper_head.hpp"
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

bool run(std::ofstream &myfile)
{
    /*
     * Parameters.
     */
    double time = 0.0;
    double dt = 0.001;
    double duration_before_step_landing = 0.1;
    Eigen::Vector3d next_support_foot = Eigen::Vector3d::Zero();

    /*
     * Initialization
     */
    reactive_planners::StepperHead stepper_head;

    // Compute the trajectory and log.

    // clang-format off
    myfile << time << "\t"
           << stepper_head.get_time_from_last_step_touchdown() << "\t"
           << stepper_head.get_is_left_leg_in_contact() << "\t"
           << stepper_head.get_previous_support_location().transpose() << "\t"
           << stepper_head.get_current_support_location().transpose() << "\t"
           << std::endl;
    // clang-format on

    for (int i = 0; i < 10000; ++i)
    {
        stepper_head.run(
            duration_before_step_landing, 0., next_support_foot, time);

        next_support_foot << 0.5*time, time, 2*time;
        time += dt;

        // clang-format off
        myfile << time << "\t"
           << stepper_head.get_time_from_last_step_touchdown() << "\t"
           << stepper_head.get_is_left_leg_in_contact() << "\t"
           << stepper_head.get_previous_support_location().transpose() << "\t"
           << stepper_head.get_current_support_location().transpose() << "\t"
           << std::endl;
        // clang-format on
    }
    return true;
}

int main(int, char **)
{
    std::ofstream myfile;
    myfile.open("/tmp/demo_stepper_head.dat");

    run(myfile);

    myfile.close();
    return 0;
}