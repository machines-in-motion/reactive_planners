/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the EndEffectorTrajectory3D class
 */

#include "reactive_planners/dynamically_consistent_end_effector_trajectory.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace reactive_planners;

void bind_dynamically_consistent_end_effector_trajectory_3d(
    pybind11::module &module)
{
    pybind11::class_<DynamicallyConsistentEndEffectorTrajectory>(
        module, "DynamicallyConsistentEndEffectorTrajectory")
        .def(pybind11::init<>())

        // public methods
        .def("compute", &DynamicallyConsistentEndEffectorTrajectory::compute)
        .def("update_robot_status",
             &DynamicallyConsistentEndEffectorTrajectory::update_robot_status)
        .def("get_forces",
             &DynamicallyConsistentEndEffectorTrajectory::get_forces)
        .def("print_solver",
             &DynamicallyConsistentEndEffectorTrajectory::print_solver)
        .def("get_mid_air_height",
             &DynamicallyConsistentEndEffectorTrajectory::get_mid_air_height)
        .def("set_mid_air_height",
             &DynamicallyConsistentEndEffectorTrajectory::set_mid_air_height)
        .def("set_planner_loop",
             &DynamicallyConsistentEndEffectorTrajectory::set_planner_loop)
        .def("cost", &DynamicallyConsistentEndEffectorTrajectory::cost)
        .def("get_slack_variables",
             &DynamicallyConsistentEndEffectorTrajectory::get_slack_variables)
        .def("calculate_t_min",
             &DynamicallyConsistentEndEffectorTrajectory::calculate_t_min)

        // String representation of the sovler.
        .def("__repr__",
             [](const DynamicallyConsistentEndEffectorTrajectory &planner)
             { return planner.to_string(); });
}