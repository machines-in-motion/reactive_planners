/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the EndEffectorTrajectory3D class
 */

#include "reactive_planners/polynomial_end_effector_trajectory.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace reactive_planners;

void bind_end_effector_trajectory_3d(pybind11::module &module)
{
    pybind11::class_<PolynomialEndEffectorTrajectory>(module,
                                                      "EndEffectorTrajectory3D")
        .def(pybind11::init<>())

        // public methods
        .def("compute", &PolynomialEndEffectorTrajectory::compute)
        .def("get_next_state", &PolynomialEndEffectorTrajectory::get_next_state)
        .def("get_mid_air_height",
             &PolynomialEndEffectorTrajectory::get_mid_air_height)
        .def("print_solver", &PolynomialEndEffectorTrajectory::print_solver)
        .def("set_mid_air_height",
             &PolynomialEndEffectorTrajectory::set_mid_air_height)
        .def("set_costs", &PolynomialEndEffectorTrajectory::set_costs)

        // String representation of the sovler.
        .def("__repr__", [](const PolynomialEndEffectorTrajectory &planner) {
            return planner.to_string();
        });
}