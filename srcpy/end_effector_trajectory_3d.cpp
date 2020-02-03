/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the EndEffectorTrajectory3D class
 */

#include "reactive_planners/end_effector_trajectory_3d.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace reactive_planners;

void bind_end_effector_trajectory_3d(pybind11::module &module) {
  pybind11::class_<EndEffectorTrajectory3D>(module, "EndEffectorTrajectory3D")
      .def(pybind11::init<>())

      // public methods
      .def("compute", &EndEffectorTrajectory3D::compute)
      .def("get_next_state", &EndEffectorTrajectory3D::get_next_state)
      .def("get_mid_air_height", &EndEffectorTrajectory3D::get_mid_air_height)
      .def("print_solver", &EndEffectorTrajectory3D::print_solver)
      .def("set_mid_air_height", &EndEffectorTrajectory3D::set_mid_air_height)

      // String representation of the sovler.
      .def("__repr__", [](const EndEffectorTrajectory3D &planner) {
        return planner.to_string();
      });
}