/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the DcmVrpPlanner class
 */

#include <pybind11/pybind11.h>

void bind_dcm_vrp_planner(pybind11::module &module);
void bind_end_effector_trajectory_3d(pybind11::module &module);

PYBIND11_MODULE(reactive_planners, m) {
  m.doc() = R"pbdoc(
        reactive_planners python bindings
        ---------------------------------
        .. currentmodule:: reactive_planners
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

  bind_dcm_vrp_planner(m);
  bind_end_effector_trajectory_3d(m);
}