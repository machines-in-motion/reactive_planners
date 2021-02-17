/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the DcmVrpPlanner class
 */

#include <pybind11/pybind11.h>

void bind_stepper_head(pybind11::module &module);
void bind_dcm_vrp_planner(pybind11::module &module);
void bind_end_effector_trajectory_3d(pybind11::module &module);
void bind_dcm_reactive_stepper(pybind11::module &module);
void bind_quadruped_dcm_reactive_stepper(pybind11::module &module);

PYBIND11_MODULE(reactive_planners_cpp, m)
{
    m.doc() = R"pbdoc(
        reactive_planners python bindings
        ---------------------------------
        .. currentmodule:: reactive_planners
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

    bind_stepper_head(m);
    bind_dcm_vrp_planner(m);
    bind_end_effector_trajectory_3d(m);
    bind_dcm_reactive_stepper(m);
    bind_quadruped_dcm_reactive_stepper(m);
}