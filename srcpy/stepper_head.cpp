/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the StepperHead class
 */

#include "reactive_planners/stepper_head.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace reactive_planners;

void bind_stepper_head(pybind11::module &module)
{
    pybind11::class_<StepperHead>(module, "StepperHead")
        .def(pybind11::init<>())

        // public methods
        .def("run",
             (void (StepperHead::*)(const double &,
                                    const Eigen::Vector3d &,
                                    const double &,
                                    bool)) &
                 StepperHead::run)

        .def("run",
             (void (StepperHead::*)(
                 const double &, const Eigen::Vector3d &, const double &)) &
                 StepperHead::run)

        .def("get_time_from_last_step_touchdown",
             &StepperHead::get_time_from_last_step_touchdown)
        .def("get_is_left_leg_in_contact",
             &StepperHead::get_is_left_leg_in_contact)
        .def("get_previous_support_location",
             &StepperHead::get_previous_support_location)
        .def("get_current_support_location",
             &StepperHead::get_current_support_location)
        .def("set_support_feet_pos", &StepperHead::set_support_feet_pos);
}