/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the StepperHead class
 */

#include "reactive_planners/dcm_reactive_stepper.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <boost/python.hpp>

using namespace reactive_planners;

void bind_dcm_reactive_stepper(pybind11::module &module)
{
    pybind11::class_<DcmReactiveStepper>(module, "DcmReactiveStepper")
        .def(pybind11::init<>())

        // Public methods.
        .def("initialize", &DcmReactiveStepper::initialize)
        .def("run",
             (bool (DcmReactiveStepper::*)(
                 double,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const double &,
                 const bool &)) &
                 DcmReactiveStepper::run)
        .def("run",
             [](DcmReactiveStepper &obj,
                double time,
                const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
                const Eigen::Ref<const Eigen::Vector3d> &right_foot_position,
                const Eigen::Ref<const Eigen::Vector3d> &left_foot_vel,
                const Eigen::Ref<const Eigen::Vector3d> &right_foot_vel,
                const Eigen::Ref<const Eigen::Vector3d> &com_position,
                const Eigen::Ref<const Eigen::Vector3d> &com_velocity,
                const pybind11::object &py_world_M_base,
                const bool &is_closed_loop) {
                 const pinocchio::SE3 &world_M_base =
                     boost::python::extract<const pinocchio::SE3 &>(
                         py_world_M_base.ptr());
                 bool ret = obj.run(time,
                                    left_foot_position,
                                    right_foot_position,
                                    left_foot_vel,
                                    right_foot_vel,
                                    com_position,
                                    com_velocity,
                                    world_M_base,
                                    is_closed_loop);
                 return ret;
             })
        .def("start", &DcmReactiveStepper::start)
        .def("stop", &DcmReactiveStepper::stop)
        // Setters.
        .def("set_desired_com_velocity",
             &DcmReactiveStepper::set_desired_com_velocity)
        .def("set_right_foot_position",
             &DcmReactiveStepper::set_right_foot_position)
        .def("set_right_foot_velocity",
             &DcmReactiveStepper::set_right_foot_velocity)
        .def("set_left_foot_position",
             &DcmReactiveStepper::set_left_foot_position)
        .def("set_left_foot_velocity",
             &DcmReactiveStepper::set_left_foot_velocity)
        .def("dcm_vrp_planner_initialization",
             &DcmReactiveStepper::dcm_vrp_planner_initialization)

        // Getters.
        .def("get_right_foot_position",
             &DcmReactiveStepper::get_right_foot_position)
        .def("get_right_foot_velocity",
             &DcmReactiveStepper::get_right_foot_velocity)
        .def("get_right_foot_acceleration",
             &DcmReactiveStepper::get_right_foot_acceleration)
        .def("get_left_foot_position",
             &DcmReactiveStepper::get_left_foot_position)
        .def("get_left_foot_velocity",
             &DcmReactiveStepper::get_left_foot_velocity)
        .def("get_left_foot_acceleration",
             &DcmReactiveStepper::get_left_foot_acceleration)
        .def("get_previous_support_foot_position",
             &DcmReactiveStepper::get_previous_support_foot_position)
        .def("get_current_support_foot_position",
             &DcmReactiveStepper::get_current_support_foot_position)
        .def("get_next_support_foot_position",
             &DcmReactiveStepper::get_next_support_foot_position)
        .def("get_step_duration", &DcmReactiveStepper::get_step_duration)
        .def("get_time_from_last_step_touchdown",
             &DcmReactiveStepper::get_time_from_last_step_touchdown)
        .def("get_flying_foot_position",
             &DcmReactiveStepper::get_flying_foot_position)
        .def("get_is_left_leg_in_contact",
             &DcmReactiveStepper::get_is_left_leg_in_contact)
        .def("get_forces", &DcmReactiveStepper::get_forces);
}