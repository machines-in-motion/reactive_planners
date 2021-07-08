/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the StepperHead class
 */

#include "reactive_planners/quadruped_dcm_reactive_stepper.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <boost/python.hpp>

using namespace reactive_planners;

void bind_quadruped_dcm_reactive_stepper(pybind11::module &module)
{
    pybind11::class_<QuadrupedDcmReactiveStepper>(module,
                                                  "QuadrupedDcmReactiveStepper")
        .def(pybind11::init<>())

        // Public methods.
        .def("initialize", &QuadrupedDcmReactiveStepper::initialize)
        .def("run",
             (bool (QuadrupedDcmReactiveStepper::*)(
                 double,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const Eigen::Ref<const Eigen::Vector3d> &,
                 const double &,
                 const bool &)) &
                 QuadrupedDcmReactiveStepper::run)
        .def(
            "run",
            [](QuadrupedDcmReactiveStepper &obj,
               double time,
               const Eigen::Ref<const Eigen::Vector3d>
                   &front_left_foot_position,
               const Eigen::Ref<const Eigen::Vector3d>
                   &front_right_foot_position,
               const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_position,
               const Eigen::Ref<const Eigen::Vector3d>
                   &hind_right_foot_position,
               const Eigen::Ref<const Eigen::Vector3d>
                   &front_left_foot_velocity,
               const Eigen::Ref<const Eigen::Vector3d>
                   &front_right_foot_velocity,
               const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_velocity,
               const Eigen::Ref<const Eigen::Vector3d>
                   &hind_right_foot_velocity,
               const Eigen::Ref<const Eigen::Vector3d> &com_position,
               const Eigen::Ref<const Eigen::Vector3d> &com_velocity,
               const pybind11::object &py_world_M_base,
               const bool &is_closed_loop) {
                const pinocchio::SE3 &world_M_base =
                    boost::python::extract<const pinocchio::SE3 &>(
                        py_world_M_base.ptr());
                bool ret = obj.run(time,
                                   front_left_foot_position,
                                   front_right_foot_position,
                                   hind_left_foot_position,
                                   hind_right_foot_position,
                                   front_left_foot_velocity,
                                   front_right_foot_velocity,
                                   hind_left_foot_velocity,
                                   hind_right_foot_velocity,
                                   com_position,
                                   com_velocity,
                                   world_M_base,
                                   is_closed_loop);
                return ret;
            })
        .def("start", &QuadrupedDcmReactiveStepper::start)
        .def("stop", &QuadrupedDcmReactiveStepper::stop)

        // Setters.
        .def("set_desired_com_velocity",
             &QuadrupedDcmReactiveStepper::set_desired_com_velocity)
        .def("set_feet_pos", &QuadrupedDcmReactiveStepper::set_feet_pos)
        .def("set_polynomial_end_effector_trajectory",
             &QuadrupedDcmReactiveStepper::
                 set_polynomial_end_effector_trajectory)
        .def(
            "set_dynamical_end_effector_trajectory",
            &QuadrupedDcmReactiveStepper::set_dynamical_end_effector_trajectory)
        .def(
            "set_steptime_nominal",
            &QuadrupedDcmReactiveStepper::set_steptime_nominal)

        // Getters.
        .def("get_front_left_foot_position",
             &QuadrupedDcmReactiveStepper::get_front_left_foot_position)
        .def("get_front_left_foot_velocity",
             &QuadrupedDcmReactiveStepper::get_front_left_foot_velocity)
        .def("get_front_right_foot_position",
             &QuadrupedDcmReactiveStepper::get_front_right_foot_position)
        .def("get_front_right_foot_velocity",
             &QuadrupedDcmReactiveStepper::get_front_right_foot_velocity)
        .def("get_hind_left_foot_position",
             &QuadrupedDcmReactiveStepper::get_hind_left_foot_position)
        .def("get_hind_left_foot_velocity",
             &QuadrupedDcmReactiveStepper::get_hind_left_foot_velocity)
        .def("get_hind_right_foot_position",
             &QuadrupedDcmReactiveStepper::get_hind_right_foot_position)
        .def("get_hind_right_foot_velocity",
             &QuadrupedDcmReactiveStepper::get_hind_right_foot_velocity)
        .def("get_feasible_com_velocity",
             &QuadrupedDcmReactiveStepper::get_feasible_com_velocity)
        .def("get_contact_array",
             &QuadrupedDcmReactiveStepper::get_contact_array)
        .def("get_forces",
             &QuadrupedDcmReactiveStepper::get_forces);
}