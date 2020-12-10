/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the DcmVrpPlanner class
 */

#include "reactive_planners/dcm_vrp_planner.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace reactive_planners;

void bind_dcm_vrp_planner(pybind11::module &module)
{
    pybind11::class_<DcmVrpPlanner>(module, "DcmVrpPlanner")
        .def(pybind11::init<>())
        // public methods
        .def("initialize", &DcmVrpPlanner::initialize)
        .def("solve", &DcmVrpPlanner::solve)
        .def("internal_checks", &DcmVrpPlanner::internal_checks)
        .def("print_solver", &DcmVrpPlanner::print_solver)
        .def("update",
             (void (DcmVrpPlanner::*)(const Eigen::Ref<const Eigen::Vector3d> &,
                                      const double &,
                                      const bool &,
                                      const Eigen::Ref<const Eigen::Vector3d> &,
                                      const Eigen::Ref<const Eigen::Vector3d> &,
                                      const Eigen::Ref<const Eigen::Vector3d> &,
                                      const pinocchio::SE3 &,
                                      const double &)) &
                 DcmVrpPlanner::update)
        .def("update",
             (void (DcmVrpPlanner::*)(const Eigen::Ref<const Eigen::Vector3d> &,
                                      const double &,
                                      const bool &,
                                      const Eigen::Ref<const Eigen::Vector3d> &,
                                      const Eigen::Ref<const Eigen::Vector3d> &,
                                      const Eigen::Ref<const Eigen::Vector3d> &,
                                      const double &,
                                      const double &)) &
                 DcmVrpPlanner::update)

        // getters
        .def("get_t_nom", &DcmVrpPlanner::get_t_nom)
        .def("get_tau_nom", &DcmVrpPlanner::get_tau_nom)
        .def("get_l_nom", &DcmVrpPlanner::get_l_nom)
        .def("get_w_nom", &DcmVrpPlanner::get_w_nom)
        .def("get_bx_nom", &DcmVrpPlanner::get_bx_nom)
        .def("get_by_nom", &DcmVrpPlanner::get_by_nom)
        .def("get_world_M_local", &DcmVrpPlanner::get_world_M_local)
        .def("get_dcm_local", &DcmVrpPlanner::get_dcm_local)
        .def("get_current_step_location_local",
             &DcmVrpPlanner::get_current_step_location_local)
        .def("get_v_des_local", &DcmVrpPlanner::get_v_des_local)
        .def("get_dcm_nominal", &DcmVrpPlanner::get_dcm_nominal)
        .def("get_next_step_location", &DcmVrpPlanner::get_next_step_location)
        .def("get_duration_before_step_landing",
             &DcmVrpPlanner::get_duration_before_step_landing)

        // String representation of the sovler.
        .def("__repr__",
             [](const DcmVrpPlanner &planner) { return planner.to_string(); });
}