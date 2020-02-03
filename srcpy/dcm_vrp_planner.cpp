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

void bind_dcm_vrp_planner(pybind11::module &module) {

  pybind11::class_<DcmVrpPlanner>(module, "DcmVrpPlanner")
      .def(pybind11::init<>())
      // public methods
      .def("initialize", &DcmVrpPlanner::initialize)
      .def("update", &DcmVrpPlanner::update)
      .def("solve", &DcmVrpPlanner::solve)
      .def("internal_checks", &DcmVrpPlanner::internal_checks)
      .def("print_solver", &DcmVrpPlanner::print_solver)
      .def("py_update", &DcmVrpPlanner::py_update)

      // getters
      .def_property_readonly("get_t_nom", &DcmVrpPlanner::get_t_nom)
      .def_property_readonly("get_tau_nom", &DcmVrpPlanner::get_tau_nom)
      .def_property_readonly("get_l_nom", &DcmVrpPlanner::get_l_nom)
      .def_property_readonly("get_w_nom", &DcmVrpPlanner::get_w_nom)
      .def_property_readonly("get_bx_nom", &DcmVrpPlanner::get_bx_nom)
      .def_property_readonly("get_by_nom", &DcmVrpPlanner::get_by_nom)
      .def_property_readonly("get_world_M_local",
                             &DcmVrpPlanner::get_world_M_local)
      .def_property_readonly("get_dcm_local", &DcmVrpPlanner::get_dcm_local)
      .def_property_readonly("get_current_step_location_local",
                             &DcmVrpPlanner::get_current_step_location_local)
      .def_property_readonly("get_v_des_local", &DcmVrpPlanner::get_v_des_local)
      .def_property_readonly("get_dcm_nominal", &DcmVrpPlanner::get_dcm_nominal)
      .def_property_readonly("get_next_step_location",
                             &DcmVrpPlanner::get_next_step_location)
      .def_property_readonly("get_duration_before_step_landing",
                             &DcmVrpPlanner::get_duration_before_step_landing)

      // String representation of the sovler.
      .def("__repr__",
           [](const DcmVrpPlanner &planner) { return planner.to_string(); });
}