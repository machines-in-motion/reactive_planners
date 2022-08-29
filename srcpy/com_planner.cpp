/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the DcmVrpPlanner class
 */

#include "reactive_planners/com_planner.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace reactive_planners;

void bind_com_planner(pybind11::module &module)
{
    pybind11::class_<ComPlanner>(module, "ComPlanner")
            .def(pybind11::init<>())
                    // public methods
            .def("update_com_single_support", &ComPlanner::update_com_single_support)
                    // getters
            .def("get_com", &ComPlanner::get_com)
            .def("get_com_d", &ComPlanner::get_com_d)
            .def("get_com_dd", &ComPlanner::get_com_dd);
}