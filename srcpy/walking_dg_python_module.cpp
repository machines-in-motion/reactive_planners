/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the Device and the periodic call to python.
 */

#include <eigenpy/eigenpy.hpp>

#include "dynamic-graph/python/module.hh"
#include "dynamic-graph/python/signal.hh"

#include "reactive_planners/dynamic_graph/dcm_reactive_stepper.hpp"
#include "reactive_planners/dynamic_graph/quadruped_dcm_reactive_stepper.hpp"
#include "reactive_planners/dynamic_graph/stepper_head.hpp"

namespace dg = dynamicgraph;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(walking)
{
    bp::import("dynamic_graph");
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<Eigen::Vector3d>();
    eigenpy::enableEigenPySpecific<Eigen::Vector7d>();
    eigenpy::enableEigenPySpecific<Eigen::Vector9d>();

    using reactive_planners::dynamic_graph::DcmReactiveStepper;
    using reactive_planners::dynamic_graph::StepperHead;
    dynamicgraph::python::exposeEntity<DcmReactiveStepper>();
    dynamicgraph::python::exposeEntity<StepperHead>();

    using reactive_planners::dynamic_graph::QuadrupedDcmReactiveStepper;
    dynamicgraph::python::exposeEntity<QuadrupedDcmReactiveStepper>()
        // NOTE: The `initialize` method has too many args for boost python.
        //       Therefore we use two initialize functions.
        // .def("initialize", &QuadrupedDcmReactiveStepper::initialize)
        .def("initialize_placement",
             &QuadrupedDcmReactiveStepper::initialize_placement)
        .def("initialize_stepper",
             &QuadrupedDcmReactiveStepper::initialize_stepper)
        .def("set_steptime_nominal",
            &QuadrupedDcmReactiveStepper::set_steptime_nominal)
        .def("set_polynomial_end_effector_trajectory",
             &QuadrupedDcmReactiveStepper::
                 set_polynomial_end_effector_trajectory)
        .def(
            "set_dynamical_end_effector_trajectory",
            &QuadrupedDcmReactiveStepper::set_dynamical_end_effector_trajectory)
        .def("start", &QuadrupedDcmReactiveStepper::start)
        .def("stop", &QuadrupedDcmReactiveStepper::stop);
}
