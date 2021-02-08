/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the Device and the periodic call to python.
 */

#include "reactive_planners/dynamic_graph/dcm_reactive_stepper.hpp"
#include "reactive_planners/dynamic_graph/stepper_head.hpp"

typedef boost::mpl::vector<reactive_planners::dynamic_graph::DcmReactiveStepper,
                           reactive_planners::dynamic_graph::StepperHead>
    entities_t;
