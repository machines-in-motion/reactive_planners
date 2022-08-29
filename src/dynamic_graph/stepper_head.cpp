/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Definition of the StepperHead class.
 */

#include "reactive_planners/dynamic_graph/stepper_head.hpp"

#include <dynamic-graph/factory.h>

#include <iostream>
#include <sstream>

namespace reactive_planners
{
namespace dynamic_graph
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StepperHead, "StepperHead");

/*
 * The entity.
 */
StepperHead::StepperHead(const std::string& name)
    : dynamicgraph::Entity(name),
      // Input signals.
      next_step_location_sin_(
          NULL, make_signal_string(true, "Vector3d", "next_step_location_sin")),
      duration_before_step_landing_sin_(
          NULL,
          make_signal_string(
              true, "double", "duration_before_step_landing_sin")),

      // Output signals.
      //
      // The value for the output signals is computed central in the
      // time_from_last_step_touchdown method.

      time_from_last_step_touchdown_sout_(
          boost::bind(
              &StepperHead::time_from_last_step_touchdown, this, _1, _2),
          next_step_location_sin_ << duration_before_step_landing_sin_,
          make_signal_string(
              false, "double", "time_from_last_step_touchdown_sout")),
      is_left_leg_in_contact_sout_(
          boost::bind(&StepperHead::is_left_leg_in_contact, this, _1, _2),
          time_from_last_step_touchdown_sout_,
          make_signal_string(false, "int", "is_left_leg_in_contact_sout")),
      old_step_location_sout_(
          boost::bind(&StepperHead::old_step_location, this, _1, _2),
          time_from_last_step_touchdown_sout_,
          make_signal_string(false, "Vector3d", "old_step_location_sout")),
      current_step_location_sout_(
          boost::bind(&StepperHead::current_step_location, this, _1, _2),
          time_from_last_step_touchdown_sout_,
          make_signal_string(false, "Vector3d", "current_step_location_sout")),
      is_left_leg_in_contact_(true)
{
    Entity::signalRegistration(
        next_step_location_sin_
        << duration_before_step_landing_sin_
        << time_from_last_step_touchdown_sout_ << is_left_leg_in_contact_sout_
        << old_step_location_sout_ << current_step_location_sout_);
}

double& StepperHead::time_from_last_step_touchdown(double& time_last_step,
                                                   int time)
{
    static int init_time = -1;
    double t_end;

    if (init_time == -1 || init_time == time)
    {
        init_time = time;
        dg_time_phase_switch_ = time;
        t_end = 1;
        time_last_step = 0;
        old_step_location_.setZero();
        current_step_location_.setZero();
    }
    else
    {
        t_end = duration_before_step_landing_sin_.access(time);
    }

    // TODO: Remove assumption of running controller at 1 kHz.
    time_last_step = 0.001 * (double)(time - dg_time_phase_switch_);

    if (time_last_step > t_end)
    {
        // Switch the contact phase.
        is_left_leg_in_contact_ = !is_left_leg_in_contact_;
        dg_time_phase_switch_ = time;
        time_last_step = 0.;

        old_step_location_ = current_step_location_;
        current_step_location_ = next_step_location_sin_.access(time);
    }

    return time_last_step;
}

int& StepperHead::is_left_leg_in_contact(int& left_leg_in_contact, int time)
{
    // Ensure internals are computed.
    time_from_last_step_touchdown_sout_.access(time);

    left_leg_in_contact = is_left_leg_in_contact_;
    return left_leg_in_contact;
}

dynamicgraph::Vector& StepperHead::old_step_location(
    dynamicgraph::Vector& step_location, int time)
{
    // Ensure internals are computed.
    time_from_last_step_touchdown_sout_.access(time);

    step_location = old_step_location_;
    return step_location;
}

dynamicgraph::Vector& StepperHead::current_step_location(
    dynamicgraph::Vector& step_location, int time)
{
    // Ensure internals are computed.
    time_from_last_step_touchdown_sout_.access(time);

    step_location = current_step_location_;
    return step_location;
}

std::string StepperHead::make_signal_string(const bool& is_input_signal,
                                            const std::string& signal_type,
                                            const std::string& signal_name)
{
    std::ostringstream oss;
    oss << CLASS_NAME << "(" << name
        << ")::" << (is_input_signal ? "input" : "output") << "(" << signal_type
        << ")::" << signal_name;
    return oss.str();
}

}  // namespace dynamic_graph
}  // namespace reactive_planners
