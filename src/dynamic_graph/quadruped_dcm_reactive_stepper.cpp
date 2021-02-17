/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Definition of the QuadrupedDcmReactiveStepper class
 */

#include "reactive_planners/dynamic_graph/quadruped_dcm_reactive_stepper.hpp"

#include <dynamic-graph/factory.h>

#include <iostream>
#include <sstream>

namespace reactive_planners
{
namespace dynamic_graph
{
using ::dynamicgraph::command::docCommandVoid0;
using ::dynamicgraph::command::docCommandVoid1;
using ::dynamicgraph::command::makeCommandVoid0;
using ::dynamicgraph::command::makeCommandVoid1;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(QuadrupedDcmReactiveStepper, "QuadrupedDcmReactiveStepper");

/*
 * The entity.
 */

// clang-format off
#define crop_underscore(var_name)                                              \
    std::string(#var_name).substr(0, std::string(#var_name).size() - 1)

#define define_input_signal(signal_var_name, signal_type)                      \
  signal_var_name(                                                             \
      NULL,                                                                    \
      make_signal_string(true, signal_type, crop_underscore(signal_var_name)))

#define define_output_signal(signal_var_name, signal_type, callback)           \
  signal_var_name(                                                             \
      boost::bind(callback, this, _1, _2),                                     \
      inner_sout_,                                                             \
      make_signal_string(false, signal_type, crop_underscore(signal_var_name)))
// clang-format on


QuadrupedDcmReactiveStepper::QuadrupedDcmReactiveStepper(const std::string &name)
    : // Inheritance.
      dynamicgraph::Entity(name),

      // Input signals.
      define_input_signal(current_front_left_foot_position_sin_, "Vector3d"),
      define_input_signal(current_front_right_foot_position_sin_, "Vector3d"),
      define_input_signal(current_hind_left_foot_position_sin_ "Vector3d"),
      define_input_signal(current_hind_right_foot_position_sin_, "Vector3d"),

      define_input_signal(current_front_left_foot_velocity_sin_, "Vector3d"),
      define_input_signal(current_front_right_foot_velocity_sin_, "Vector3d"),
      define_input_signal(current_hind_left_foot_velocity_sin_ "Vector3d"),
      define_input_signal(current_hind_right_foot_velocity_sin_, "Vector3d"),

      define_input_signal(com_position_sin_, "Vector3d"),
      define_input_signal(com_velocity_sin_, "Vector3d"),

      define_input_signal(base_yaw_sin_, "Vector3d"),
      define_input_signal(xyzquat_base_sin_, "Vector7d"),
      define_input_signal(is_closed_loop_sin_, "double"),

      define_input_signal(desired_com_velocity_sin_, "Vector3d"),

      // Output signals.
      define_output_signal(front_left_foot_position_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::front_left_foot_position),
      define_output_signal(front_right_foot_position_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::front_right_foot_position),
      define_output_signal(hind_left_foot_position_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::hind_left_foot_position),
      define_output_signal(hind_right_foot_position_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::hind_right_foot_position),

      define_output_signal(front_left_foot_velocity_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::front_left_foot_velocity),
      define_output_signal(front_right_foot_velocity_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::front_right_foot_velocity),
      define_output_signal(hind_left_foot_velocity_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::hind_left_foot_velocity),
      define_output_signal(hind_right_foot_velocity_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::hind_right_foot_velocity),

      define_output_signal(front_left_foot_acceleration_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::front_left_foot_acceleration),
      define_output_signal(front_right_foot_acceleration_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::front_right_foot_acceleration),
      define_output_signal(hind_left_foot_acceleration_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::hind_left_foot_acceleration),
      define_output_signal(hind_right_foot_acceleration_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::hind_right_foot_acceleration),

      define_output_signal(feasible_com_velocity,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::feasible_com_velocity),
      define_output_signal(contact_array,
                           "Vector4d",
                           &QuadrupedDcmReactiveStepper::contact_array),

      // Inner signal.
      inner_sout_(
          boost::bind(&QuadrupedDcmReactiveStepper::inner, this, _1, _2),
              current_front_left_foot_position_sin_
              << current_front_right_foot_position_sin_
              << current_hind_left_foot_position_sin_
              << current_hind_right_foot_position_sin_
              << current_front_left_foot_velocity_sin_
              << current_front_right_foot_velocity_sin_
              << current_hind_left_foot_velocity_sin_
              << current_hind_right_foot_velocity_sin_
              << com_position_sin_
              << com_velocity_sin_
              << base_yaw_sin_
              << xyzquat_base_sin_
              << is_closed_loop_sin_
              << desired_com_velocity_sin_
          make_signal_string(false, "bool", "inner_sout")),
{
    /*
     * Initializes the signals
     */
    Entity::signalRegistration(
        current_front_left_foot_position_sin_
              << current_front_right_foot_position_sin_
              << current_hind_left_foot_position_sin_
              << current_hind_right_foot_position_sin_
              << current_front_left_foot_velocity_sin_
              << current_front_right_foot_velocity_sin_
              << current_hind_left_foot_velocity_sin_
              << current_hind_right_foot_velocity_sin_
              << com_position_sin_
              << com_velocity_sin_
              << base_yaw_sin_
              << xyzquat_base_sin_
              << is_closed_loop_sin_
              << desired_com_velocity_sin_

              << front_left_foot_position_sout_
              << front_right_foot_position_sout_
              << hind_left_foot_position_sout_
              << hind_right_foot_position_sout_
              << front_left_foot_velocity_sout_
              << front_right_foot_velocity_sout_
              << hind_left_foot_velocity_sout_
              << hind_right_foot_velocity_sout_
              << front_left_foot_acceleration_sout_
              << front_right_foot_acceleration_sout_
              << hind_left_foot_acceleration_sout_
              << hind_right_foot_acceleration_sout_
              << feasible_com_velocity
              << contact_array
              );
}

QuadrupedDcmReactiveStepper::front_left_foot_position(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_left_foot_position();
    return signal_data;
}

QuadrupedDcmReactiveStepper::front_right_foot_position(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_right_foot_position();
    return signal_data;
}

QuadrupedDcmReactiveStepper::hind_left_foot_position(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_left_foot_position();
    return signal_data;
}

QuadrupedDcmReactiveStepper::hind_right_foot_position(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_right_foot_position();
    return signal_data;
}

QuadrupedDcmReactiveStepper::front_left_foot_velocity(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_left_foot_velocity();
    return signal_data;
}

QuadrupedDcmReactiveStepper::front_right_foot_velocity(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_right_foot_velocity();
    return signal_data;
}

QuadrupedDcmReactiveStepper::hind_left_foot_velocity(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_left_foot_velocity();
    return signal_data;
}

QuadrupedDcmReactiveStepper::hind_right_foot_velocity(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_right_foot_velocity();
    return signal_data;
}

QuadrupedDcmReactiveStepper::front_left_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_left_foot_acceleration();
    return signal_data;
}

QuadrupedDcmReactiveStepper::front_right_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_right_foot_acceleration();
    return signal_data;
}

QuadrupedDcmReactiveStepper::hind_left_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_left_foot_acceleration();
    return signal_data;
}

QuadrupedDcmReactiveStepper::hind_right_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_right_foot_acceleration();
    return signal_data;
}

QuadrupedDcmReactiveStepper::feasible_com_velocity(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_feasible_com_velocity();
    return signal_data;
}

QuadrupedDcmReactiveStepper::contact_array(
        dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_contact_array();
    return signal_data;
}



