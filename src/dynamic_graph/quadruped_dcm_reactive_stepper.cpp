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

#include "pinocchio/math/rpy.hpp"

#include <iostream>
#include <sstream>

namespace reactive_planners
{
namespace dynamic_graph
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(QuadrupedDcmReactiveStepper,
                                   "QuadrupedDcmReactiveStepper");

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

QuadrupedDcmReactiveStepper::QuadrupedDcmReactiveStepper(
    const std::string& name)
    :  // Inheritance.
      dynamicgraph::Entity(name),

      init_placement_(false),

      // Input signals.
      define_input_signal(current_front_left_foot_position_sin_, "Vector3d"),
      define_input_signal(current_front_right_foot_position_sin_, "Vector3d"),
      define_input_signal(current_hind_left_foot_position_sin_, "Vector3d"),
      define_input_signal(current_hind_right_foot_position_sin_, "Vector3d"),

      define_input_signal(current_front_left_foot_velocity_sin_, "Vector3d"),
      define_input_signal(current_front_right_foot_velocity_sin_, "Vector3d"),
      define_input_signal(current_hind_left_foot_velocity_sin_, "Vector3d"),
      define_input_signal(current_hind_right_foot_velocity_sin_, "Vector3d"),

      define_input_signal(com_position_sin_, "Vector3d"),
      define_input_signal(com_velocity_sin_, "Vector3d"),

      define_input_signal(xyzquat_base_sin_, "Vector7d"),
      define_input_signal(is_closed_loop_sin_, "double"),

      define_input_signal(desired_com_velocity_sin_, "Vector3d"),

      // Output signals.
      define_output_signal(
          front_left_foot_position_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::front_left_foot_position),
      define_output_signal(
          front_right_foot_position_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::front_right_foot_position),
      define_output_signal(
          hind_left_foot_position_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::hind_left_foot_position),
      define_output_signal(
          hind_right_foot_position_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::hind_right_foot_position),

      define_output_signal(
          front_left_foot_velocity_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::front_left_foot_velocity),
      define_output_signal(
          front_right_foot_velocity_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::front_right_foot_velocity),
      define_output_signal(
          hind_left_foot_velocity_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::hind_left_foot_velocity),
      define_output_signal(
          hind_right_foot_velocity_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::hind_right_foot_velocity),

      define_output_signal(
          front_left_foot_acceleration_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::front_left_foot_acceleration),
      define_output_signal(
          front_right_foot_acceleration_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::front_right_foot_acceleration),
      define_output_signal(
          hind_left_foot_acceleration_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::hind_left_foot_acceleration),
      define_output_signal(
          hind_right_foot_acceleration_sout_,
          "Vector3d",
          &QuadrupedDcmReactiveStepper::hind_right_foot_acceleration),

      define_output_signal(feasible_com_velocity_sout_,
                           "Vector3d",
                           &QuadrupedDcmReactiveStepper::feasible_com_velocity),
      define_output_signal(contact_array_sout_,
                           "Vector4d",
                           &QuadrupedDcmReactiveStepper::contact_array),
      define_output_signal(swing_foot_forces_sout_,
                           "Vector24d",
                           &QuadrupedDcmReactiveStepper::swing_foot_forces),

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
              << current_hind_right_foot_velocity_sin_ << com_position_sin_
              << com_velocity_sin_ << xyzquat_base_sin_ << is_closed_loop_sin_
              << desired_com_velocity_sin_,
          make_signal_string(false, "bool", "inner_sout"))
{
    /*
     * Initializes the signals
     */
    // clang-format off
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
              << feasible_com_velocity_sout_
              << contact_array_sout_
              << swing_foot_forces_sout_
              );
    // clang-format on
}

std::string QuadrupedDcmReactiveStepper::make_signal_string(
    const bool& is_input_signal,
    const std::string& signal_type,
    const std::string& signal_name)
{
    std::ostringstream oss;
    oss << CLASS_NAME << "(" << name
        << ")::" << (is_input_signal ? "input" : "output") << "(" << signal_type
        << ")::" << signal_name;
    return oss.str();
}

void QuadrupedDcmReactiveStepper::initialize_placement(
    const Eigen::Ref<const Eigen::Vector7d>& base_placement,
    const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_position)
{
    init_placement_ = true;
    init_base_placement_ = base_placement;
    init_front_left_foot_position_ = front_left_foot_position;
    init_front_right_foot_position_ = front_right_foot_position;
    init_hind_left_foot_position_ = hind_left_foot_position;
    init_hind_right_foot_position_ = hind_right_foot_position;
}

void QuadrupedDcmReactiveStepper::initialize_stepper(
    const bool& is_left_leg_in_contact,
    const double& l_min,
    const double& l_max,
    const double& w_min,
    const double& w_max,
    const double& t_min,
    const double& t_max,
    const double& l_p,
    const double& com_height,
    const Eigen::Vector9d& weight,
    const double& mid_air_foot_height,
    const double& control_period,
    const double& planner_loop)
{
    if (!init_placement_)
    {
        throw std::runtime_error("Please call initialize_placement first.");
    }

    stepper_.initialize(is_left_leg_in_contact,
                        l_min,
                        l_max,
                        w_min,
                        w_max,
                        t_min,
                        t_max,
                        l_p,
                        com_height,
                        weight,
                        mid_air_foot_height,
                        control_period,
                        planner_loop,
                        init_base_placement_,
                        init_front_left_foot_position_,
                        init_front_right_foot_position_,
                        init_hind_left_foot_position_,
                        init_hind_right_foot_position_);
}

void QuadrupedDcmReactiveStepper::initialize(
    const bool& is_left_leg_in_contact,
    const double& l_min,
    const double& l_max,
    const double& w_min,
    const double& w_max,
    const double& t_min,
    const double& t_max,
    const double& l_p,
    const double& com_height,
    const Eigen::Vector9d& weight,
    const double& mid_air_foot_height,
    const double& control_period,
    const double& planner_loop,
    const Eigen::Ref<const Eigen::Vector7d>& base_placement,
    const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_position)
{
    stepper_.initialize(is_left_leg_in_contact,
                        l_min,
                        l_max,
                        w_min,
                        w_max,
                        t_min,
                        t_max,
                        l_p,
                        com_height,
                        weight,
                        mid_air_foot_height,
                        control_period,
                        planner_loop,
                        base_placement,
                        front_left_foot_position,
                        front_right_foot_position,
                        hind_left_foot_position,
                        hind_right_foot_position);
}

void QuadrupedDcmReactiveStepper::set_steptime_nominal(double t_nom)
{
    stepper_.set_steptime_nominal(t_nom);
}

void QuadrupedDcmReactiveStepper::set_polynomial_end_effector_trajectory()
{
    stepper_.set_polynomial_end_effector_trajectory();
}

void QuadrupedDcmReactiveStepper::set_dynamical_end_effector_trajectory()
{
    stepper_.set_dynamical_end_effector_trajectory();
}

void QuadrupedDcmReactiveStepper::start()
{
    stepper_.start();
}

void QuadrupedDcmReactiveStepper::stop()
{
    stepper_.stop();
}

bool& QuadrupedDcmReactiveStepper::inner(bool& s, int time)
{
    const dynamicgraph::Vector& xyzquat_base = xyzquat_base_sin_.access(time);
    base_quaternion_.x() = xyzquat_base(3);
    base_quaternion_.y() = xyzquat_base(4);
    base_quaternion_.z() = xyzquat_base(5);
    base_quaternion_.w() = xyzquat_base(6);
    base_quaternion_.normalize();
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(
        base_quaternion_.matrix());

    // Rotate the passed desired_com_velocity_sin_ from local to world frame.
    Eigen::Vector3d vec_yaw;
    vec_yaw << 0., 0., rpy(2);
    stepper_.set_desired_com_velocity(
        pinocchio::rpy::rpyToMatrix(vec_yaw) *
            desired_com_velocity_sin_.access(time)
    );

    s = stepper_.run(time * 0.001,
                     current_front_left_foot_position_sin_.access(time),
                     current_front_right_foot_position_sin_.access(time),
                     current_hind_left_foot_position_sin_.access(time),
                     current_hind_right_foot_position_sin_.access(time),
                     current_front_left_foot_velocity_sin_.access(time),
                     current_front_right_foot_velocity_sin_.access(time),
                     current_hind_left_foot_velocity_sin_.access(time),
                     current_hind_right_foot_velocity_sin_.access(time),
                     com_position_sin_.access(time),
                     com_velocity_sin_.access(time),
                     rpy(2),
                     is_closed_loop_sin_.access(time));
    return s;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::front_left_foot_position(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_left_foot_position();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::front_right_foot_position(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_right_foot_position();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::hind_left_foot_position(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_left_foot_position();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::hind_right_foot_position(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_right_foot_position();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::front_left_foot_velocity(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_left_foot_velocity();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::front_right_foot_velocity(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_right_foot_velocity();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::hind_left_foot_velocity(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_left_foot_velocity();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::hind_right_foot_velocity(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_right_foot_velocity();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::front_left_foot_acceleration(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_left_foot_acceleration();
    return signal_data;
}

dynamicgraph::Vector&
QuadrupedDcmReactiveStepper::front_right_foot_acceleration(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_front_right_foot_acceleration();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::hind_left_foot_acceleration(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_left_foot_acceleration();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::hind_right_foot_acceleration(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_hind_right_foot_acceleration();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::feasible_com_velocity(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_feasible_com_velocity();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::contact_array(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_contact_array();
    return signal_data;
}

dynamicgraph::Vector& QuadrupedDcmReactiveStepper::swing_foot_forces(
    dynamicgraph::Vector& signal_data, int time)
{
    inner_sout_.access(time);
    signal_data = stepper_.get_forces();
    return signal_data;
}

}  // namespace dynamic_graph
}  // namespace reactive_planners
