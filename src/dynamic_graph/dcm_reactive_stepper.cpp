/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Definition of the DcmReactiveStepper class
 */

#include "reactive_planners/dynamic_graph/dcm_reactive_stepper.hpp"

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

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DcmReactiveStepper, "DcmReactiveStepper");

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

DcmReactiveStepper::DcmReactiveStepper(const std::string &name)
    :  // Inheritance.
      dynamicgraph::Entity(name),
      // Input signals.
      define_input_signal(desired_com_velocity_sin_, "Vector3d"),
      define_input_signal(current_left_foot_position_sin_, "Vector3d"),
      define_input_signal(current_right_foot_position_sin_, "Vector3d"),
      define_input_signal(current_left_foot_velocity_sin_, "Vector3d"),
      define_input_signal(current_right_foot_velocity_sin_, "Vector3d"),
      define_input_signal(com_position_sin_, "Vector3d"),
      define_input_signal(com_velocity_sin_, "Vector3d"),
      define_input_signal(base_yaw_sin_, "Vector3d"),
      define_input_signal(xyzquat_base_sin_, "Vector7d"),
      define_input_signal(is_closed_loop_sin_, "double"),
      // Output signals.
      define_output_signal(right_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::right_foot_position),
      define_output_signal(right_foot_velocity_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::right_foot_velocity),
      define_output_signal(right_foot_acceleration_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::right_foot_acceleration),
      define_output_signal(left_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::left_foot_position),
      define_output_signal(left_foot_velocity_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::left_foot_velocity),
      define_output_signal(left_foot_acceleration_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::left_foot_acceleration),
      define_output_signal(local_right_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::local_right_foot_position),
      define_output_signal(local_right_foot_velocity_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::local_right_foot_velocity),
      define_output_signal(local_left_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::local_left_foot_position),
      define_output_signal(local_left_foot_velocity_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::local_left_foot_velocity),
      define_output_signal(feasible_com_velocity_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::feasible_com_velocity),
      define_output_signal(previous_support_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::previous_support_foot_position),
      define_output_signal(current_support_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::current_support_foot_position),
      define_output_signal(next_support_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::next_support_foot_position),
      define_output_signal(
          step_duration_sout_, "double", &DcmReactiveStepper::step_duration),
      define_output_signal(time_from_last_step_touchdown_sout_,
                           "double",
                           &DcmReactiveStepper::time_from_last_step_touchdown),
      define_output_signal(flying_foot_position_sout_,
                           "Vector3d",
                           &DcmReactiveStepper::flying_foot_position),
      define_output_signal(is_left_leg_in_contact_sout_,
                           "int",
                           &DcmReactiveStepper::is_left_leg_in_contact),
      define_output_signal(
          has_solution_sout_, "int", &DcmReactiveStepper::has_solution),
      define_output_signal(dcm_sout_, "Vector3d", &DcmReactiveStepper::dcm),
      define_output_signal(
          force_sout_, "Vector12d", &DcmReactiveStepper::force),
      // Inner signal.
      inner_sout_(
          boost::bind(&DcmReactiveStepper::inner, this, _1, _2),
          desired_com_velocity_sin_
              << current_left_foot_position_sin_
              << current_right_foot_position_sin_ << com_position_sin_
              << current_left_foot_velocity_sin_
              << current_right_foot_velocity_sin_ << com_velocity_sin_
              << base_yaw_sin_,  // << is_closed_loop_sin_,//Lhum closed loop
          make_signal_string(false, "Vector3d", "inner_sout")),
      // Parameters.
      control_period_(0.0),
      has_solution_(0)
{
    /*
     * Initializes the signals
     */
    Entity::signalRegistration(
        desired_com_velocity_sin_
        << current_left_foot_position_sin_ << current_right_foot_position_sin_
        << current_left_foot_velocity_sin_ << current_right_foot_velocity_sin_
        << com_position_sin_ << com_velocity_sin_ << base_yaw_sin_
        << xyzquat_base_sin_
        << right_foot_position_sout_  //  << is_closed_loop_sin_//Lhum closed
                                      //  loop
        << right_foot_velocity_sout_ << right_foot_acceleration_sout_
        << left_foot_position_sout_ << left_foot_velocity_sout_
        << left_foot_acceleration_sout_ << local_right_foot_position_sout_
        << local_right_foot_velocity_sout_ << local_left_foot_position_sout_
        << local_left_foot_velocity_sout_ << feasible_com_velocity_sout_
        << previous_support_foot_position_sout_
        << current_support_foot_position_sout_
        << next_support_foot_position_sout_ << step_duration_sout_
        << time_from_last_step_touchdown_sout_ << flying_foot_position_sout_
        << is_left_leg_in_contact_sout_ << has_solution_sout_ << inner_sout_
        << dcm_sout_ << force_sout_);
    /*
     * Initializes the commands
     */
    addCommand("initializeParamVector",
               makeCommandVoid1(*this,
                                &DcmReactiveStepper::initializeParamVector,
                                docCommandVoid1("Initialize parameters.",
                                                "Parameters (vector)")));

    addCommand("start",
               makeCommandVoid0(*this,
                                &DcmReactiveStepper::start,
                                docCommandVoid0("Start stepping")));

    addCommand("stop",
               makeCommandVoid0(*this,
                                &DcmReactiveStepper::stop,
                                docCommandVoid0("Start stepping")));
}

void DcmReactiveStepper::initialize(
    const bool &is_left_leg_in_contact,
    const double &l_min,
    const double &l_max,
    const double &w_min,
    const double &w_max,
    const double &t_min,
    const double &t_max,
    const double &l_p,
    const double &com_height,
    Eigen::Ref<const Eigen::Vector9d> cost_weights_local,
    const double &mid_air_foot_height,
    const double &control_period,
    const double &planner_loop,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position)
{
    dcm_reactive_stepper_.initialize(is_left_leg_in_contact,
                                     l_min,
                                     l_max,
                                     w_min,
                                     w_max,
                                     t_min,
                                     t_max,
                                     l_p,
                                     com_height,
                                     cost_weights_local,
                                     mid_air_foot_height,
                                     control_period,
                                     planner_loop,
                                     left_foot_position,
                                     right_foot_position);
    control_period_ = control_period;
}

bool &DcmReactiveStepper::inner(bool &s, int time)
{
    // Access the input signals
    const Eigen::VectorXd &desired_com_velocity =
        desired_com_velocity_sin_.access(time);
    const Eigen::VectorXd &com_position = com_position_sin_.access(time);
    const Eigen::VectorXd &com_velocity = com_velocity_sin_.access(time);
    const Eigen::VectorXd &base_yaw = base_yaw_sin_.access(time);
    //    const Eigen::VectorXd &xyzquat_base = xyzquat_base_sin_.access(time);
    //    const double &is_closed_loop = is_closed_loop_sin_.access(time);//Lhum
    //    closed loop

    //    pinocchio::SE3 world_M_base;
    //    world_M_base.translation() = xyzquat_base.head<3>();
    //    Eigen::Quaterniond quat(xyzquat_base(6), xyzquat_base(3),
    //    xyzquat_base(4), xyzquat_base(5)); world_M_base.rotation() =
    //    quat.toRotationMatrix(); std::cout << "y" << base_yaw(5) << std::endl
    //    << std::endl;
    start_stop_mutex_.lock();
    // Set the desired velocity
    if (base_yaw(5) - last_yaw_ > 1. || base_yaw(5) - last_yaw_ < -1.)
    {
        nb_switch_yaw_ = (nb_switch_yaw_ + 1) % 2;
    }
    last_yaw_ = base_yaw(5);
    //    std::cout << dcm_reactive_stepper_.get_time_from_last_step_touchdown()
    //    << " + " << control_period_ + 0.0000001 << " > " <<
    //    dcm_reactive_stepper_.get_step_duration() << std::endl;
    //    std::cout << time_from_double_support_started_ << " < " <<
    //    double_support_time_ << std::endl;
    if (dcm_reactive_stepper_.get_time_from_last_step_touchdown() +
                control_period_ + 0.0000001 >
            dcm_reactive_stepper_.get_step_duration() &&
        time_from_double_support_started_ < double_support_time_)
    {
        //        std::cout << "DOUBLESUPPORT\n";
        //        std::cout << "DS\n";
        //        std::cout <<
        //        (dcm_reactive_stepper_.get_time_from_last_step_touchdown() +
        //        control_period_ + 0.0000001 >
        //        dcm_reactive_stepper_.get_step_duration()) << std::endl;
        //        std::cout << (time_from_double_support_started_ <
        //        double_support_time_) << std::endl;
        time_from_double_support_started_ += control_period_;
        Eigen::Vector3d current_left_foot_position =
            current_left_foot_position_sin_.access(time);
        Eigen::Vector3d current_right_foot_position =
            current_right_foot_position_sin_.access(time);
        current_left_foot_position[2] = 0;
        current_right_foot_position[2] = 0;
        dcm_reactive_stepper_.set_feet_pos(current_left_foot_position,
                                           current_right_foot_position);
        start_stop_mutex_.unlock();
        return s = true;
    }
    //    std::cout << "NDS\n";
    time_from_double_support_started_ = 0;
    dcm_reactive_stepper_.set_desired_com_velocity(desired_com_velocity);

    //    std::cout << nb_switch_yaw_  << " " <<  base_yaw(5) << std::endl;
    Eigen::VectorXd current_left_foot_position =
        current_left_foot_position_sin_.access(time);
    Eigen::VectorXd current_right_foot_position =
        current_right_foot_position_sin_.access(time);
    Eigen::Vector3d current_left_foot_velocity;
    Eigen::Vector3d current_right_foot_velocity;
    if (false)
    {
        current_left_foot_velocity =
            current_left_foot_velocity_sin_.access(time);
        current_right_foot_velocity =
            current_right_foot_velocity_sin_.access(time);
        if (dcm_reactive_stepper_.get_is_left_leg_in_contact())
        {
            dcm_reactive_stepper_.set_left_foot_position(
                current_left_foot_position);
            dcm_reactive_stepper_.set_left_foot_velocity(
                current_left_foot_velocity);
        }
        else
        {
            dcm_reactive_stepper_.set_right_foot_position(
                current_right_foot_position);
            dcm_reactive_stepper_.set_right_foot_velocity(
                current_right_foot_velocity);
        }
    }  // Lhum closed loop

    // Compute the planner.
    s = dcm_reactive_stepper_.run(time * control_period_,
                                  current_left_foot_position,
                                  current_right_foot_position,
                                  current_left_foot_velocity,
                                  current_right_foot_velocity,
                                  com_position,
                                  com_velocity,
                                  nb_switch_yaw_ * 3.141592 + base_yaw(5),
                                  false);
    start_stop_mutex_.unlock();

    has_solution_ = static_cast<int>(s);
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::dcm(dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_dcm();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::force(dynamicgraph::Vector &s,
                                                int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_force();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::right_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_right_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::right_foot_velocity(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_right_foot_velocity();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::right_foot_acceleration(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_right_foot_acceleration();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::left_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_left_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::left_foot_velocity(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_left_foot_velocity();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::left_foot_acceleration(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_left_foot_acceleration();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::local_right_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_local_right_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::local_right_foot_velocity(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_local_right_foot_velocity();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::local_left_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_local_left_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::local_left_foot_velocity(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_local_left_foot_velocity();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::feasible_com_velocity(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_feasible_com_velocity();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::previous_support_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_previous_support_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::current_support_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_current_support_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::next_support_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_next_support_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

double &DcmReactiveStepper::step_duration(double &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_step_duration();
    start_stop_mutex_.unlock();
    return s;
}

double &DcmReactiveStepper::time_from_last_step_touchdown(double &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_time_from_last_step_touchdown();
    start_stop_mutex_.unlock();
    return s;
}

dynamicgraph::Vector &DcmReactiveStepper::flying_foot_position(
    dynamicgraph::Vector &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_flying_foot_position();
    start_stop_mutex_.unlock();
    return s;
}

int &DcmReactiveStepper::is_left_leg_in_contact(int &s, int time)
{
    inner_sout_.access(time);
    if (time_from_double_support_started_ != 0) return s = 2;  // double support
    start_stop_mutex_.lock();
    s = dcm_reactive_stepper_.get_is_left_leg_in_contact();
    start_stop_mutex_.unlock();
    return s;
}

int &DcmReactiveStepper::has_solution(int &s, int time)
{
    inner_sout_.access(time);
    start_stop_mutex_.lock();
    s = has_solution_;
    start_stop_mutex_.unlock();
    return s;
}

std::string DcmReactiveStepper::make_signal_string(
    const bool &is_input_signal,
    const std::string &signal_type,
    const std::string &signal_name)
{
    std::ostringstream oss;
    oss << CLASS_NAME << "(" << name
        << ")::" << (is_input_signal ? "input" : "output") << "(" << signal_type
        << ")::" << signal_name;
    return oss.str();
}

/*
 * The commands.
 */

void DcmReactiveStepper::initializeParamVector(
    const dynamicgraph::Vector &params)
{
    if (params.size() != 29)
    {
        std::cout << "DcmReactiveStepper::InitializeCommand(): Error invalid "
                     "number of paramters, please retry.";
        return;
    }

    initialize(params(0),               // is_left_leg_in_contact,
               params(1),               // l_min
               params(2),               // l_max
               params(3),               // w_min
               params(4),               // w_max
               params(5),               // t_min
               params(6),               // t_max
               params(7),               // l_p
               params(8),               // com_height
               params.segment<9>(9),    // Vector9d weight,
               params(18),              // mid_air_foot_height,
               params(19),              // control_period,
               params(20),              // planner_loop,
               params.segment<3>(21),   // Vector3d left_foot_position,
               params.segment<3>(24));  // Vector3d right_foot_position
}

DcmReactiveStepper::InitializeCommand::InitializeCommand(
    DcmReactiveStepper &entity, const std::string &docstring)
    : dynamicgraph::command::Command(
          entity,
          boost::assign::list_of(dynamicgraph::command::Value::BOOL)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::VECTOR)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::DOUBLE)(
              dynamicgraph::command::Value::VECTOR)(
              dynamicgraph::command::Value::VECTOR),
          docstring)
{
}

dynamicgraph::command::Value DcmReactiveStepper::InitializeCommand::doExecute()
{
    DcmReactiveStepper &entity = static_cast<DcmReactiveStepper &>(owner());
    std::vector<dynamicgraph::command::Value> values = getParameterValues();

    const bool &is_left_leg_in_contact = values[0].value();
    const double &l_min = values[1].value();
    const double &l_max = values[2].value();
    const double &w_min = values[3].value();
    const double &w_max = values[4].value();
    const double &t_min = values[5].value();
    const double &t_max = values[6].value();
    const double &l_p = values[7].value();
    const double &ht = values[8].value();
    const Eigen::VectorXd &cost_weights_local = values[9].value();
    const double &mid_air_foot_height = values[10].value();
    const double &control_period = values[11].value();
    const double &planner_loop = values[12].value();
    const Eigen::VectorXd &left_foot_position = values[13].value();
    const Eigen::VectorXd &right_foot_position = values[14].value();

    if (cost_weights_local.size() != 9)
    {
        std::cout << "DcmReactiveStepper::InitializeCommand(): Error invalid "
                     "number of "
                     "weights, please retry.";
    }
    else
    {
        entity.initialize(is_left_leg_in_contact,
                          l_min,
                          l_max,
                          w_min,
                          w_max,
                          t_min,
                          t_max,
                          l_p,
                          ht,
                          cost_weights_local,
                          mid_air_foot_height,
                          control_period,
                          planner_loop,
                          left_foot_position,
                          right_foot_position);
    }
    return dynamicgraph::command::Value();
}

}  // namespace dynamic_graph
}  // namespace reactive_planners
