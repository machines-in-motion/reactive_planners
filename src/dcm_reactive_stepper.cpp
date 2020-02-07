/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the DcmReactiveStepper class
 */

#include "reactive_planners/dcm_reactive_stepper.hpp"
#include <iostream>

namespace reactive_planners
{
DcmReactiveStepper::DcmReactiveStepper()
{
    // Parameters
    control_period_ = 0.0;
    is_left_leg_in_contact_ = false;
    step_duration_ = 0.0;
    time_from_last_step_touchdown_ = 0.0;
    previous_support_foot_position_.setZero();
    current_support_foot_position_.setZero();
    next_support_foot_position_.setZero();
    desired_com_velocity_.setZero();
    right_foot_position_.setZero();
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();
    left_foot_position_.setZero();
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    feasible_com_velocity_.setZero();
    running_ = false;
}

DcmReactiveStepper::~DcmReactiveStepper()
{
}

void DcmReactiveStepper::initialize(const bool &is_left_leg_in_contact,
                                    const double &l_min,
                                    const double &l_max,
                                    const double &w_min,
                                    const double &w_max,
                                    const double &t_min,
                                    const double &t_max,
                                    const double &l_p,
                                    const double &com_height,
                                    const Eigen::Vector9d &weight,
                                    const double &mid_air_foot_height,
                                    const double &control_period)
{
    // Initialize the dcm vrp planner and initialize it.
    dcm_vrp_planner_.initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight);
    // Initialize the end-effector trajecotry generator.
    end_eff_traj3d_.set_mid_air_height(mid_air_foot_height);

    // Parameters
    control_period_ = control_period;
    is_left_leg_in_contact_ = is_left_leg_in_contact;
    step_duration_ = 0.0;
    time_from_last_step_touchdown_ = 0.0;
    previous_support_foot_position_.setZero();
    current_support_foot_position_.setZero();
    next_support_foot_position_.setZero();
    desired_com_velocity_.setZero();
    right_foot_position_.setZero();
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();
    left_foot_position_.setZero();
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    feasible_com_velocity_.setZero();
    running_ = false;
}

bool DcmReactiveStepper::run(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const double &base_yaw)
{
    bool succeed = true;
    std::cout << "running_ = " << running_ << " ; "
              << "time_from_last_step_touchdown_ "
              << time_from_last_step_touchdown_ +
                     std::numeric_limits<double>::epsilon()
              << " ; "
              << "step_duration_ + epsilon = " << step_duration_ << " ; "
              << std::endl;
    if (running_ ||
        (!running_ && time_from_last_step_touchdown_ + control_period_ +
                              std::numeric_limits<double>::epsilon() <
                          step_duration_))
    {
        std::cout << "walking asked!" << std::endl;
        walk(time,
             left_foot_position,
             right_foot_position,
             com_position,
             com_velocity,
             base_yaw);
    }
    else
    {
        std::cout << "Standing still asked!" << std::endl;
        stand_still(time, left_foot_position, right_foot_position);
    }
    return succeed;
}

bool DcmReactiveStepper::walk(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const double &base_yaw)
{
    bool succeed = true;

    // Run the scheduler of the planner.
    if (is_left_leg_in_contact_)
    {
        stepper_head_.run(step_duration_, right_foot_position, time);
    }
    else
    {
        stepper_head_.run(step_duration_, left_foot_position, time);
    }
    // Extract the usefull informations.
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();

    // Run the DcmVrpPlanner to get the next foot step location.
    dcm_vrp_planner_.update(current_support_foot_position_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            desired_com_velocity_,
                            com_position,
                            com_velocity,
                            base_yaw);
    succeed += succeed && dcm_vrp_planner_.solve();
    // Extract the usefull informations.
    step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
    double start_time = 0.0;
    double current_time = stepper_head_.get_time_from_last_step_touchdown();
    double end_time = dcm_vrp_planner_.get_duration_before_step_landing();

    // Compute the flying foot trajectory.
    if (is_left_leg_in_contact_)  // check which foot is in contact
    {
        // flying foot is the right foot
        succeed +=
            succeed && end_eff_traj3d_.compute(previous_support_foot_position_,
                                               right_foot_position_,
                                               right_foot_velocity_,
                                               right_foot_acceleration_,
                                               next_support_foot_position_,
                                               start_time,
                                               current_time,
                                               end_time);

        end_eff_traj3d_.get_next_state(current_time + control_period_,
                                       right_foot_position_,
                                       right_foot_velocity_,
                                       right_foot_acceleration_);
        // The current support foot does not move
        left_foot_position_ = current_support_foot_position_;
        left_foot_velocity_.setZero();
        left_foot_acceleration_.setZero();
    }
    else
    {
        // flying foot is the left foot
        succeed +=
            succeed && end_eff_traj3d_.compute(previous_support_foot_position_,
                                               left_foot_position_,
                                               left_foot_velocity_,
                                               left_foot_acceleration_,
                                               next_support_foot_position_,
                                               start_time,
                                               current_time,
                                               end_time);

        end_eff_traj3d_.get_next_state(current_time + control_period_,
                                       left_foot_position_,
                                       left_foot_velocity_,
                                       left_foot_acceleration_);
        // The current support foot does not move
        right_foot_position_ = current_support_foot_position_;
        right_foot_velocity_.setZero();
        right_foot_acceleration_.setZero();
    }

    // Compute the feasible velocity.
    feasible_com_velocity_ =
        (next_support_foot_position_ - previous_support_foot_position_) * 0.5;
    feasible_com_velocity_[2] = 0.0;

    return succeed;
}

bool DcmReactiveStepper::stand_still(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position)
{
    bool succeed = true;

    // Run the scheduler of the planner.
    if (is_left_leg_in_contact_)
    {
        stepper_head_.run(0.0, right_foot_position, time);
    }
    else
    {
        stepper_head_.run(0.0, left_foot_position, time);
    }
    // Extract the usefull informations.
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();

    // Extract the usefull informations.
    step_duration_ = 0.0;
    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();

    // Feet do not move.
    left_foot_position_(2) = 0.0;
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    right_foot_position_(2) = 0.0;
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();

    // Compute the feasible velocity.
    feasible_com_velocity_.setZero();

    return succeed;
}

}  // namespace reactive_planners