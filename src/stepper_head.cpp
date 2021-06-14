/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Definition of the StepperHead class.
 */

#include "reactive_planners/stepper_head.hpp"
#include<iostream>

namespace reactive_planners
{
StepperHead::StepperHead()
{
    // inputs
    duration_stance_phase_ = 0.0;
    duration_before_foot_landing_ = 0.0;
    next_support_location_.setZero();
    current_time_ = 0.0;
    contact_<< 1, 1;

    // outputs
    time_from_last_step_touchdown_ = 0.0;
    is_left_leg_in_contact_ = true;
    previous_support_location_.setZero();
    current_support_location_.setZero();

    // internals
    time_support_switch_ = 0.0;
}

void StepperHead::run(const double& duration_stance_phase,
                      const double& v_z,
                      const double& kesay,
                      const Eigen::Vector3d& next_support_location,
                      const double& current_time)
{
    // copy the argument
    duration_stance_phase_ = duration_stance_phase;
    next_support_location_ = next_support_location;
    current_time_ = current_time;

    // Compute the time_from_last_step_touchdown_
    time_from_last_step_touchdown_ = current_time_ - time_support_switch_;
    contact_ << is_left_leg_in_contact_, !is_left_leg_in_contact_;
//    std::cout << time_from_last_step_touchdown_ << "   " << duration_stance_phase_ << "    " << duration_swing_phase << std::endl;
    std::cout << "UPDATE  stepper    " << time_from_last_step_touchdown_ << "    " << duration_stance_phase_ << std::endl;
    if(time_from_last_step_touchdown_ > duration_stance_phase_){
        contact_<< 0, 0;
    }
    std::cout << "UPDATE switch " << time_from_last_step_touchdown_ << " > " << duration_stance_phase_ << "     0 >=" << v_z << "      0.2 >=" << kesay << std::endl;
    if (time_from_last_step_touchdown_ > duration_stance_phase_ && v_z <= 0 && kesay <= 0.2)
    {
        // Switch the contact phase.
        is_left_leg_in_contact_ = !is_left_leg_in_contact_;
        time_support_switch_ = current_time;
        time_from_last_step_touchdown_ = 0.0;
        contact_<< is_left_leg_in_contact_, !is_left_leg_in_contact_;

        previous_support_location_ = current_support_location_;
        current_support_location_ = next_support_location_;
    }
}

}  // namespace reactive_planners