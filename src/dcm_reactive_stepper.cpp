/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the DcmReactiveStepper class
 */

#include "reactive_planners/dcm_reactive_stepper.hpp"

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
    local_frame_.setIdentity();
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
                                    const double &control_period,
                                    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
                                    Eigen::Ref<const Eigen::Vector3d> right_foot_position)
{
    std::cout << is_left_leg_in_contact << " " <<
                 l_min << " " <<
                 l_max << " " <<
                 w_min << " " <<
                 w_max << " " <<
                 t_min << " " <<
                 t_max << " " <<
                 l_p << " " <<
                 com_height << " " <<
                 weight << " " <<
                 mid_air_foot_height << " " <<
                 control_period << " " <<
                 left_foot_position << " " <<
                 right_foot_position << std::endl;
    // Initialize the dcm vrp planner and initialize it.
    dcm_vrp_planner_.initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight);
    // Initialize the end-effector trajecotry generator.
    end_eff_traj3d_.set_mid_air_height(mid_air_foot_height);

    com_base_height_difference_ = 0.053;

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
    right_foot_position_ = right_foot_position;
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();
    left_foot_position_.setZero();
    left_foot_position_ = left_foot_position;
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    local_right_foot_position_.setZero();
    local_right_foot_velocity_.setZero();
    local_left_foot_position_.setZero();
    local_left_foot_velocity_.setZero();
    feasible_com_velocity_.setZero();
    if(is_left_leg_in_contact_){
        stepper_head_.set_support_feet_pos(right_foot_position, left_foot_position);
        current_support_foot_position_ = left_foot_position_;
        previous_support_foot_position_ = right_foot_position;
    }
    else{
        stepper_head_.set_support_feet_pos(left_foot_position, right_foot_position);
        current_support_foot_position_ = right_foot_position;
        previous_support_foot_position_ = left_foot_position_;
    }
    forces_.resize(ceil(t_max * 1000) * 3);
    running_ = false;
}

bool DcmReactiveStepper::run(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const double &base_yaw,
    Eigen::Ref<const Eigen::Vector2d> contact)
{
    Eigen::Vector3d base_pose;
    base_pose << 0, 0, dcm_vrp_planner_.get_com_height();
    pinocchio::SE3 world_M_base(
        Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix(),
        base_pose);

    return run(time,
               left_foot_position,
               right_foot_position,
               com_position,
               com_velocity,
               world_M_base,
               contact);
}

bool DcmReactiveStepper::run(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const pinocchio::SE3 &world_M_base,
    Eigen::Ref<const Eigen::Vector2d> contact)
{
    // if (first_iteration_)
    // {
    //     com_base_height_difference_ =
    //         world_M_base.translation()(2) - com_position(2);
    //     first_iteration_ = false;
    // }
    local_frame_ = world_M_base;
//    local_frame_.translation()(2) =
//        dcm_vrp_planner_.get_com_height() + com_base_height_difference_;
//    local_frame_.rotation() =
//        Eigen::AngleAxisd(world_M_base.rotation().eulerAngles(2, 1, 0)(2),
//                          Eigen::Vector3d::UnitZ());
    bool succeed = true;
    std::cout << "Lhum run" << running_ << " " << time_from_last_step_touchdown_ + control_period_ +
                              std::numeric_limits<double>::epsilon() << " " <<
                          step_duration_ << std::endl;
    if (running_ ||
        (!running_ && time_from_last_step_touchdown_ + control_period_ +
                              std::numeric_limits<double>::epsilon() <
                          step_duration_))
    {
        walk(time,
             left_foot_position,
             right_foot_position,
             com_position,
             com_velocity,
             local_frame_,
             contact);
    }
    else
    {
        stand_still(time, left_foot_position, right_foot_position);
    }
    // Convert the feet position, velocity and acceleration to the local frame.
    // Assumption: The velocity of the CoM is the velocity of the base.

//    local_right_foot_position_ =
//        right_foot_position_ - local_frame_.translation();
//    local_right_foot_velocity_ = right_foot_velocity_ - com_velocity;
//    local_left_foot_position_ =
//        left_foot_position_ - local_frame_.translation();
//    local_left_foot_velocity_ = left_foot_velocity_ - com_velocity;

    return succeed;
}

bool DcmReactiveStepper::walk(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const pinocchio::SE3 &local_frame,
    Eigen::Ref<const Eigen::Vector2d> contact)
{
    std::cout << "Lhum walk" << std::endl;
    bool succeed = true;
    double previous_end_time = dcm_vrp_planner_.get_duration_before_step_landing();
    Eigen::Vector3d previous_next_support_foot_position_ = next_support_foot_position_;//Lhum new changes
    // Run the scheduler of the planner.
    if(contact[0] == 0 && contact[1] == 0){
        if (is_left_leg_in_contact_)
        {
            stepper_head_.run(step_duration_, right_foot_position, time);
        }
        else
        {
            stepper_head_.run(step_duration_, left_foot_position, time);
        }
    }
    else{
        if (is_left_leg_in_contact_)
        {
            stepper_head_.run(step_duration_, right_foot_position, time, contact[1]);
        }
        else
        {
            stepper_head_.run(step_duration_, left_foot_position, time, contact[0]);
        }
    }
    // Extract the usefull informations.
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();
    if(is_left_leg_in_contact_){
        stepper_head_.set_support_foot_pos(left_foot_position);
    }
    else{
        stepper_head_.set_support_foot_pos(right_foot_position);
    }
    std::cout << "Lhum if" << std::endl;
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();

    // Run the DcmVrpPlanner to get the next foot step location.
    std::cout << "Lhum update" << std::endl;
    dcm_vrp_planner_.update(current_support_foot_position_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            desired_com_velocity_,
                            com_position,
                            com_velocity,
                            local_frame);
    std::cout << "Lhum update end" << std::endl;
    succeed += succeed && dcm_vrp_planner_.solve();
    // Extract the usefull informations.
    step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
    double start_time = 0.0;
    double current_time = stepper_head_.get_time_from_last_step_touchdown();
    double end_time = dcm_vrp_planner_.get_duration_before_step_landing();

    std::cout << "Lhum get" << std::endl;
    std::cout << std::setprecision(12) << previous_end_time << " " << end_time << " " << current_time << " " << previous_end_time << " "
              << (previous_end_time != end_time) << " " << (current_time == previous_end_time) << std::endl;
    if(previous_end_time != end_time && current_time >= previous_end_time - 0.001){
        //next_support_foot_position_ = previous_next_support_foot_position_;
        std::cout <<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << next_support_foot_position_ << "\n";
        next_support_foot_position_[2] = previous_next_support_foot_position_[2] - 0.0001;//Lhum new changes
    }
    std::cout << "Lhum nextSFP" << next_support_foot_position_ << std::endl;
    // Compute the flying foot trajectory.
    if (is_left_leg_in_contact_)  // check which foot is in contact
    {
        // flying foot is the right foot
        std::cout << "Lhum right %%%%%%% " << right_foot_position_ << "    " <<
                     right_foot_velocity_ << "   " <<
                     right_foot_acceleration_<< std::endl;

        std::cout << "Lhum vel %%%%%%% " << right_foot_velocity_ << std::endl;
        succeed +=
            succeed && end_eff_traj3d_.compute(previous_support_foot_position_,
                                               right_foot_position_,
                                               right_foot_velocity_,
                                               right_foot_acceleration_,
                                               next_support_foot_position_,
                                               start_time,
                                               current_time,
                                               end_time);
        std::cout << "Lhum compute" << std::endl;
        nb_force_ = end_eff_traj3d_.get_forces(forces_,
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
        std::cout << "Lhum left %%%%%%% " << left_foot_position_ << "    " <<
                     left_foot_velocity_ << "   " <<
                     left_foot_acceleration_<< std::endl;

        std::cout << "Lhum vel %%%%%%% " << left_foot_velocity_ << std::endl;
        succeed +=
            succeed && end_eff_traj3d_.compute(previous_support_foot_position_,
                                               left_foot_position_,
                                               left_foot_velocity_,
                                               left_foot_acceleration_,
                                               next_support_foot_position_,
                                               start_time,
                                               current_time,
                                               end_time);
        nb_force_ = end_eff_traj3d_.get_forces(forces_,
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