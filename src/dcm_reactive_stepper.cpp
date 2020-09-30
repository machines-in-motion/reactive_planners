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
    nb_usage_of_force_ = 0.;
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
                                    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
                                    Eigen::Ref<const Eigen::Vector3d> v_des)
{
    // Initialize the dcm vrp planner and initialize it.
    dcm_vrp_planner_.initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight);
    // Initialize the end-effector trajecotry generator.
    if(new_) {
        new_end_eff_traj3d_.set_mid_air_height(mid_air_foot_height);
        new_end_eff_traj3d_.init_calculate_dcm(v_des, com_height, l_p, t_min, t_max);
    }
    else{
        end_eff_traj3d_.set_mid_air_height(mid_air_foot_height);
    }

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
    l_p_ = l_p;
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
    Eigen::Ref<const Eigen::Vector3d> left_foot_vel,
    Eigen::Ref<const Eigen::Vector3d> right_foot_vel,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const double &base_yaw,
    Eigen::Ref<const Eigen::Vector2d> contact,
    const bool &is_closed_loop)
{
    Eigen::Vector3d support_foot;
    if(is_left_leg_in_contact_)
        support_foot << left_foot_position(0), left_foot_position(1), dcm_vrp_planner_.get_com_height();
    else
        support_foot << right_foot_position(0), right_foot_position(1), dcm_vrp_planner_.get_com_height();
    pinocchio::SE3 world_M_base(
        Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix(),
        support_foot);

    return run(time,
               left_foot_position,
               right_foot_position,
               left_foot_vel,
               right_foot_vel,
               com_position,
               com_velocity,
               world_M_base,
               contact,
               is_closed_loop);
}

bool DcmReactiveStepper::run(
    double time,
    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
    Eigen::Ref<const Eigen::Vector3d> right_foot_position,
    Eigen::Ref<const Eigen::Vector3d> left_foot_vel,
    Eigen::Ref<const Eigen::Vector3d> right_foot_vel,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    const pinocchio::SE3 &world_M_base,
    Eigen::Ref<const Eigen::Vector2d> contact,
    const bool &is_closed_loop)
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
//    std::cout << "Lhum run" << running_ << " " << time_from_last_step_touchdown_ + control_period_ +
//                              std::numeric_limits<double>::epsilon() << " " <<
//                          step_duration_ << std::endl;
    if (running_ ||
        (!running_ && time_from_last_step_touchdown_ + control_period_ +
                              std::numeric_limits<double>::epsilon() <
                          step_duration_))
    {
        walk(time,
             left_foot_position,
             right_foot_position,
             left_foot_vel,
             right_foot_vel,
             com_position,
             com_velocity,
             local_frame_,
             contact,
             is_closed_loop);
    }
    else
    {
//        std::cout << "stand_still\n"; // Lhum you should replace to left_on_ground for stand_still if you see this in output
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
    Eigen::Ref<const Eigen::Vector3d> left_foot_vel,
    Eigen::Ref<const Eigen::Vector3d> right_foot_vel,
    Eigen::Ref<const Eigen::Vector3d> com_position,
    Eigen::Ref<const Eigen::Vector3d> com_velocity,
    pinocchio::SE3 &local_frame,
    Eigen::Ref<const Eigen::Vector2d> contact,
    const bool &is_closed_loop)
{
    Eigen::Vector3d left_on_ground = left_foot_position;
    left_on_ground[2] = 0;
    Eigen::Vector3d right_on_ground = right_foot_position;
    right_on_ground[2] = 0;
    bool succeed = true;
    Eigen::Vector3d previous_next_support_foot_position_ = next_support_foot_position_;
    // Run the scheduler of the planner.
    if(contact[0] == 0 && contact[1] == 0){
        if (is_left_leg_in_contact_)
        {
            stepper_head_.run(step_duration_, right_on_ground, time);
        }
        else
        {
            stepper_head_.run(step_duration_, left_on_ground, time);
        }
    }
    else{
        if (is_left_leg_in_contact_)
        {
            stepper_head_.run(step_duration_, right_on_ground, time, contact[1]);
        }
        else
        {
            stepper_head_.run(step_duration_, left_on_ground, time, contact[0]);
        }
    }
    // Extract the usefull informations.
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();
    if(is_left_leg_in_contact_){
        stepper_head_.set_support_foot_pos(left_on_ground);
    }
    else{
        stepper_head_.set_support_foot_pos(right_on_ground);
    }
//    if(is_closed_loop){
//        if(is_left_leg_in_contact_){
////            std::cout << "right\n" << std::endl;
//            right_foot_position_ = right_foot_position;
//            right_foot_velocity_ = right_foot_vel;
//        }
//        else{
////            std::cout << "left\n" << std::endl;
//            left_foot_position_ = left_foot_position;
//            left_foot_velocity_ = left_foot_vel;
//
//        }
//    }
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    Eigen::Vector3d current_swing_foot_position = is_left_leg_in_contact_? right_foot_position_: left_foot_position_;
    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();



    ///change solver loop time_step
    if(new_ && time_from_last_step_touchdown_ == 0.0)
        nb_usage_of_force_ = 0;
    if(new_ && nb_usage_of_force_ % 10 != 0){//Lhum TODO update 10 automatically
        // Compute the flying foot trajectory.
        if (is_left_leg_in_contact_)  // check which foot is in contact
        {
            // flying foot is the right foot
            new_end_eff_traj3d_.update_robot_status(right_foot_position_,
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
            new_end_eff_traj3d_.update_robot_status(left_foot_position_,
                                                    left_foot_velocity_,
                                                    left_foot_acceleration_);
            // The current support foot does not move
            right_foot_position_ = current_support_foot_position_;
            right_foot_velocity_.setZero();
            right_foot_acceleration_.setZero();
        }
        nb_usage_of_force_ += 1;
        return true;
    }
    Eigen::Vector3d support_foot;
    if(is_left_leg_in_contact_)
        support_foot << left_on_ground(0), left_on_ground(1), dcm_vrp_planner_.get_com_height();
    else
        support_foot << right_on_ground(0), right_on_ground(1), dcm_vrp_planner_.get_com_height();
    local_frame.translation() = support_foot;
    nb_usage_of_force_ = 1;
    ///change solver loop time_step


    double previous_end_time = dcm_vrp_planner_.get_duration_before_step_landing();
    // Run the DcmVrpPlanner to get the next foot step location.
    double current_time = stepper_head_.get_time_from_last_step_touchdown();
    double new_t_min = 0.0;
    if(new_) {
//        if(is_left_leg_in_contact_)
//            new_t_min = new_end_eff_traj3d_.calculate_t_min(
//                    right_foot_position_,
//                    right_foot_velocity_,
//                    right_foot_acceleration_,
//                    current_time,
//                    is_left_leg_in_contact_);
//        else
//            new_t_min = new_end_eff_traj3d_.calculate_t_min(
//                    left_foot_position_,
//                    left_foot_velocity_,
//                    left_foot_acceleration_,
//                    current_time,
//                    is_left_leg_in_contact_);
        new_t_min = 0.;

    }
    dcm_vrp_planner_.update(current_support_foot_position_,
                            current_swing_foot_position,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            desired_com_velocity_,
                            com_position,
                            com_velocity,
                            local_frame,
                            new_t_min);

    succeed += succeed && dcm_vrp_planner_.solve(0, 0, 0);
    // Extract the usefull informations.
    step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
//    next_support_foot_position_(0) = 0;
//    next_support_foot_position_(1) = 0;
//    next_support_foot_position_(2) = 0;
    double start_time = 0.0;
    double end_time = dcm_vrp_planner_.get_duration_before_step_landing();// + step_local_duration_;////////////////////////////////

//    if(previous_end_time != end_time && current_time >= previous_end_time - 0.001){
//        //next_support_foot_position_ = previous_next_support_foot_position_;
//        next_support_foot_position_[2] = previous_next_support_foot_position_[2] - 0.0001;
//    }
//    std::cout << "Here" << std::endl;
    // Compute the flying foot trajectory.
    if (is_left_leg_in_contact_)  // check which foot is in contact
    {
        // flying foot is the right foot
//
//        ///////cost
//        double cost[1000];
//        std::cout << "Lhum cost: _______________________\n";
//        std::cout << time_from_last_step_touchdown_ << std::endl;
//        double minimum = 10000000;
//        bool Flag = false;
//        for(double i = std::max(ceil(current_time / 0.01) * 0.01 + 0.01, 0.1); i <= 0.2; i += 0.01){
//            dcm_vrp_planner_.add_t_eq(i);
//            dcm_vrp_planner_.solve();
//            end_eff_traj3d_.compute(previous_support_foot_position_,
//                                    right_foot_position_,
//                                    right_foot_velocity_,
//                                    right_foot_acceleration_,
//                                    next_support_foot_position_,
//                                    start_time,
//                                    current_time,
//                                    i,
//                                    com_position,
//                                    com_velocity,
//                                    current_support_foot_position_,
//                                    is_left_leg_in_contact_);
//            std::cout << "Lhum cost: " << time_from_last_step_touchdown_ << " " << i << "          " << dcm_vrp_planner_.cost() << " " <<
//                                          end_eff_traj3d_.cost() << " " << dcm_vrp_planner_.cost() + end_eff_traj3d_.cost() << " " << end_time<< std::endl;
//            std::cout << "Lhum cost " << i - current_time << " " << (i - current_time) / 0.005 << " @ " << "        " <<  int(ceil((i - current_time) / 0.005) - (ceil((i - start_time) / 0.005) / 2)) << std::endl;
////            std::cout.precision(17);
////            std::cout << "Lhum cost 2 " << i << " " << start_time << " " << (i - start_time) << " " << ((i - start_time) / 0.005) << " " << round((i - start_time + 0.000001) / 0.005) << "        " <<  (ceil((i - start_time) / 0.005) / 2) << std::endl;
//
//            if(int(round((i - current_time + 0.00498) / 0.005) - (round((i - start_time + 0.00498) / 0.005) / 2)) == 1){
//                Flag = false;
//                minimum = 10000000;
//            }
//            if(minimum < dcm_vrp_planner_.cost() + end_eff_traj3d_.cost()) {
//                Flag = true;
//            }
//            else {
//                if (Flag)
//                    std::cout << "Lhum cost ????????????????????????\n";
//                minimum = dcm_vrp_planner_.cost() + end_eff_traj3d_.cost();
//            }
//        }

        if(new_) {
            Eigen::Vector3d local_right_foot_position_ = right_foot_position_;
            Eigen::Vector3d local_right_foot_velocity_ = right_foot_velocity_;
            Eigen::Vector3d local_right_foot_acceleration_ = right_foot_acceleration_;
            succeed +=
                    succeed && new_end_eff_traj3d_.compute(previous_support_foot_position_,
                                                           local_right_foot_position_,
                                                           local_right_foot_velocity_,
                                                           local_right_foot_acceleration_,
                                                           next_support_foot_position_,
                                                           start_time,
                                                           current_time,
                                                           end_time,
                                                           com_position,
                                                           com_velocity,
                                                           current_support_foot_position_,
                                                           is_left_leg_in_contact_);
            nb_force_ = new_end_eff_traj3d_.get_forces(forces_,
                                                       local_right_foot_position_,
                                                       local_right_foot_velocity_,
                                                       local_right_foot_acceleration_);
            Eigen::Vector3d slack = new_end_eff_traj3d_.get_slack_variables();
//            if((fabs(slack(0)) > .01 || fabs(slack(1)) > 0.01)){
//                std::cout << BLUE << "slack 1\n " << std::endl;
//                std::cout << slack << RESET << std::endl;
//                std::cout << "Lhum second solve1 L " << next_support_foot_position_(0) + slack(0) << " " <<
//                          next_support_foot_position_(1) + slack(1) << " " << end_time << std::endl;
////                std::cout << "Lhum second input1" << previous_support_foot_position_ << " " <<
////                          right_foot_position_ << " " <<
////                          right_foot_velocity_ << " " <<
////                          right_foot_acceleration_ << " " <<
////                          next_support_foot_position_ << " " <<
////                          start_time << " " <<
////                          current_time << " " <<
////                          end_time << " " <<
////                          com_position << " " <<
////                          com_velocity << " " <<
////                          current_support_foot_position_ << " " <<
////                          is_left_leg_in_contact_ << std::endl;
//                std::cout << "__________\n";
//                dcm_vrp_planner_.solve(end_time, next_support_foot_position_(0) + slack(0),
//                                       next_support_foot_position_(1) + slack(1));
//                step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
//                next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
//                current_time = stepper_head_.get_time_from_last_step_touchdown();
//                end_time = dcm_vrp_planner_.get_duration_before_step_landing();
//                std::cout << "step_duration " << step_duration_ <<
//                             "\nnext_support " << previous_next_support_foot_position_ <<
//                             "\ncurrent_time " << current_time <<
//                             "\nnext_support_foot_position " << next_support_foot_position_ <<
//                             "\nend_time " << end_time << std::endl;
//                local_right_foot_position_ = right_foot_position_;
//                local_right_foot_velocity_ = right_foot_velocity_;
//                local_right_foot_acceleration_ = right_foot_acceleration_;
//
//                succeed +=
//                        succeed && new_end_eff_traj3d_.compute(previous_support_foot_position_,
//                                                               local_right_foot_position_,
//                                                               local_right_foot_velocity_,
//                                                               local_right_foot_acceleration_,
//                                                               next_support_foot_position_,
//                                                               start_time,
//                                                               current_time,
//                                                               end_time,
//                                                               com_position,
//                                                               com_velocity,
//                                                               current_support_foot_position_,
//                                                               is_left_leg_in_contact_);
//                std::cout << "Lhum compute" << std::endl;
//                nb_force_ = new_end_eff_traj3d_.get_forces(forces_,
//                                                           local_right_foot_position_,
//                                                           local_right_foot_velocity_,
//                                                           local_right_foot_acceleration_);
//                slack = new_end_eff_traj3d_.get_slack_variables();
//                std::cout << BLUE << "slack 2\n "  << std::endl;
//                std::cout << slack << RESET << std::endl;
//                std::cout << "_________________\n";
//                std::cout << "Lhum second solve1.5 " << next_support_foot_position_(0) + slack(0) << " " <<
//                          next_support_foot_position_(1) + slack(1) << " " << end_time << std::endl;
//
//
//
//
//                dcm_vrp_planner_.solve(end_time, next_support_foot_position_(0) + slack(0),
//                                       next_support_foot_position_(1) + slack(1));
//                step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
//                next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
//                current_time = stepper_head_.get_time_from_last_step_touchdown();
//                end_time = dcm_vrp_planner_.get_duration_before_step_landing();
//
//                std::cout << "step_duration " << step_duration_ <<
//                          "\nnext_support " << previous_next_support_foot_position_ <<
//                          "\ncurrent_time " << current_time <<
//                          "\nnext_support_foot_position " << next_support_foot_position_ <<
//                          "\nend_time " << end_time << std::endl;
//                local_right_foot_position_ = right_foot_position_;
//                local_right_foot_velocity_ = right_foot_velocity_;
//                local_right_foot_acceleration_ = right_foot_acceleration_;
//                succeed +=
//                        succeed && new_end_eff_traj3d_.compute(previous_support_foot_position_,
//                                                               local_right_foot_position_,
//                                                               local_right_foot_velocity_,
//                                                               local_right_foot_acceleration_,
//                                                               next_support_foot_position_,
//                                                               start_time,
//                                                               current_time,
//                                                               end_time,
//                                                               com_position,
//                                                               com_velocity,
//                                                               current_support_foot_position_,
//                                                               is_left_leg_in_contact_);
//                std::cout << "Lhum compute" << std::endl;
//                nb_force_ = new_end_eff_traj3d_.get_forces(forces_,
//                                                           local_right_foot_position_,
//                                                           local_right_foot_velocity_,
//                                                           local_right_foot_acceleration_);
//                slack = new_end_eff_traj3d_.get_slack_variables();
//                std::cout << BLUE << "slack 3\n "  << std::endl;
//                std::cout << slack << RESET << std::endl;
////                std::cout << "Lhum second input2" << previous_support_foot_position_ << " " <<
////                          right_foot_position_ << " " <<
////                          right_foot_velocity_ << " " <<
////                          right_foot_acceleration_ << " " <<
////                          next_support_foot_position_ << " " <<
////                          start_time << " " <<
////                          current_time << " " <<
////                          end_time << " " <<
////                          com_position << " " <<
////                          com_velocity << " " <<
////                          current_support_foot_position_ << " " <<
////                          is_left_leg_in_contact_ << std::endl;
//                slack = new_end_eff_traj3d_.get_slack_variables();
//                std::cout << "Lhum second solve2 " << next_support_foot_position_(0) + slack(0) << " " <<
//                          next_support_foot_position_(1) + slack(1) << " " << end_time << std::endl;
//                std::cout << "_________________________________________\n";
//            }
            right_foot_position_ = local_right_foot_position_;
            right_foot_velocity_ = local_right_foot_velocity_;
            right_foot_acceleration_ = local_right_foot_acceleration_;

        }
        else {
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
        }
        // The current support foot does not move
        left_foot_position_ = current_support_foot_position_;
        left_foot_velocity_.setZero();
        left_foot_acceleration_.setZero();
    }
    else{
        // flying foot is the left foot
//        ///////cost
//        double cost[1000];
//        std::cout << "Lhum cost: _______________________\n";
//        std::cout << time_from_last_step_touchdown_ << std::endl;
//        double minimum = 10000000;
//        bool Flag = false;
//        for(double i = std::max(ceil(current_time / 0.01) * 0.01 + 0.01, 0.1); i <= 0.2; i += 0.01){
//            dcm_vrp_planner_.add_t_eq(i);
//            dcm_vrp_planner_.solve();
//            end_eff_traj3d_.compute(previous_support_foot_position_,
//                                    left_foot_position_,
//                                    left_foot_velocity_,
//                                    left_foot_acceleration_,
//                                    next_support_foot_position_,
//                                    start_time,
//                                    current_time,
//                                    i,
//                                    com_position,
//                                    com_velocity,
//                                    current_support_foot_position_,
//                                    is_left_leg_in_contact_);
//            std::cout << "Lhum cost: " << time_from_last_step_touchdown_ << " " << i << "          " << dcm_vrp_planner_.cost() << " " <<
//                      end_eff_traj3d_.cost() << " " << dcm_vrp_planner_.cost() + end_eff_traj3d_.cost() << " " << end_time<< std::endl;
//            std::cout << "Lhum cost " << i - current_time << " " << (i - current_time) / 0.005 << " @ " << "        " <<  int(ceil((i - current_time) / 0.005) - (ceil((i - start_time) / 0.005) / 2)) << std::endl;
////            std::cout.precision(17);
////            std::cout << "Lhum cost 2 " << i << " " << start_time << " " <<  (i - start_time) << " " << ((i - start_time) / 0.005) << " " << round((i - start_time + 0.000001) / 0.005) << "        " <<  (ceil((i - start_time) / 0.005) / 2) << std::endl;
//            if(int(round((i - current_time + 0.00498) / 0.005) - (round((i - start_time + 0.00498) / 0.005) / 2)) == 1){
//                Flag = false;
//                minimum = 10000000;
//            }
//            if(minimum < dcm_vrp_planner_.cost() + end_eff_traj3d_.cost()) {
//                Flag = true;
//            }
//            else {
//                if (Flag)
//                    std::cout << "Lhum cost ????????????????????????\n";
//                minimum = dcm_vrp_planner_.cost() + end_eff_traj3d_.cost();
//            }
//        }
        if(new_) {
            Eigen::Vector3d local_left_foot_position_ = left_foot_position_;
            Eigen::Vector3d local_left_foot_velocity_ = left_foot_velocity_;
            Eigen::Vector3d local_left_foot_acceleration_ = left_foot_acceleration_;
            succeed +=
                    succeed && new_end_eff_traj3d_.compute(previous_support_foot_position_,
                                                           local_left_foot_position_,
                                                           local_left_foot_velocity_,
                                                           local_left_foot_acceleration_,
                                                           next_support_foot_position_,
                                                           start_time,
                                                           current_time,
                                                           end_time,
                                                           com_position,
                                                           com_velocity,
                                                           current_support_foot_position_,
                                                           is_left_leg_in_contact_);
            nb_force_ = new_end_eff_traj3d_.get_forces(forces_,
                                                       local_left_foot_position_,
                                                       local_left_foot_velocity_,
                                                       local_left_foot_acceleration_);
            Eigen::Vector3d slack = new_end_eff_traj3d_.get_slack_variables();
//            if(fabs(slack(0)) > .01 || fabs(slack(1)) > 0.01){
//                std::cout << BLUE << "slack 1\n "  << std::endl;
//                std::cout << slack << RESET << std::endl;
//                std::cout << "Lhum second solve1 R " << next_support_foot_position_(0) + slack(0) << " " <<
//                          next_support_foot_position_(1) + slack(1) << " " << end_time << std::endl;
////                std::cout << "Lhum second input1" << previous_support_foot_position_ << " " <<
////                          right_foot_position_ << " " <<
////                          right_foot_velocity_ << " " <<
////                          right_foot_acceleration_ << " " <<
////                          next_support_foot_position_ << " " <<
////                          start_time << " " <<
////                          current_time << " " <<
////                          end_time << " " <<
////                          com_position << " " <<
////                          com_velocity << " " <<
////                          current_support_foot_position_ << " " <<
////                          is_left_leg_in_contact_ << std::endl;
//                dcm_vrp_planner_.solve(end_time, next_support_foot_position_(0) + slack(0),
//                                       next_support_foot_position_(1) + slack(1));
//                step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
//                next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
//                current_time = stepper_head_.get_time_from_last_step_touchdown();
//                end_time = dcm_vrp_planner_.get_duration_before_step_landing();
//                std::cout << "step_duration " << step_duration_ <<
//                             "\nnext_support " << previous_next_support_foot_position_ <<
//                             "\ncurrent_time " << current_time <<
//                             "\nnext_support_foot_position " << next_support_foot_position_ <<
//                             "\nend_time " << end_time << std::endl;
//                local_left_foot_position_ = left_foot_position_;
//                local_left_foot_velocity_ = left_foot_velocity_;
//                local_left_foot_acceleration_ = left_foot_acceleration_;
//
//                succeed +=
//                        succeed && new_end_eff_traj3d_.compute(previous_support_foot_position_,
//                                                           local_left_foot_position_,
//                                                           local_left_foot_velocity_,
//                                                           local_left_foot_acceleration_,
//                                                           next_support_foot_position_,
//                                                           start_time,
//                                                           current_time,
//                                                           end_time,
//                                                           com_position,
//                                                           com_velocity,
//                                                           current_support_foot_position_,
//                                                           is_left_leg_in_contact_);
//                std::cout << "Lhum compute" << std::endl;
//                nb_force_ = new_end_eff_traj3d_.get_forces(forces_,
//                                                       local_left_foot_position_,
//                                                       local_left_foot_velocity_,
//                                                       local_left_foot_acceleration_);
//                slack = new_end_eff_traj3d_.get_slack_variables();
//                std::cout << BLUE << "slack 2\n "  << std::endl;
//                std::cout << slack << RESET << std::endl;
//                std::cout << "Lhum second solve1.5 " << next_support_foot_position_(0) + slack(0) << " " <<
//                          next_support_foot_position_(1) + slack(1) << " " << end_time << std::endl;
//
//
//
//
//                dcm_vrp_planner_.solve(end_time, next_support_foot_position_(0) + slack(0),
//                                       next_support_foot_position_(1) + slack(1));
//                step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
//                next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
//                current_time = stepper_head_.get_time_from_last_step_touchdown();
//                end_time = dcm_vrp_planner_.get_duration_before_step_landing();
//
//                std::cout << "step_duration " << step_duration_ <<
//                          "\nnext_support " << previous_next_support_foot_position_ <<
//                          "\ncurrent_time " << current_time <<
//                          "\nnext_support_foot_position " << next_support_foot_position_ <<
//                          "\nend_time " << end_time << std::endl;
//                local_left_foot_position_ = left_foot_position_;
//                local_left_foot_velocity_ = left_foot_velocity_;
//                local_left_foot_acceleration_ = left_foot_acceleration_;
//                succeed +=
//                        succeed && new_end_eff_traj3d_.compute(previous_support_foot_position_,
//                                                               local_left_foot_position_,
//                                                               local_left_foot_velocity_,
//                                                               local_left_foot_acceleration_,
//                                                               next_support_foot_position_,
//                                                               start_time,
//                                                               current_time,
//                                                               end_time,
//                                                               com_position,
//                                                               com_velocity,
//                                                               current_support_foot_position_,
//                                                               is_left_leg_in_contact_);
//                std::cout << "Lhum compute" << std::endl;
//                nb_force_ = new_end_eff_traj3d_.get_forces(forces_,
//                                                           local_left_foot_position_,
//                                                           local_left_foot_velocity_,
//                                                           local_left_foot_acceleration_);
//                slack = new_end_eff_traj3d_.get_slack_variables();
//                std::cout << BLUE << "slack 3\n "  << std::endl;
//                std::cout << slack << RESET << std::endl;
////                std::cout << "Lhum second input2" << previous_support_foot_position_ << " " <<
////                          right_foot_position_ << " " <<
////                          right_foot_velocity_ << " " <<
////                          right_foot_acceleration_ << " " <<
////                          next_support_foot_position_ << " " <<
////                          start_time << " " <<
////                          current_time << " " <<
////                          end_time << " " <<
////                          com_position << " " <<
////                          com_velocity << " " <<
////                          current_support_foot_position_ << " " <<
////                          is_left_leg_in_contact_ << std::endl;
//                slack = new_end_eff_traj3d_.get_slack_variables();
//                std::cout << "Lhum second solve2 " << next_support_foot_position_(0) + slack(0) << " " <<
//                          next_support_foot_position_(1) + slack(1) << " " << end_time << std::endl;
//                std::cout << "_________________________________________\n";
//            }
            left_foot_position_ = local_left_foot_position_;
            left_foot_velocity_ = local_left_foot_velocity_;
            left_foot_acceleration_ = local_left_foot_acceleration_;
        }
        else{
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
        }
        // The current support foot does not move
        right_foot_position_ = current_support_foot_position_;
        right_foot_velocity_.setZero();
        right_foot_acceleration_.setZero();
    }

    // Compute the feasible velocity.
    feasible_com_velocity_ =
        (next_support_foot_position_ - previous_support_foot_position_) * 0.5;
    feasible_com_velocity_[2] = 0.0;
    dcm_ = dcm_vrp_planner_.get_dcm_local();
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