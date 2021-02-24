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
    nb_force_ = 0;
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
    x_T_s_.setZero();
    x_d_T_s_.setZero();
    running_ = false;
    local_frame_.setIdentity();
    nb_usage_of_force_ = 0.;
    flying_phase_duration_ = 0.1;
    stance_phase_duration_ = 0.2;
    omega_ = 7.2;//sqrt(9.81 / dcm_vrp_planner_.get_com_height());//Lhum TODO read it from python or DG.
    contact_(0) = is_left_leg_in_contact_;
    contact_(1) = !is_left_leg_in_contact_;
    des_swing_position_ << 0, 0, 0.05;
}

DcmReactiveStepper::~DcmReactiveStepper() = default;

void DcmReactiveStepper::initialize(
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
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& v_des)
{
    // Initialize the dcm vrp planner and initialize it.
    dcm_vrp_planner_.initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, flying_phase_duration_, omega_, weight);

    // Parameters
    control_period_ = control_period;
    planner_loop_ = planner_loop;
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

    // Initialize the end-effector trajectory generator.
    if (new_)
    {
        dynamically_consistent_end_eff_trajectory_.set_mid_air_height(
            mid_air_foot_height);
        dynamically_consistent_end_eff_trajectory_.set_planner_loop(
            planner_loop_);
    }
    else
    {
        polynomial_end_eff_trajectory_.set_mid_air_height(mid_air_foot_height);
    }
    if (is_left_leg_in_contact_)
    {
        stepper_head_.set_support_feet_pos(right_foot_position,
                                           left_foot_position);
        current_support_foot_position_ = left_foot_position_;
        previous_support_foot_position_ = right_foot_position;
    }
    else
    {
        stepper_head_.set_support_feet_pos(left_foot_position,
                                           right_foot_position);
        current_support_foot_position_ = right_foot_position;
        previous_support_foot_position_ = left_foot_position_;
    }
    forces_.resize(ceil((t_max + 1.) * 1000) * 3);
    f_swing_.resize(ceil((t_max + 1.) * 1000) * 3);
    running_ = false;
}

bool DcmReactiveStepper::run(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    const double& base_yaw,
    const bool& is_closed_loop)
{
    Eigen::Vector3d support_foot;
    if (is_left_leg_in_contact_)
        support_foot << left_foot_position(0), left_foot_position(1),
            dcm_vrp_planner_.get_com_height();
    else
        support_foot << right_foot_position(0), right_foot_position(1),
            dcm_vrp_planner_.get_com_height();
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
               is_closed_loop);
}

bool DcmReactiveStepper::run(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    const pinocchio::SE3& world_M_base,
    const bool& is_closed_loop)
{
    local_frame_ = world_M_base;
    bool succeed = true;
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
             is_closed_loop);
    }
    else
    {
        stand_still(time, left_foot_position, right_foot_position);
    }
    return succeed;
}

bool DcmReactiveStepper::walk(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    pinocchio::SE3& local_frame,
    const bool& is_closed_loop)
{
    Eigen::Vector3d left_on_ground = left_foot_position;
    left_on_ground[2] = 0;
    Eigen::Vector3d right_on_ground = right_foot_position;
    right_on_ground[2] = 0;
    bool succeed = true;
    // Run the scheduler of the planner.
    if (is_left_leg_in_contact_)
    {
        stepper_head_.run(step_duration_ - flying_phase_duration_, flying_phase_duration_, right_on_ground, time);
    }
    else
    {
        stepper_head_.run(step_duration_ - flying_phase_duration_, flying_phase_duration_, left_on_ground, time);
    }
    contact_ = stepper_head_.get_contact_phase();

    // Extract the useful information.
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();
    if (is_left_leg_in_contact_)
    {
        stepper_head_.set_support_foot_pos(left_on_ground);
    }
    else
    {
        stepper_head_.set_support_foot_pos(right_on_ground);
    }
    if (is_closed_loop)
    {
        if (is_left_leg_in_contact_)
        {
            right_foot_position_ = right_foot_position;
            right_foot_velocity_ = right_foot_vel;
        }
        else
        {
            left_foot_position_ = left_foot_position;
            left_foot_velocity_ = left_foot_vel;
        }
    }
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    Eigen::Vector3d current_swing_foot_position =
        is_left_leg_in_contact_ ? right_foot_position_ : left_foot_position_;
    if(contact_(0) != contact_(1))
        previous_support_foot_position_ = stepper_head_.get_previous_support_location();

    /// change solver loop time_step
    if (new_ && time_from_last_step_touchdown_ == 0.0) nb_usage_of_force_ = 0;
    if (new_ && nb_usage_of_force_ % 10 != 0)
    {  // Lhum TODO update 10 automatically
        if(contact_(0) != contact_(1)) {
            if (is_left_leg_in_contact_)  // check which foot is in contact
            {
                // flying foot is the right foot
                dynamically_consistent_end_eff_trajectory_.update_robot_status(right_foot_position_,
                                                        right_foot_velocity_,
                                                        right_foot_acceleration_);
                // The current support foot does not move
                left_foot_position_ = current_support_foot_position_;
                left_foot_velocity_.setZero();
                left_foot_acceleration_.setZero();
            } else {
                // flying foot is the left foot
                dynamically_consistent_end_eff_trajectory_.update_robot_status(left_foot_position_,
                                                        left_foot_velocity_,
                                                        left_foot_acceleration_);
                // The current support foot does not move
                right_foot_position_ = current_support_foot_position_;
                right_foot_velocity_.setZero();
                right_foot_acceleration_.setZero();
            }
        }
        else {
            // Compute the flying foot trajectory.
            if (is_left_leg_in_contact_)  // check which foot is in contact
            {
                // flying foot is the right foot
                dynamically_consistent_end_eff_trajectory_.update_robot_status(
                        right_foot_position_,
                        right_foot_velocity_,
                        right_foot_acceleration_);
                // The current support foot does not move
//                left_foot_position_ = current_support_foot_position_;
//                left_foot_velocity_.setZero();
//                left_foot_acceleration_.setZero();
            } else {
                // flying foot is the left foot
                dynamically_consistent_end_eff_trajectory_.update_robot_status(
                        left_foot_position_,
                        left_foot_velocity_,
                        left_foot_acceleration_);
                // The current support foot does not move
//                right_foot_position_ = current_support_foot_position_;
//                right_foot_velocity_.setZero();
//                right_foot_acceleration_.setZero();
            }
        }
        nb_usage_of_force_ += 1;
        return true;
    }
    Eigen::Vector3d support_foot;
    if (is_left_leg_in_contact_)
        support_foot << left_on_ground(0), left_on_ground(1),
            dcm_vrp_planner_.get_com_height();
    else
        support_foot << right_on_ground(0), right_on_ground(1),
            dcm_vrp_planner_.get_com_height();
    local_frame.translation() = support_foot;
    nb_usage_of_force_ = 1;
    /// change solver loop time_step


    double previous_end_time = dcm_vrp_planner_.get_duration_before_step_landing();
    // Run the DcmVrpPlanner to get the next foot step location.
    double current_time = stepper_head_.get_time_from_last_step_touchdown();
    double new_t_min = 0.0;
    if (new_ && false) {//Lhum jump delete false when you want normal walking
        if (is_left_leg_in_contact_)
            new_t_min =
                dynamically_consistent_end_eff_trajectory_.calculate_t_min(
                    right_foot_position_,
                    right_foot_velocity_,
                    is_left_leg_in_contact_);
        else
            new_t_min =
                dynamically_consistent_end_eff_trajectory_.calculate_t_min(
                    left_foot_position_,
                    left_foot_velocity_,
                    is_left_leg_in_contact_);
    }

    Eigen::Vector3d stance_pos = is_left_leg_in_contact_? left_foot_position_:right_foot_position_;
    if(current_time < stance_phase_duration_) {
        com_planner_.update_com_mea_single_support(omega_, stance_phase_duration_ - current_time,
                                                   stance_pos, com_position, com_velocity);
        x_T_s_ = com_planner_.get_com_mea();
        x_d_T_s_ = com_planner_.get_com_d_mea();
    }

    dcm_vrp_planner_.update(current_support_foot_position_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            desired_com_velocity_,
                            com_position,
                            com_velocity,
                            local_frame,
                            new_t_min,
                            omega_,
                            x_T_s_,
                            x_d_T_s_);

    succeed = succeed && dcm_vrp_planner_.solve();
    // Extract the useful information.
    step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();

    double start_time = 0.0;
    double end_time = dcm_vrp_planner_.get_duration_before_step_landing();

    //calculate com and vcom traj
    Eigen::Vector3d com;
    Eigen::Vector3d vcom;
    com << 0., 0., dcm_vrp_planner_.get_com_height();//Lhum TODO read it from python or DG.
    vcom << 0., 0., -0.491985;//Lhum TODO read it from python or DG.
    if(contact_(0) != contact_(1)) {
        std::cout << "stance phase\n";
        com_planner_.update_com_single_support(omega_, current_time, stance_pos, com, vcom);
        com_ = com_planner_.get_com();
        v_com_ = com_planner_.get_com_d();
//        std::cout << "v_com " << v_com_ << std::endl;
    }
    else{
        std::cout << "swing phase\n";
        com_ = com_position;
        v_com_ = com_velocity;
//        std::cout << "Other v_com " << v_com_ << std::endl;
    }

    // Compute the flying foot trajectory.
    if(contact_(0) != contact_(1)) {
        if (is_left_leg_in_contact_)  // check which foot is in contact
        {
            // flying foot is the right foot
            if (new_)
            {
                succeed =
                    succeed && dynamically_consistent_end_eff_trajectory_.compute(
                                   previous_support_foot_position_,
                                   right_foot_position_,
                                   right_foot_velocity_,
                                   next_support_foot_position_ + des_swing_position_,
                                   start_time,
                                   current_time,
                                   end_time - flying_phase_duration_,
                                   is_left_leg_in_contact_);
                nb_force_ = dynamically_consistent_end_eff_trajectory_.get_forces(
                    forces_,
                    right_foot_position_,
                    right_foot_velocity_,
                    right_foot_acceleration_);
                Eigen::Vector3d slack = dynamically_consistent_end_eff_trajectory_
                                            .get_slack_variables();
            }
            else
            {
                succeed = succeed && polynomial_end_eff_trajectory_.compute(
                                         previous_support_foot_position_,
                                         right_foot_position_,
                                         right_foot_velocity_,
                                         right_foot_acceleration_,
                                         next_support_foot_position_,
                                         start_time,
                                         current_time,
                                         end_time);
                polynomial_end_eff_trajectory_.get_next_state(
                    current_time + control_period_,
                    right_foot_position_,
                    right_foot_velocity_,
                    right_foot_acceleration_);
                // The current support foot does not move
                left_foot_position_ = current_support_foot_position_;
                left_foot_velocity_.setZero();
                left_foot_acceleration_.setZero();
            }
        }
        else
        {
            // flying foot is the left foot
            if (new_)
            {
                succeed =
                    succeed && dynamically_consistent_end_eff_trajectory_.compute(
                                   previous_support_foot_position_,
                                   left_foot_position_,
                                   left_foot_velocity_,
                                   next_support_foot_position_ + des_swing_position_,
                                   start_time,
                                   current_time,
                                   end_time - flying_phase_duration_,
                                   is_left_leg_in_contact_);
                nb_force_ = dynamically_consistent_end_eff_trajectory_.get_forces(
                    forces_,
                    left_foot_position_,
                    left_foot_velocity_,
                    left_foot_acceleration_);
                Eigen::Vector3d slack = dynamically_consistent_end_eff_trajectory_
                                            .get_slack_variables();
            }
            else
            {
                succeed = succeed && polynomial_end_eff_trajectory_.compute(
                                         previous_support_foot_position_,
                                         left_foot_position_,
                                         left_foot_velocity_,
                                         left_foot_acceleration_,
                                         next_support_foot_position_,
                                         start_time,
                                         current_time,
                                         end_time);
                polynomial_end_eff_trajectory_.get_next_state(
                    current_time + control_period_,
                    left_foot_position_,
                    left_foot_velocity_,
                    left_foot_acceleration_);

                // The current support foot does not move
                right_foot_position_ = current_support_foot_position_;
                right_foot_velocity_.setZero();
                right_foot_acceleration_.setZero();
            }
        }
    }
    else{
        forces_.setZero();
        nb_force_ = 3;
        left_foot_position_ = current_support_foot_position_;
        left_foot_velocity_.setZero();
        left_foot_acceleration_.setZero();
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
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position)
{
    bool succeed = true;

    // Run the scheduler of the planner.
    if (is_left_leg_in_contact_)
    {
        stepper_head_.run(0.0, 0.0, right_foot_position, time);
    }
    else
    {
        stepper_head_.run(0.0, 0.0, left_foot_position, time);
    }
    // Extract the useful information.
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();

    // Extract the useful information.
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