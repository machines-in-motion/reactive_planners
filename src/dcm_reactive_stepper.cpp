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
    running_ = false;
    local_frame_.setIdentity();
    nb_usage_of_force_ = 0.;
    flying_phase_duration_ = 0.2;
    stance_phase_duration_ = 0.1;
    omega_ = 10.18;//sqrt(9.81 / dcm_vrp_planner_.get_com_height());//Lhum TODO read it from python or DG.
    contact_(0) = is_left_leg_in_contact_;
    contact_(1) = !is_left_leg_in_contact_;
    des_swing_position_ << 0, 0, 0.1;
    b_ << 0, 0, 0.0;
    is_polynomial_ = false;
    com_0_ << 0., 0., 0.3;//Lhum TODO read it from python or DG.
    vcom << 0., 0., -0.9807185;//Lhum TODO read it from python or DG.
    com_.setZero();
    v_com_.setZero();
    a_com_.setZero();
    a_com_[0] = 5000;//non zero, TODO Lhum should change it!
    first_ = true;
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
    const double& t_s_nom,
    const Eigen::Vector10d& weight,
    const double& mid_air_foot_height,
    const double& control_period,
    const double& planner_loop,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& v_des)
{
    // Initialize the dcm vrp planner and initialize it.
    dcm_vrp_planner_.initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, omega_, t_s_nom, weight);

    // Parameters
    control_period_ = control_period;
    planner_loop_ = planner_loop;
    is_left_leg_in_contact_ = is_left_leg_in_contact;
    step_duration_ = t_s_nom;
    duration_of_flight_phase_ = 0.2;
    time_from_last_step_touchdown_ = 0.0;
    previous_support_foot_position_.setZero();
    current_support_foot_position_.setZero();
    next_support_foot_position_.setZero();
    desired_com_velocity_ = v_des;
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
    if (!is_polynomial_)
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
            dcm_vrp_planner_.get_z_0();
    else
        support_foot << right_foot_position(0), right_foot_position(1),
            dcm_vrp_planner_.get_z_0();
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
    local_frame_ = world_M_base;//Lhum thinks local_frame_ can be deleted
    bool succeed = true;
    if (running_ ||
        (!running_ && time_from_last_step_touchdown_ + control_period_ +
                              std::numeric_limits<double>::epsilon() <
                          step_duration_))//Lhum it should change for running!
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
        stepper_head_.run(step_duration_, duration_of_flight_phase_, com_velocity[2], com_position[2] + com_velocity[2] / omega_, right_on_ground, time);
    }
    else
    {
        stepper_head_.run(step_duration_, duration_of_flight_phase_, com_velocity[2], com_position[2] + com_velocity[2] / omega_, left_on_ground, time);
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
    if(contact_(0) != contact_(1)) {
        previous_support_foot_position_ = is_left_leg_in_contact_ ? right_foot_position : left_foot_position;
    }
    /// change solver loop time_step
    if (!is_polynomial_ && time_from_last_step_touchdown_ == 0.0) nb_usage_of_force_ = 0;
    int planner_frequency = round(planner_loop_ * 1000);
    if (!is_polynomial_ && nb_usage_of_force_ % planner_frequency != 0) //Lhum Change it if you want to change loop_period
    {  // Lhum TODO update 10 automatically
        if(contact_(0) == contact_(1)) {
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
            } else {
                // flying foot is the left foot
                dynamically_consistent_end_eff_trajectory_.update_robot_status(
                        left_foot_position_,
                        left_foot_velocity_,
                        left_foot_acceleration_);
            }
        }
        nb_usage_of_force_ += 1;
        return true;
    }
    nb_usage_of_force_ = 1;
    /// change solver loop time_step


    // Run the DcmVrpPlanner to get the next foot step location.
    double current_time = stepper_head_.get_time_from_last_step_touchdown();
    Eigen::Vector3d stance_pos = is_left_leg_in_contact_? left_foot_position:right_foot_position;

    //calculate com and vcom traj
    if(current_time < 0.01){
       if(omega_ == 10.18){
           stepper_head_.set_dcm_offset_nom(0.2);
           vcom << 0., 0., -0.9807185;
       }
       else if(omega_ == 5.7183913822){
           stepper_head_.set_dcm_offset_nom(0.3);
           vcom << 0., 0., -0.0;
       }
    }
    if(contact_(0) != contact_(1)) {
        com_planner_.update_com_single_support(omega_, current_time, stance_pos, com_0_, vcom);
        com_ = com_planner_.get_com();
        v_com_ = com_planner_.get_com_d();
        a_com_ = com_planner_.get_com_dd();
    }
    else{
        com_ = com_position;
        v_com_ = com_velocity;
        a_com_.setZero();
        first_ = false;
    }
    if(time_from_last_step_touchdown_<= dcm_vrp_planner_.get_duration_before_step_landing()){
        //At least one leg is in contact
        dcm_vrp_planner_.update(current_support_foot_position_,
                                time_from_last_step_touchdown_,
                                static_cast<contact>(is_left_leg_in_contact_),
                                desired_com_velocity_,
                                com_position,
                                com_velocity,
                                local_frame,
                                omega_);
        succeed = succeed && dcm_vrp_planner_.solve();

        last_support_foot_position_during_stance_ = current_support_foot_position_;
        last_com_position_during_stance_ = com_position;
        last_com_velocity_during_stance_ = com_velocity;
        last_local_frame_during_stance_ = local_frame;

    }
    else{
        // Flight phase
        Eigen::Vector3d projected_com_velocity = com_velocity;
        projected_com_velocity[2] += 9.81 * (time_from_last_step_touchdown_ - dcm_vrp_planner_.get_duration_before_step_landing());
        std::cout << "should be positive" << (time_from_last_step_touchdown_ - dcm_vrp_planner_.get_duration_before_step_landing()) << std::endl;
        Eigen::Vector3d projected_com = com_position - projected_com_velocity * (time_from_last_step_touchdown_ - dcm_vrp_planner_.get_duration_before_step_landing());
        projected_com[2] += 1.0 / 2 * 9.81 * pow(time_from_last_step_touchdown_ - dcm_vrp_planner_.get_duration_before_step_landing(), 2);
        dcm_vrp_planner_.update(last_support_foot_position_during_stance_ -last_support_foot_position_during_stance_,//- last_com_position_during_stance_ + projected_com,//
                                time_from_last_step_touchdown_,//
                                contact(is_left_leg_in_contact_ + 3),//
                                desired_com_velocity_,
                                projected_com,//
                                projected_com_velocity,
                                last_local_frame_during_stance_,//
                                omega_);//#
        succeed = succeed && dcm_vrp_planner_.solve();
    }

    // Extract the useful information.
    step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
    duration_of_flight_phase_ = dcm_vrp_planner_.get_duration_of_flight_phase();
//        next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
    dcm_offset_ = dcm_vrp_planner_.get_dcm_offset();
    std::cout << "TIME: " << time_from_last_step_touchdown_ << std::endl;
    next_support_foot_position_ = -dcm_offset_ + com_position + com_velocity / omega_;
    next_support_foot_position_[2] = 0;

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
        stepper_head_.run(0.0, -1.0, 0.1, 0.0, right_foot_position, time);
    }
    else
    {
        stepper_head_.run(0.0, -1.0, 0.1, 0.0, left_foot_position, time);
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