/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the QuadrupedDcmReactiveStepper class
 */

#include "reactive_planners/quadruped_dcm_reactive_stepper.hpp"

#include "pinocchio/math/rpy.hpp"

namespace reactive_planners
{
QuadrupedDcmReactiveStepper::QuadrupedDcmReactiveStepper()
{
    front_left_foot_position_.setZero();
    front_left_foot_velocity_.setZero();
    front_left_foot_acceleration_.setZero();
    front_right_foot_position_.setZero();
    front_right_foot_velocity_.setZero();
    front_right_foot_acceleration_.setZero();
    hind_left_foot_position_.setZero();
    hind_left_foot_velocity_.setZero();
    hind_left_foot_acceleration_.setZero();
    hind_right_foot_position_.setZero();
    hind_right_foot_velocity_.setZero();
    hind_right_foot_acceleration_.setZero();
    // biped_stepper_ nothing to be done.
    fr_offset_.setZero();
    fl_offset_.setZero();
    hr_offset_.setZero();
    hl_offset_.setZero();
    foot_height_offset_ = 0.0;
    forces_.setZero();
    contact_array_.fill(1);
    nb_force_ = 0;
    mid_air_foot_height_= 0.0;
}

QuadrupedDcmReactiveStepper::~QuadrupedDcmReactiveStepper() = default;

void QuadrupedDcmReactiveStepper::set_steptime_nominal(double t_nom)
{
    biped_stepper_.set_steptime_nominal(t_nom);
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
    mid_air_foot_height_ = mid_air_foot_height;

    Eigen::Map<const pinocchio::SE3::Quaternion> quat(
        base_placement.tail<4>().data());
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(quat.matrix());
    rpy.head<2>().setZero();
    Eigen::Matrix3d rot_mat = pinocchio::rpy::rpyToMatrix(rpy).transpose();
    fr_offset_ =
        rot_mat * (front_right_foot_position - base_placement.head<3>());
    fl_offset_ =
        rot_mat * (front_left_foot_position - base_placement.head<3>());
    hr_offset_ =
        rot_mat * (hind_right_foot_position - base_placement.head<3>());
    hl_offset_ = rot_mat * (hind_left_foot_position - base_placement.head<3>());

    foot_height_offset_ =
        (front_right_foot_position(2) + front_left_foot_position(2) +
         hind_right_foot_position(2) + hind_left_foot_position(2)) /
        4.0;

    fr_offset_(2) = 0.0;
    fl_offset_(2) = 0.0;
    hr_offset_(2) = 0.0;
    hl_offset_(2) = 0.0;

    Eigen::Vector3d virtual_left_foot_position =
        (front_left_foot_position + hind_right_foot_position) * 0.5;
    virtual_left_foot_position[2] = 0.0;

    Eigen::Vector3d virtual_right_foot_position =
        (front_right_foot_position + hind_left_foot_position) * 0.5;
    virtual_right_foot_position[2] = 0.0;

    // Initialize the dcm vrp planner and initialize it.
    biped_stepper_.initialize(is_left_leg_in_contact,
                              l_min,
                              l_max,
                              w_min,
                              w_max,
                              t_min,
                              t_max,
                              l_p,
                              com_height,
                              weight,
                              mid_air_foot_height_,
                              control_period,
                              planner_loop,
                              virtual_left_foot_position,
                              virtual_right_foot_position);

    fl_traj_.set_mid_air_height(mid_air_foot_height_);
    fl_traj_.set_planner_loop(planner_loop);
    fr_traj_.set_mid_air_height(mid_air_foot_height_);
    fr_traj_.set_planner_loop(planner_loop);
    hl_traj_.set_mid_air_height(mid_air_foot_height_);
    hl_traj_.set_planner_loop(planner_loop);
    hr_traj_.set_mid_air_height(mid_air_foot_height_);
    hr_traj_.set_planner_loop(planner_loop);

    front_left_forces_.resize(ceil(t_max * 1.0/control_period * 3.0));
    front_right_forces_.resize(ceil(t_max * 1.0/control_period * 3.0));
    hind_left_forces_.resize(ceil(t_max * 1.0/control_period * 3.0));
    hind_right_forces_.resize(ceil(t_max * 1.0/control_period * 3.0));
    front_left_forces_.setZero();
    front_right_forces_.setZero();
    hind_left_forces_.setZero();
    hind_right_forces_.setZero();

    front_left_foot_last_support_position_ = front_left_foot_position_;
    hind_right_foot_last_support_position_ = hind_right_foot_position_;
    front_right_foot_last_support_position_ = front_right_foot_position_;
    hind_left_foot_last_support_position_ = hind_left_foot_position_;
}

bool QuadrupedDcmReactiveStepper::run(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    const double& base_yaw,
    const bool& is_closed_loop)
{
    bool succeed = true;
    Eigen::Matrix<double, 12, 1> stepper_forces;

    // Compute Left and Right virtual foot positions.
    Eigen::Vector3d virtual_left_foot_position =
        (front_left_foot_position + hind_right_foot_position) * 0.5;
    virtual_left_foot_position(2) -= foot_height_offset_;
    Eigen::Vector3d virtual_left_foot_velocity =
        (front_left_foot_velocity + hind_right_foot_velocity) * 0.5;
    
    Eigen::Vector3d virtual_right_foot_position =
        (front_right_foot_position + hind_left_foot_position) * 0.5;
    virtual_right_foot_position(2) -= foot_height_offset_;
    Eigen::Vector3d virtual_right_foot_velocity =
        (front_right_foot_velocity + hind_left_foot_velocity) * 0.5;

    biped_stepper_.run(time,
                       virtual_left_foot_position,
                       virtual_right_foot_position,
                       virtual_left_foot_velocity,
                       virtual_right_foot_velocity,
                       com_position,
                       com_velocity,
                       base_yaw,
                       is_closed_loop);

    Eigen::Matrix3d base_yaw_rot =
        pinocchio::rpy::rpyToMatrix(0.0, 0.0, base_yaw);

    if(biped_stepper_.is_running())
    {
        double current_time = biped_stepper_.get_time_from_last_step_touchdown();
        double start_time = 0.0;
        double end_time = biped_stepper_.get_step_duration();
        // std::cout << "current_time = " << current_time << std::endl;
        // std::cout << "start_time = " << start_time << std::endl;
        // std::cout << "end_time = " << end_time << std::endl;
        bool is_left_leg_in_contact = 
            biped_stepper_.get_is_left_leg_in_contact();
        if(is_left_leg_in_contact)
        { // Right foot flying, Left foot support
            // Flying.
            // Front left.
            front_right_foot_last_support_position_(2) -= foot_height_offset_;
            front_right_foot_position_(2) -= foot_height_offset_;
            succeed = succeed && fr_traj_.compute(
                front_right_foot_last_support_position_,
                front_right_foot_position_,
                front_right_foot_velocity_,
                biped_stepper_.get_next_support_foot_position() +
                    base_yaw_rot * fr_offset_,
                start_time,
                current_time,
                end_time,
                true);
            nb_force_ = fr_traj_.get_forces(front_right_forces_,
                                            front_right_foot_position_,
                                            front_right_foot_velocity_,
                                            front_right_foot_acceleration_);
            front_right_foot_last_support_position_(2) += foot_height_offset_;
            front_right_foot_position_(2) += foot_height_offset_;
            
            // Hind right.
            hind_left_foot_last_support_position_(2) -= foot_height_offset_;
            hind_left_foot_position_(2) -= foot_height_offset_;
            succeed = succeed && hl_traj_.compute(
                hind_left_foot_last_support_position_,
                hind_left_foot_position_,
                hind_left_foot_velocity_,
                biped_stepper_.get_next_support_foot_position() +
                    base_yaw_rot * hl_offset_,
                start_time,
                current_time,
                end_time,
                true);
            nb_force_ = hl_traj_.get_forces(hind_left_forces_,
                                            hind_left_foot_position_,
                                            hind_left_foot_velocity_,
                                            hind_left_foot_acceleration_);
            hind_left_foot_last_support_position_(2) += foot_height_offset_;
            hind_left_foot_position_(2) += foot_height_offset_;
            // Support.
            front_left_forces_.setZero();
            front_left_foot_position_ = front_left_foot_position;
            front_left_foot_position_[2] = foot_height_offset_;
            front_left_foot_velocity_.fill(0.0);
            front_left_foot_acceleration_.fill(0.0);
            front_left_foot_last_support_position_ = front_left_foot_position_;
            hind_right_forces_.setZero();
            hind_right_foot_position_ = hind_right_foot_position;
            hind_right_foot_position_[2] = foot_height_offset_;
            hind_right_foot_velocity_.fill(0.0);
            hind_right_foot_acceleration_.fill(0.0);
            hind_right_foot_last_support_position_ = hind_right_foot_position_;
        }else{ // Left foot flying, Right foot support
            // Flying.
            // Front left.
            front_left_foot_last_support_position_(2) -= foot_height_offset_;
            front_left_foot_position_(2) -= foot_height_offset_;
            succeed = succeed && fl_traj_.compute(
                front_left_foot_last_support_position_,
                front_left_foot_position_,
                front_left_foot_velocity_,
                biped_stepper_.get_next_support_foot_position() +
                    base_yaw_rot * fl_offset_,
                start_time,
                current_time,
                end_time,
                true);
            nb_force_ = fl_traj_.get_forces(front_left_forces_,
                                            front_left_foot_position_,
                                            front_left_foot_velocity_,
                                            front_left_foot_acceleration_);
            front_left_foot_last_support_position_(2) += foot_height_offset_;
            front_left_foot_position_(2) += foot_height_offset_;
            // Hind right.
            hind_right_foot_last_support_position_(2) -= foot_height_offset_;
            hind_right_foot_position_(2) -= foot_height_offset_;
            succeed = succeed && hr_traj_.compute(
                hind_right_foot_last_support_position_,
                hind_right_foot_position_,
                hind_right_foot_velocity_,
                biped_stepper_.get_next_support_foot_position() +
                    base_yaw_rot * hr_offset_,
                start_time,
                current_time,
                end_time,
                true);
            nb_force_ = hr_traj_.get_forces(hind_right_forces_,
                                            hind_right_foot_position_,
                                            hind_right_foot_velocity_,
                                            hind_right_foot_acceleration_);
            hind_right_foot_last_support_position_(2) += foot_height_offset_;
            hind_right_foot_position_(2) += foot_height_offset_;
            // Support.
            front_right_forces_.setZero();
            front_right_foot_position_ = front_right_foot_position;
            front_right_foot_position_[2] = foot_height_offset_;
            front_right_foot_velocity_.fill(0.0);
            front_right_foot_acceleration_.fill(0.0);
            front_right_foot_last_support_position_ = front_right_foot_position_;
            hind_left_forces_.setZero();
            hind_left_foot_position_ = hind_left_foot_position;
            hind_left_foot_position_[2] = foot_height_offset_;
            hind_left_foot_velocity_.fill(0.0);
            hind_left_foot_acceleration_.fill(0.0);
            hind_left_foot_last_support_position_ = hind_left_foot_position_;
        }
    }else{
        foot_height_offset_ =
            (front_right_foot_position(2) + front_left_foot_position(2) +
            hind_right_foot_position(2) + hind_left_foot_position(2)) /
            4.0;
        // Support.
        front_left_foot_position_ = front_left_foot_position;
        front_left_foot_position_[2] = foot_height_offset_;
        front_left_foot_velocity_.fill(0.0);
        front_left_foot_acceleration_.fill(0.0);
        front_left_foot_last_support_position_ = front_left_foot_position_;
        hind_right_foot_position_ = hind_right_foot_position;
        hind_right_foot_position_[2] = foot_height_offset_;
        hind_right_foot_velocity_.fill(0.0);
        hind_right_foot_acceleration_.fill(0.0);
        hind_right_foot_last_support_position_ = hind_right_foot_position_;
        // Support.
        front_right_foot_position_ = front_right_foot_position;
        front_right_foot_position_[2] = foot_height_offset_;
        front_right_foot_velocity_.fill(0.0);
        front_right_foot_acceleration_.fill(0.0);
        front_right_foot_last_support_position_ = front_right_foot_position_;
        hind_left_foot_position_ = hind_left_foot_position;
        hind_left_foot_position_[2] = foot_height_offset_;
        hind_left_foot_velocity_.fill(0.0);
        hind_left_foot_acceleration_.fill(0.0);
        hind_left_foot_last_support_position_ = hind_left_foot_position_;
    }
    
    forces_.setZero();
    if (biped_stepper_.is_running())
    {
        if (biped_stepper_.get_is_left_leg_in_contact())
        {
            contact_array_ << 1.0, 0.0, 0.0, 1.0;
            forces_.segment<6>(1 * 6) << front_right_forces_.head<3>(),
                                         0.0, 0.0, 0.0;
            forces_.segment<6>(2 * 6) << hind_left_forces_.head<3>(),
                                         0.0, 0.0, 0.0;
        }
        else
        {
            contact_array_ << 0.0, 1.0, 1.0, 0.0;
            forces_.segment<6>(0 * 6) << front_left_forces_.head<3>(),
                                         0.0, 0.0, 0.0;
            forces_.segment<6>(3 * 6) << hind_right_forces_.head<3>(),
                                         0.0, 0.0, 0.0;
        }
    }
    else
    {
        contact_array_.fill(1.0);
        forces_.setZero();
    }
    forces_.setZero();
    return succeed;
}

bool QuadrupedDcmReactiveStepper::run(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    const pinocchio::SE3& world_M_base,
    const bool& is_closed_loop)
{
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(world_M_base.rotation());

    return run(time,
               front_left_foot_position,
               front_right_foot_position,
               hind_left_foot_position,
               hind_right_foot_position,
               front_left_foot_velocity,
               front_right_foot_velocity,
               hind_left_foot_velocity,
               hind_right_foot_velocity,
               com_position,
               com_velocity,
               rpy(2),
               is_closed_loop);
}

}  // namespace reactive_planners
