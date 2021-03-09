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

    fr_offset_(2) = foot_height_offset_;
    fl_offset_(2) = foot_height_offset_;
    hr_offset_(2) = foot_height_offset_;
    hl_offset_(2) = foot_height_offset_;

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
                              mid_air_foot_height,
                              control_period,
                              planner_loop,
                              virtual_left_foot_position,
                              virtual_right_foot_position);
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
    front_left_foot_position_ =
        biped_stepper_.get_left_foot_position() + base_yaw_rot * fl_offset_;
    hind_right_foot_position_ =
        biped_stepper_.get_left_foot_position() + base_yaw_rot * hr_offset_;
    front_right_foot_position_ =
        biped_stepper_.get_right_foot_position() + base_yaw_rot * fr_offset_;
    hind_left_foot_position_ =
        biped_stepper_.get_right_foot_position() + base_yaw_rot * hl_offset_;

    front_left_foot_velocity_ = biped_stepper_.get_left_foot_velocity();
    hind_right_foot_velocity_ = biped_stepper_.get_left_foot_velocity();
    front_right_foot_velocity_ = biped_stepper_.get_right_foot_velocity();
    hind_left_foot_velocity_ = biped_stepper_.get_right_foot_velocity();

    front_left_foot_acceleration_ = biped_stepper_.get_left_foot_acceleration();
    hind_right_foot_acceleration_ = biped_stepper_.get_left_foot_acceleration();
    front_right_foot_acceleration_ =
        biped_stepper_.get_right_foot_acceleration();
    hind_left_foot_acceleration_ = biped_stepper_.get_right_foot_acceleration();

    forces_.setZero();
    if (biped_stepper_.is_running())
    {
        stepper_forces = biped_stepper_.get_force();
        if (biped_stepper_.get_is_left_leg_in_contact())
        {
            contact_array_ << 1.0, 0.0, 0.0, 1.0;
            forces_.block(6, 0, 6, 1) = stepper_forces.block(6, 0, 6, 1);
            forces_.block(12, 0, 6, 1) = stepper_forces.block(6, 0, 6, 1);
        }
        else
        {
            contact_array_ << 0.0, 1.0, 1.0, 0.0;
            forces_.block(0, 0, 6, 1) = stepper_forces.block(0, 0, 6, 1);
            forces_.block(18, 0, 6, 1) = stepper_forces.block(0, 0, 6, 1);
        }
    }
    else
    {
        contact_array_.fill(1.0);
    }
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
