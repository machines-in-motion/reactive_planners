/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Biped robots reactive stepper using the divergent component of motion.
 *
 * Declare a class that encapsulate the DcmVrpPlanner, the
 * EndEffectorTrajectory3D, and the StepperHead.
 */

#pragma once

#include <iostream>

#include "reactive_planners/dcm_reactive_stepper.hpp"

namespace Eigen
{
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 24, 1> Vector24d;
}

namespace reactive_planners
{
class QuadrupedDcmReactiveStepper
{
public:
    QuadrupedDcmReactiveStepper();

    ~QuadrupedDcmReactiveStepper();

    /**
     * @brief Initialize the reactive stepper for quadruped. We assume that the
     * base is horizontal and that the feet are touching the ground.
     *
     * @param is_left_leg_in_contact Left foot is in contact with the ground, if
     * not then the right foot is.
     * @param l_min Lower bound in the forward direction where to step.
     * @param l_max Upper bound in the forward direction where to step.
     * @param w_min Lower bound in the lateral direction where to step.
     * @param w_max Upper bound in the lateral direction where to step.
     * @param t_min The minimum time required to step.
     * @param t_max The maximum time required to step.
     * @param l_p The nominal stride length.
     * @param com_height Center of mass height from the ground.
     * @param weight Total weight of the robot.
     * @param mid_air_foot_height Maximum height of the foot in mid-air.
     * @param control_period Robot control period.
     * @param planner_loop Period of the end-effector trajectory update..
     * @param front_left_foot_position 3D position of the front left foot.
     * @param front_right_foot_position 3D position of the front right foot.
     * @param hind_left_foot_position 3D position of the hind left foot.
     * @param hind_right_foot_position 3D position of the hind right foot.
     */
    void initialize(
        const bool &is_left_leg_in_contact,
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
        const double &planner_loop,
        const Eigen::Ref<const Eigen::Vector7d> &base_placement,
        const Eigen::Ref<const Eigen::Vector3d> &front_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &front_right_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &hind_right_foot_position);

    /**
     * @brief Set the nominal steptime.
     */
    void set_steptime_nominal(double t_nom);

    /**
     * @brief Compute the plan trajectory from input variable.
     *
     * @param time Duration from the beginning of the experiment
     * @param front_left_foot_position 3D front left foot position.
     * @param front_right_foot_position 3D front right foot position.
     * @param hind_left_foot_position 3D hind left foot position.
     * @param hind_right_foot_position 3D hind right foot position.
     * @param front_left_foot_velocity 3D front left foot velocity.
     * @param front_right_foot_velocity 3D front right foot velocity.
     * @param hind_left_foot_velocity 3D hind left foot velocity.
     * @param hind_right_foot_velocity 3D hind right foot velocity.
     * @param com_position Center of Mass position.
     * @param com_velocity Center of mass position.
     * @param base_yaw Current orientation of the base.
     * @param is_closed_loop ????
     * @return bool, true upon success.
     */
    bool run(double time,
             const Eigen::Ref<const Eigen::Vector3d> &front_left_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &front_right_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &hind_right_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &front_left_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &front_right_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &hind_right_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &com_position,
             const Eigen::Ref<const Eigen::Vector3d> &com_velocity,
             const double &base_yaw,
             const bool &is_closed_loop);
    /**
     * @brief Compute the plan trajectory from input variable.
     *
     * @param time Duration from the beginning of the experiment
     * @param left_foot_position 3D left foot position.
     * @param right_foot_position 3D right foot position.
     * @param left_foot_vel 3D left foot velocity.
     * @param right_foot_vel 3D right foot velocity.
     * @param com_position Center of Mass position.
     * @param com_velocity Center of mass position.
     * @param world_M_base SE3 placement of the base frame with respect to
     *                     the world frame.
     * @param is_closed_loop  ???
     * @return bool, true upon success.
     */
    bool run(double time,
             const Eigen::Ref<const Eigen::Vector3d> &front_left_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &front_right_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &hind_right_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &front_left_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &front_right_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &hind_right_foot_velocity,
             const Eigen::Ref<const Eigen::Vector3d> &com_position,
             const Eigen::Ref<const Eigen::Vector3d> &com_velocity,
             const pinocchio::SE3 &world_M_base,
             const bool &is_closed_loop);

    /**
     * @brief Start the stepping.
     */
    void start()
    {
        biped_stepper_.start();
    }

    /**
     * @brief Stop the stepping.
     */
    void stop()
    {
        biped_stepper_.stop();
    }

    /*
     * Setters
     */

    /**
     * @brief Set the desired center of mass velocity.
     *
     * @param desired_com_velocity
     */
    void set_desired_com_velocity(
        Eigen::Ref<const Eigen::Vector3d> desired_com_velocity)
    {
        biped_stepper_.set_desired_com_velocity(desired_com_velocity);
    }

    void set_feet_pos(
        const Eigen::Ref<const Eigen::Vector3d> &front_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &front_right_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &hind_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &hind_right_foot_position)
    {
        front_left_foot_position_ = front_left_foot_position;
        front_right_foot_position_ = front_right_foot_position;
        hind_left_foot_position_ = hind_left_foot_position;
        hind_right_foot_position_ = hind_right_foot_position;
    }

    /**
     * @brief Set polynomial end effector trajectory.
     */
    void set_polynomial_end_effector_trajectory()
    {
        biped_stepper_.set_polynomial_end_effector_trajectory();
    }

    /**
     * @brief Set dynamical end effector trajectory.
     */
    void set_dynamical_end_effector_trajectory()
    {
        biped_stepper_.set_dynamical_end_effector_trajectory();
    }

    /*
     * Getters
     */

    /* front left foot */

    /**
     * @brief Get the front left foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_front_left_foot_position()
    {
        return front_left_foot_position_;
    }

    /**
     * @brief Get the front left foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_front_left_foot_velocity()
    {
        return front_left_foot_velocity_;
    }

    /**
     * @brief Get the front left foot 3d acceleration.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_front_left_foot_acceleration()
    {
        return front_left_foot_acceleration_;
    }

    /* front right foot */

    /**
     * @brief Get the front right foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_front_right_foot_position()
    {
        return front_right_foot_position_;
    }

    /**
     * @brief Get the front right foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_front_right_foot_velocity()
    {
        return front_right_foot_velocity_;
    }

    /**
     * @brief Get the front right foot 3d acceleration.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_front_right_foot_acceleration()
    {
        return front_right_foot_acceleration_;
    }

    /* hind left foot */

    /**
     * @brief Get the hind left foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_hind_left_foot_position()
    {
        return hind_left_foot_position_;
    }

    /**
     * @brief Get the hind left foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_hind_left_foot_velocity()
    {
        return hind_left_foot_velocity_;
    }

    /**
     * @brief Get the hind left foot 3d acceleration.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_hind_left_foot_acceleration()
    {
        return hind_left_foot_acceleration_;
    }

    /* hind right foot */

    /**
     * @brief Get the hind right foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_hind_right_foot_position()
    {
        return hind_right_foot_position_;
    }

    /**
     * @brief Get the hind right foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_hind_right_foot_velocity()
    {
        return hind_right_foot_velocity_;
    }

    /**
     * @brief Get the hind right foot 3d acceleration.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_hind_right_foot_acceleration()
    {
        return hind_right_foot_acceleration_;
    }

    /* hind right foot */

    /**
     * @brief Get the feasible velocity computed from the foot stride
     * length.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_feasible_com_velocity()
    {
        return biped_stepper_.get_feasible_com_velocity();
    }

    const Eigen::Vector4d &get_contact_array()
    {
        return contact_array_;
    }

    const Eigen::Vector24d &get_forces()
    {
        return forces_;
    }

private:
    /* User inputs */

    /* Outputs */

    /* Front left foot trajectory. */

    /** @brief The front left foot 3d position. */
    Eigen::Vector3d front_left_foot_position_;

    /** @brief The front left foot 3d velocity. */
    Eigen::Vector3d front_left_foot_velocity_;

    /** @brief The front left foot 3d acceleration. */
    Eigen::Vector3d front_left_foot_acceleration_;

    /* Front right foot trajectory. */

    /** @brief The front right foot 3d position. */
    Eigen::Vector3d front_right_foot_position_;

    /** @brief The front right foot 3d velocity. */
    Eigen::Vector3d front_right_foot_velocity_;

    /** @brief The front right foot 3d acceleration. */
    Eigen::Vector3d front_right_foot_acceleration_;

    /* Hind right foot trajectory. */

    /** @brief The hind right foot 3d position. */
    Eigen::Vector3d hind_right_foot_position_;

    /** @brief The hind right foot 3d velocity. */
    Eigen::Vector3d hind_right_foot_velocity_;

    /** @brief The hind right foot 3d acceleration. */
    Eigen::Vector3d hind_right_foot_acceleration_;

    /* Hind left foot trajectory. */

    /** @brief The hind left foot 3d position. */
    Eigen::Vector3d hind_left_foot_position_;

    /** @brief The hind left foot 3d velocity. */
    Eigen::Vector3d hind_left_foot_velocity_;

    /** @brief The hind left foot 3d acceleration. */
    Eigen::Vector3d hind_left_foot_acceleration_;

    /* Other output */

    /** @brief Biped reactive stepper based on DCM. */
    DcmReactiveStepper biped_stepper_;

    /** @brief Contact array containing which foot are in contact, the order is
     * [FL, FR, HL, HR]. */
    Eigen::Vector4d contact_array_;

    /** @brief Feedforward forces for the swing foot. */
    Eigen::Vector24d forces_;

    /* PLanner settings. */

    /** @brief Position of the front right foot with respect to the base. */
    Eigen::Vector3d fr_offset_;

    /** @brief Position of the front left foot with respect to the base. */
    Eigen::Vector3d fl_offset_;

    /** @brief Position of the hind right foot with respect to the base. */
    Eigen::Vector3d hr_offset_;

    /** @brief Position of the hind left foot with respect to the base. */
    Eigen::Vector3d hl_offset_;

    /** @brief Initial foot height during support. */
    double foot_height_offset_;
};

}  // namespace reactive_planners
