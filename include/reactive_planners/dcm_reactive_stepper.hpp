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

#include "reactive_planners/dcm_vrp_planner.hpp"
#include "reactive_planners/dynamically_consistent_end_effector_trajectory.hpp"
#include "reactive_planners/polynomial_end_effector_trajectory.hpp"
#include "reactive_planners/stepper_head.hpp"

namespace reactive_planners
{
class DcmReactiveStepper
{
public:
    DcmReactiveStepper();

    ~DcmReactiveStepper();

    /**
     * @brief Initialize the reactive stepper for biped.
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
     * @param planner_loop Period of the end-effector trajectory update.
     * @param left_foot_position 3D position of the left foot.
     * @param right_foot_position 3D position of the right foot.
     * @param v_des Desired velocity.
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
        const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &right_foot_position);

    /**
     * @brief Set the nominal steptime.
     */
    void set_steptime_nominal(double t_nom);

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
     * @param base_yaw Current orientation of the base.
     * @param is_closed_loop ????
     * @return bool, true upon success.
     */
    bool run(double time,
             const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &right_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &left_foot_vel,
             const Eigen::Ref<const Eigen::Vector3d> &right_foot_vel,
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
             const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &right_foot_position,
             const Eigen::Ref<const Eigen::Vector3d> &left_foot_vel,
             const Eigen::Ref<const Eigen::Vector3d> &right_foot_vel,
             const Eigen::Ref<const Eigen::Vector3d> &com_position,
             const Eigen::Ref<const Eigen::Vector3d> &com_velocity,
             const pinocchio::SE3 &world_M_base,
             const bool &is_closed_loop);

    /**
     * @brief Start the stepping.
     */
    void start()
    {
        running_ = true;
    }

    /**
     * @brief Stop the stepping.
     */
    void stop()
    {
        running_ = false;
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
        const Eigen::Ref<const Eigen::Vector3d> &desired_com_velocity)
    {
        desired_com_velocity_ = desired_com_velocity;
    }

    void set_feet_pos(
        const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &right_foot_position)
    {
        left_foot_position_ = left_foot_position;
        right_foot_position_ = right_foot_position;
    }

    /**
     * @brief Set the right foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    void set_right_foot_position(
        const Eigen::Ref<const Eigen::Vector3d> &right_foot_position)
    {
        right_foot_position_ = right_foot_position;
    }

    /**
     * @brief Set the right foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    void set_right_foot_velocity(
        const Eigen::Ref<const Eigen::Vector3d> &right_foot_velocity)
    {
        right_foot_velocity_ = right_foot_velocity;
    }

    /**
     * @brief Set the left foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    void set_left_foot_position(
        const Eigen::Ref<const Eigen::Vector3d> &left_foot_position)
    {
        left_foot_position_ = left_foot_position;
    }

    /**
     * @brief Set the left foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    void set_left_foot_velocity(
        const Eigen::Ref<const Eigen::Vector3d> &left_foot_velocity)
    {
        left_foot_velocity_ = left_foot_velocity;
    }

    /**
     * @brief Set the desired center of mass velocity.
     *
     * @param desired_com_velocity
     */
    void dcm_vrp_planner_initialization(const double &l_min,
                                        const double &l_max,
                                        const double &w_min,
                                        const double &w_max,
                                        const double &t_min,
                                        const double &t_max,
                                        const double &l_p,
                                        const double &com_height,
                                        const Eigen::Vector9d &weight)
    {
        dcm_vrp_planner_.initialize(
            l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight);
    }

    /**
     * @brief Set polynomial end effector trajectory.
     */
    void set_polynomial_end_effector_trajectory()
    {
        new_ = false;
    }

    /**
     * @brief Set dynamical end effector trajectory.
     */
    void set_dynamical_end_effector_trajectory()
    {
        new_ = true;
    }

    /*
     * Getters
     */

    /**
     * @brief Get the right foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_right_foot_position()
    {
        return right_foot_position_;
    }

    /**
     * @brief Get the right foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_right_foot_velocity()
    {
        return right_foot_velocity_;
    }

    /**
     * @brief Get the right foot 3d acceleration.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_right_foot_acceleration()
    {
        return right_foot_acceleration_;
    }

    /**
     * @brief Get the left foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_left_foot_position()
    {
        return left_foot_position_;
    }

    /**
     * @brief Get the left foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_left_foot_velocity()
    {
        return left_foot_velocity_;
    }

    /**
     * @brief Get the left foot 3d acceleration.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_left_foot_acceleration()
    {
        return left_foot_acceleration_;
    }

    /**
     * @brief Get the feasible velocity computed from the foot ste stride
     * length.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_feasible_com_velocity()
    {
        return feasible_com_velocity_;
    }

    /**
     * @brief Get the previous support foot position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_previous_support_foot_position()
    {
        return previous_support_foot_position_;
    }

    /**
     * @brief Get the current support foot position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_current_support_foot_position()
    {
        return current_support_foot_position_;
    }

    /**
     * @brief Get the next support foot position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_next_support_foot_position()
    {
        return next_support_foot_position_;
    }

    /**
     * @brief Get the step duration.
     *
     * @return const double&
     */
    const double &get_step_duration() const
    {
        return step_duration_;
    }

    /**
     * @brief Get the time computed from last step touchdown.
     *
     * @return const double&
     */
    const double &get_time_from_last_step_touchdown() const
    {
        return time_from_last_step_touchdown_;
    }

    /**
     * @brief Get if the left foot is in contact, else it is the right foot.
     *
     * @return const bool&
     */
    const bool &get_is_left_leg_in_contact() const
    {
        return is_left_leg_in_contact_;
    }

    /**
     * @brief Get the flying foot position.
     *
     * @return const double&
     */
    const Eigen::Vector3d &get_flying_foot_position() const
    {
        return is_left_leg_in_contact_ ? right_foot_position_
                                       : left_foot_position_;
    }
    /**
     * @brief Get the local foot position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_local_right_foot_position() const
    {
        return local_right_foot_position_;
    }

    /**
     * @brief Get the local right foot velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_local_right_foot_velocity() const
    {
        return local_right_foot_velocity_;
    }

    /**
     * @brief Get the local foot position.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_local_left_foot_position() const
    {
        return local_left_foot_position_;
    }

    /**
     * @brief Get the local left foot velocity.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_local_left_foot_velocity() const
    {
        return local_left_foot_velocity_;
    }

    /**
     * @brief Get dcm.
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */
    Eigen::Ref<const Eigen::Vector3d> get_dcm() const
    {
        return dcm_;
    }

    /**
     * @brief Get forces until the foot land.
     *
     * @return const Eigen::VectorXd&
     */
    Eigen::VectorXd get_forces()
    {
        return forces_.col(0).head(nb_force_);
    }

    /**
     * @brief Get force until the foot land.
     * Don't use it for non biped robot.
     *
     * @return const Eigen::Matrix12,1d&
     */

    const Eigen::Matrix<double, 12, 1> &get_force()
    {
        if (is_left_leg_in_contact_)
            force << 0., 0., 0., 0., 0., 0., forces_.head(3), 0., 0., 0.;
        else
            force << forces_.head(3), 0., 0., 0., 0., 0., 0., 0., 0., 0.;
        return force;
    }

    /**
     * @brief Return true if the stepper is running or if it standing still.
     *
     * @return true if running
     * @return false if standing still.
     */
    bool is_running()
    {
        return running_;
    }

    /*
     * Private methods
     */
private:
    /**
     * @brief Makes the robot walk.
     *
     * @param time
     * @param current_flying_foot_position
     * @param com_position
     * @param com_velocity
     * @param base_yaw
     * @return true
     * @return false
     */
    bool walk(double time,
              const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
              const Eigen::Ref<const Eigen::Vector3d> &right_foot_position,
              const Eigen::Ref<const Eigen::Vector3d> &left_foot_vel,
              const Eigen::Ref<const Eigen::Vector3d> &right_foot_vel,
              const Eigen::Ref<const Eigen::Vector3d> &com_position,
              const Eigen::Ref<const Eigen::Vector3d> &com_velocity,
              pinocchio::SE3 &local_frame,
              const bool &is_closed_loop);

    /**
     * @brief Makes the robot stand still.
     *
     * @param time
     * @param current_flying_foot_position
     * @param com_position
     * @param com_velocity
     * @param base_yaw
     * @return true
     * @return false
     */
    bool stand_still(
        double time,
        const Eigen::Ref<const Eigen::Vector3d> &left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d> &right_foot_position);

private:
    /** @brief Robot control loop period. */
    double control_period_;

    /** @brief Robot planner loop period. */
    double planner_loop_;

    /** @brief Scheduler of the stepping. */
    StepperHead stepper_head_;

    /** @brief Computes the next foot step location in order to avoid a fall and
     * tracking at best a reference CoM velocity. */
    DcmVrpPlanner dcm_vrp_planner_;

    /** @brief Set it to one if you want use the article's trajectory. */
    bool new_;

    /** @brief Computes the end-effector flying trajectory. */
    PolynomialEndEffectorTrajectory polynomial_end_eff_trajectory_;
    DynamicallyConsistentEndEffectorTrajectory
        dynamically_consistent_end_eff_trajectory_;

    /** @brief Is the left foot in contact? otherwise the right foot is. */
    bool is_left_leg_in_contact_;

    /** @brief Duration from the last foot touchdown until the next. */
    double step_duration_;

    /** @brief Time from the last foot touchdown. */
    double time_from_last_step_touchdown_;

    /** @brief Previous support foot. The corresponding foot is now flying. */
    Eigen::Vector3d previous_support_foot_position_;

    /** @brief The current support foot. This foot is in contact. */
    Eigen::Vector3d current_support_foot_position_;

    /** @brief The next upcoming support foot location. */
    Eigen::Vector3d next_support_foot_position_;

    /** @brief The desired center of mass velocity. */
    Eigen::Vector3d desired_com_velocity_;

    /** @brief The right foot 3d position. */
    Eigen::Vector3d right_foot_position_;

    /** @brief The right foot 3d velocity. */
    Eigen::Vector3d right_foot_velocity_;

    /** @brief The right foot 3d acceleration. */
    Eigen::Vector3d right_foot_acceleration_;

    /** @brief The left foot position. */
    Eigen::Vector3d left_foot_position_;

    /** @brief The left foot 3d velocity. */
    Eigen::Vector3d left_foot_velocity_;

    /** @brief The left foot 3d acceleration. */
    Eigen::Vector3d left_foot_acceleration_;

    /** @brief The right foot 3d position with respect to the base in frame
     * parallel to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_right_foot_position_;

    /** @brief The right foot 3d velocity with respect to the base in frame
     * parallel to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_right_foot_velocity_;

    /** @brief The left foot position with respect to the base in frame
     * parallel to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_left_foot_position_;

    /** @brief The left foot 3d velocity with respect to the base in frame
     * parallel to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_left_foot_velocity_;

    /** @brief The feasible center of mass velocity achievable by the robot. */
    Eigen::Vector3d feasible_com_velocity_;

    /** @brief All of forces calculated until the next contact. */
    Eigen::VectorXd forces_;

    /** @brief This frame is located at a the constant CoM height, and at the
     * current CoM XY position. The roll and pitch are null and the yaw comes
     * from the current robot base yaw orientation.
     */
    pinocchio::SE3 local_frame_;

    /** @brief Define if we compute a solution or stay still. */
    bool running_;

    /** @brief The number of acceptable force at forces_. */
    int nb_force_;

    /** @brief The number of acceptable force used at forces_. */
    int nb_usage_of_force_;

    /** @brief Force calculated until the next contact. */
    Eigen::Matrix<double, 12, 1> force;

    /** @brief Nominal DCM computed from the CoM estimation and nominal time. */
    Eigen::Vector3d dcm_;
};

}  // namespace reactive_planners