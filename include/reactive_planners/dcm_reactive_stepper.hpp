/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Declare a class that encapsulate the DcmVrpPlanner, the
 * EndEffectorTrajectory3D, and the StepperHead
 */

#pragma once

#include "reactive_planners/dcm_vrp_planner.hpp"
#include "reactive_planners/end_effector_trajectory_3d.hpp"
#include "reactive_planners/new_end_effector_trajectory_3d.hpp"
#include "reactive_planners/stepper_head.hpp"
#include <iostream>

namespace reactive_planners
{
class DcmReactiveStepper
{
public:
    DcmReactiveStepper();

    ~DcmReactiveStepper();

    /**
     * @brief
     *
     * @param is_left_leg_in_contact
     * @param l_min
     * @param l_max
     * @param w_min
     * @param w_max
     * @param t_min
     * @param t_max
     * @param l_p
     * @param com_height
     * @param weight
     * @param mid_air_foot_height
     * @param control_period
     */
    void initialize(const bool &is_left_leg_in_contact,
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
                    Eigen::Ref<const Eigen::Vector3d> v_des);

    /**
     * @brief
     *
     * @param time
     * @param current_flying_foot_position
     * @param com_position
     * @param com_velocity
     * @param base_yaw
     *
     * @return bool, true upon success.
     */
    bool run(double time,
             Eigen::Ref<const Eigen::Vector3d> left_foot_position,
             Eigen::Ref<const Eigen::Vector3d> right_foot_position,
             Eigen::Ref<const Eigen::Vector3d> com_position,
             Eigen::Ref<const Eigen::Vector3d> com_velocity,
             const double &base_yaw,
             Eigen::Ref<const Eigen::Vector2d> contact);
    /**
     * @brief
     *
     * @param time
     * @param current_flying_foot_position
     * @param com_position
     * @param com_velocity
     * @param world_M_base
     *
     * @return bool, true upon success.
     */
    bool run(double time,
             Eigen::Ref<const Eigen::Vector3d> left_foot_position,
             Eigen::Ref<const Eigen::Vector3d> right_foot_position,
             Eigen::Ref<const Eigen::Vector3d> com_position,
             Eigen::Ref<const Eigen::Vector3d> com_velocity,
             const pinocchio::SE3 &world_M_base,
             Eigen::Ref<const Eigen::Vector2d> contact);


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
        Eigen::Ref<const Eigen::Vector3d> desired_com_velocity)
    {
        desired_com_velocity_ = desired_com_velocity;
    }

    set_feet_pos(
            Eigen::Ref<const Eigen::Vector3d> left_foot_position,
            Eigen::Ref<const Eigen::Vector3d> right_foot_position)
    {
        left_foot_position_ = left_foot_position;
        right_foot_position_ = right_foot_position;
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
        dcm_vrp_planner_.initialize(l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight);
    }

    /** @brief Set the costs of x, y, z axes, and hessian regularization.
    *
    * @param cost_x
    * @param cost_y
    * @param cost_z
    * @param hess_regularization
    */
//    void set_end_eff_traj_costs(double cost_x, double cost_y, double cost_z, double hess_regularization){
//        end_eff_traj3d_.set_costs(cost_x, cost_y, cost_z, hess_regularization);
//    }

    /**
     * @brief Set the right foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    void set_right_foot_position(Eigen::Vector3d right_foot_position)
    {
        right_foot_position_ = right_foot_position;
    }

    /**
     * @brief Set the right foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    void set_right_foot_velocity(Eigen::Vector3d right_foot_velocity)
    {
        right_foot_velocity_= right_foot_velocity;
    }

    /**
     * @brief Set the left foot 3d position.
     *
     * @return const Eigen::Vector3d&
     */
    void set_left_foot_position(Eigen::Vector3d left_foot_position)
    {
        left_foot_position_ = left_foot_position;
    }

    /**
     * @brief Set the left foot 3d velocity.
     *
     * @return const Eigen::Vector3d&
     */
    void set_left_foot_velocity(Eigen::Vector3d left_foot_velocity)
    {
        left_foot_velocity_ = left_foot_velocity;
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
    const double &get_step_duration()
    {
        return step_duration_;
    }

    /**
     * @brief Get the time computed from last step touchdown.
     *
     * @return const double&
     */
    const double &get_time_from_last_step_touchdown()
    {
        return time_from_last_step_touchdown_;
    }

    /**
     * @brief Get if the left foot is in contact, else it is the right foot.
     *
     * @return const bool&
     */
    const bool &get_is_left_leg_in_contact()
    {
        return is_left_leg_in_contact_;
    }

    /**
     * @brief Get the flying foot position.
     *
     * @return const double&
     */
    const Eigen::Vector3d &get_flying_foot_position()
    {
        if (is_left_leg_in_contact_)
        {
            return right_foot_position_;
        }
        else
        {
            return left_foot_position_;
        }
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
     * @brief Get the local frame.
     *
     * @return const pinocchio::SE3&
     */
    const pinocchio::SE3 &get_local_frame()
    {
        return local_frame_;
    }

    /**
     * @brief Get dcm.
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */
    Eigen::Ref<const Eigen::Vector3d> &get_dcm() const
    {
        return dcm_;
    }

    /**
     * @brief Get forces until the foot land.
     *
     * @return const Eigen::Vector3d&
     */
    const Eigen::Ref<const Eigen::VectorXd> get_forces()
    {
        return forces_.col(0).head(nb_force_);
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
              Eigen::Ref<const Eigen::Vector3d> left_foot_position,
              Eigen::Ref<const Eigen::Vector3d> right_foot_position,
              Eigen::Ref<const Eigen::Vector3d> com_position,
              Eigen::Ref<const Eigen::Vector3d> com_velocity,
              const pinocchio::SE3 &local_frame,
              Eigen::Ref<const Eigen::Vector2d> contact);

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
    bool stand_still(double time,
                     Eigen::Ref<const Eigen::Vector3d> left_foot_position,
                     Eigen::Ref<const Eigen::Vector3d> right_foot_position);

private:
    /** @brief Robot control period, used when computing the end-effector
     * trajectory. */
    double control_period_;

    /** @brief Scheduler of the stepping. */
    StepperHead stepper_head_;

    /** @brief Computes the next foot step location in order to avoid a fall and
     * tracking at best a reference CoM velocity. */
    DcmVrpPlanner dcm_vrp_planner_;

    bool new_ = true;
    /** @brief Computes the end-effector flying trajectory. */
    EndEffectorTrajectory3D end_eff_traj3d_;
    NewEndEffectorTrajectory3D new_end_eff_traj3d_;

    /** @brief Is the left foot in contact? otherwize the right foot is. */
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
     * parallele to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_right_foot_position_;

    /** @brief The right foot 3d velocity with respect to the base in frame
     * parallele to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_right_foot_velocity_;

    /** @brief The left foot position with respect to the base in frame
     * parallele to the world frame with the base yaw orientation. */
    Eigen::Vector3d local_left_foot_position_;

    /** @brief The left foot 3d velocity with respect to the base in frame
     * parallele to the world frame with the base yaw orientation. */
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

    /** @brief base height - CoM height. */
    double com_base_height_difference_;

    /** @brief True only unit the first iteration is past. */
    bool first_iteration_;

    /** @brief Define if we compute a solution or stay still. */
    bool running_;

    /** @brief The number of acceptable force at forces_. */
    int nb_force_;

    /** @brief Default step width. */
    double l_p_;

    int nb_usage_of_force_;

    /** @brief Nominal DCM computed from the CoM estimation and nominal time. */
    Eigen::Vector3d dcm_;

};

}  // namespace reactive_planners