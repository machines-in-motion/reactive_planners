/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements a class to coordinate the stepping for the dcm_vrp_planner.
 */

#pragma once

#include "Eigen/Eigen"

namespace reactive_planners
{
/**
 * @brief Main entity controlling the stepping phase and keeps track of
 *   previously computed variables.
 */
class StepperHead
{
    /*
     * Public methods.
     */
public:
    /** @brief Construct a new StepperHead object with simple default
     * parameters. Please call init() in order to setup this class properly. */
    StepperHead();

    void set_support_feet_pos(
        const Eigen::Ref<const Eigen::Vector3d> &previous_support_location,
        const Eigen::Ref<const Eigen::Vector3d> &current_support_location)
    {
        previous_support_location_ = previous_support_location;
        current_support_location_ = current_support_location;
    }
    void set_support_foot_pos(
        const Eigen::Ref<const Eigen::Vector3d> &current_support_location)
    {
        current_support_location_ = current_support_location;
    }

    void run(const double &duration_before_foot_landing,
             const Eigen::Vector3d &next_support_location,
             const double &current_time);

    /*
     * Getters.
     */

    /** @brief Get the time from last foot touchdown.
     * @return const double&
     */
    const double &get_time_from_last_step_touchdown() const
    {
        return time_from_last_step_touchdown_;
    }

    /** @brief Get if the left foot is in contact. If not then it is the right
     * foot.
     * @return const double&
     */
    bool &get_is_left_leg_in_contact()
    {
        return is_left_leg_in_contact_;
    }

    /** @brief Get the previous foot step location in the world frame. The
     * previous foot is therefore currently a flying foot.
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_previous_support_location()
    {
        return previous_support_location_;
    }

    /** @brief Get the current step location. The current foot is the one in
     * contact.
     * @return const Eigen::Vector3d&
     */
    const Eigen::Vector3d &get_current_support_location()
    {
        return current_support_location_;
    }

    /*
     * Private methods.
     */
protected:
    /*
     * Private attributes.
     */
protected:
    /*
     * Inputs
     */

    /** @brief This is the duration before the current flying foot needs to
     * land. */
    double duration_before_foot_landing_;

    /** @brief Next support location in absolute coordinates. The
     * corresponding foot is currently **flying**. */
    Eigen::Vector3d next_support_location_;

    /** @brief Current absolute time; */
    double current_time_;

    /*
     * Outputs
     */

    /** @brief Is the duration from the last flying foot touchdown. */
    double time_from_last_step_touchdown_;

    /** @brief Left (true) or right (false) foot is in contact. */
    bool is_left_leg_in_contact_;

    /** @brief Previous support location in absolute coordinates. The
     * corresponding foot is currently **flying**. */
    Eigen::Vector3d previous_support_location_;

    /** @brief Current support location in absolute coordinates. The
     * corresponding foot is currently in **contact**. */
    Eigen::Vector3d current_support_location_;

    /*
     * Internal
     */

    /** @brief Time to switch the support foot from left to right. */
    double time_support_switch_;
};

}  // namespace reactive_planners
