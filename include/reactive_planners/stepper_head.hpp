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
#include <iostream>

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
    /** @brief Construct a new StepperHead object with simple default paramters.
     * Please call init() in order to setup this class properly. */
    StepperHead();

    void init();

    void set_support_feet_pos(
            Eigen::Ref<const Eigen::Vector3d> previous_support_location,
            Eigen::Ref<const Eigen::Vector3d> current_support_location){
        previous_support_location_ = previous_support_location;
        current_support_location_ = current_support_location;
    }
    void set_support_foot_pos(
            Eigen::Ref<const Eigen::Vector3d> current_support_location){
        current_support_location_ = current_support_location;
    }

    void run(const double &duration_stance_phase,
             const double &duration_swing_phase,
             const Eigen::Vector3d &next_support_location,
             const double &current_time);

    void run(const double &duration_stance_phase,
             const Eigen::Vector3d &next_support_location,
             const double &current_time, bool impact);

    /*
     * Getters.
     */

    /** @brief Get the time from last foot touchdown.
     * @return const double&
     */
    const double &get_time_from_last_step_touchdown()
    {
        return time_from_last_step_touchdown_;
    }

    /** @brief Get if the left foot is in contact. If not then it is the right
     * foot.
     * @return const double&
     */
    const bool &get_is_left_leg_in_contact()
    {
        return is_left_leg_in_contact_;
    }

    /** @brief Get the previous foot step location in the world frame. The
     * previous foot is therfore currently a flying foot.
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

    /** @brief Get if the left foot is in contact. If not then it is the right
     * foot.
     * @return const double&
     */

    Eigen::Vector2d &get_contact_phase()
    {
        return contact_;
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

    /** @brief This is the duration of stance phase. */
    double duration_stance_phase_;

    /** @brief This is the duration of swing phase. */
    double duration_swing_phase_;

    /** @brief Next support location in absolute coordintates. The
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

    /** @brief Current support location in absolute coordintates. The
     * corresponding foot is currently in **contact**. */
    Eigen::Vector3d current_support_location_;

    /*
     * Internal
     */

    /** @brief Time to switch the support foot from left to right. */
    double time_support_switch_;

    /** @brief [is_left_in_contact, is_right_in_contact] */
    Eigen::Vector2d contact_;
};

}  // namespace reactive_planners
