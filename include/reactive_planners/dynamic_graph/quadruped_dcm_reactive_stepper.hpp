/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" step
 * planner for a quadruped.
 */

#pragma once

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>

#include "reactive_planners/dcm_reactive_stepper.hpp"
#include "real_time_tools/mutex.hpp"

namespace reactive_planners
{
namespace dynamic_graph
{
/**
 * @brief Dynamic-graph entity wrapping the QuadrupedDcmReactiveStepper form the
 * reactive_stepping package.
 */
class QuadrupedDcmReactiveStepper : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new QuadrupedDcmReactiveStepper object using its
     * Dynamic Graph name.
     *
     * @param name
     */
    QuadrupedDcmReactiveStepper(const std::string &name);

public:
    /*
     * Input Signals
     */

    /** @brief Current quadruped front left foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_front_left_foot_position_sin_;

    /** @brief Current quadruped front right foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_front_right_foot_position_sin_;

    /** @brief Current quadruped hind left foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_hind_left_foot_position_sin_;

    /** @brief Current quadruped hind right foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_hind_right_foot_position_sin_;

    /** @brief Current quadruped front left foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_front_left_foot_velocity_sin_;

    /** @brief Current quadruped front right foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_front_right_foot_velocity_sin_;

    /** @brief Current quadruped hind left foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_hind_left_foot_velocity_sin_;

    /** @brief Current quadruped hind right foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> current_hind_right_foot_velocity_sin_;

    /** @brief Current quadruped com position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> com_position_sin_;

    /** @brief Current quadruped com velocity in world frame. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> com_velocity_sin_;

    /** @brief Current quadruped base yaw angle. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> base_yaw_sin_;

    /** @brief Current quadruped base SE3 posture. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> xyzquat_base_sin_;

    /** @brief ?????? . */
    dynamicgraph::SignalPtr<double, int> is_closed_loop_sin_;

    /** @brief Desired COM velocity to track with the stepper. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> desired_com_velocity_sin_;

    /*
     * Output Signals
     */
    /** @brief Desired front left foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> front_left_foot_position_sout_;

    /** @brief Desired front right foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> front_right_foot_position_sout_;

    /** @brief Desired hind left foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> hind_left_foot_position_sout_;

    /** @brief Desired hind right foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> hind_right_foot_position_sout_;

    /** @brief Desired front left foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> front_left_foot_velocity_sout_;

    /** @brief Desired front right foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> front_right_foot_velocity_sout_;

    /** @brief Desired hind left foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> hind_left_foot_velocity_sout_;

    /** @brief Desired hind right foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> hind_right_foot_velocity_sout_;

    /** @brief Desired front left foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> front_left_foot_acceleration_sout_;

    /** @brief Desired front right foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> front_right_foot_acceleration_sout_;

    /** @brief Desired hind left foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> hind_left_foot_acceleration_sout_;

    /** @brief Desired hind right foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> hind_right_foot_acceleration_sout_;

    /** @brief Feasible com velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> feasible_com_velocity;

    /** @brief Active endeffector contacts. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> contact_array;

protected:
    /**
     * @brief Callback of the inner_sout_ signal.
     *
     * @param s
     * @param time
     */
    bool &inner(bool &s, int time);

    /**
     * @brief Callback of the front_left_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &front_left_foot_position(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_right_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &front_right_foot_position(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_left_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &hind_left_foot_position(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_right_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &hind_right_foot_position(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_left_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &front_left_foot_velocity(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_right_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &front_right_foot_velocity(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_left_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &hind_left_foot_velocity(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_right_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &hind_right_foot_velocity(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_left_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &front_left_foot_acceleration(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_right_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &front_right_foot_acceleration(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_left_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &hind_left_foot_acceleration(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_right_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &hind_right_foot_acceleration(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_feasible_com_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &feasible_com_velocity(dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the contact_array_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &contact_array(dynamicgraph::Vector& signal_data, int time);

protected:
    /** @brief Inner signal to manage all other signals. */
    dynamicgraph::SignalTimeDependent<bool, int> inner_sout_;

};

}  // namespace dynamic_graph
}  // namespace reactive_planners
