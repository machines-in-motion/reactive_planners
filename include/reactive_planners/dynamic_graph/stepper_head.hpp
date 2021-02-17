/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements an entity to coordinate the stepping for the
 * dcm_vrp_planner.
 */

#pragma once

#include "reactive_planners/dcm_vrp_planner.hpp"

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

namespace reactive_planners
{
namespace dynamic_graph
{
/**
 * @brief Main entity controlling the stepping phase and keeps track of
 *   previously computed variables.
 */
class StepperHead : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new StepperHead object using its Dynamic Graph name.
     *
     * @param name
     */
    StepperHead(const std::string& name);

    /*
     * Input Signals.
     */

    /** @brief The u as computed by the dcm_vrp_planner. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> next_step_location_sin_;

    /** @brief The t_end as computed by the dcm_vrp_planner. */
    dynamicgraph::SignalPtr<double, int> duration_before_step_landing_sin_;

    /*
     * Output Signals.
     */

    /** @brief The time [s] of the current stepping phase. */
    dynamicgraph::SignalTimeDependent<double, int>
        time_from_last_step_touchdown_sout_;

    /** @brief The stepping phase. */
    dynamicgraph::SignalTimeDependent<int, int> is_left_leg_in_contact_sout_;

    /** @brief The old u. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        old_step_location_sout_;

    /** @brief The current u. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        current_step_location_sout_;

    /**
     * @brief Helper to define the name of the signals.
     *
     * @param isInputSignal
     * @param signalType
     * @param signalName
     * @return std::string
     */
    std::string make_signal_string(const bool& is_input_signal,
                                   const std::string& signal_type,
                                   const std::string& signal_name);

protected:
    // Use these variables to determine the current outputs.
    int dg_time_phase_switch_;
    double last_duration_before_step_landing_;
    bool is_left_leg_in_contact_;
    Eigen::Vector3d last_next_step_location_;
    Eigen::Vector3d old_step_location_;
    Eigen::Vector3d current_step_location_;

    /**
     * @brief Callback for time_from_last_step_touchdown_sout_ signal.
     *
     * @param time_last_step
     * @param t
     * @return double&
     */
    double& time_from_last_step_touchdown(double& time_last_step, int time);

    /**
     * @brief Callback for is_left_leg_in_contact_sout_ signal.
     *
     * @param left_leg_in_contact
     * @param t
     * @return double&
     */
    int& is_left_leg_in_contact(int& left_leg_in_contact, int time);

    /**
     * @brief Callback for old_step_location_sout_ signal.
     *
     * @param step_location
     * @param t
     * @return double&
     */
    dynamicgraph::Vector& old_step_location(dynamicgraph::Vector& step_location,
                                            int time);

    /**
     * @brief Callback for current_step_location_sout_ signal.
     *
     * @param step_location
     * @param t
     * @return double&
     */
    dynamicgraph::Vector& current_step_location(
        dynamicgraph::Vector& step_location, int time);
};

}  // namespace dynamic_graph
}  // namespace reactive_planners
